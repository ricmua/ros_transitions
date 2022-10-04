""" Report benchmarks for a `ros_transitions` state machine for which state 
    transitions are governed by a one-shot timer.
"""

# Copyright 2022 Carnegie Mellon University Neuromechatronics Lab (a.whit)
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
# 
# Contact: a.whit (nml@whit.contact)



if __name__ == '__main__':
    
    # Process the command line arguments.
    import argparse
    argument_parser = argparse.ArgumentParser()
    argument_parser.add_argument('--timeout_s', type=float, default=0.010)
    argument_parser.add_argument('--N', type=int, default=1000)
    argument_parser.add_argument('--log_level', type=str, default='WARN')
    argument_parser.add_argument('--id', type=str, default=None)
    arguments = argument_parser.parse_args()
    
    # Define the states of a simple state machine that switches from an 
    # inactive to an active state, and vice-versa.
    states = ['inactive', 'active']
    
    # Define state transitions for the state machine.
    # Define triggers for manually transitioning from one state to another.
    # Also define a timeout trigger, for transitioning to the inactive state.
    state_transitions \
      = [dict(trigger='activate',   source='inactive', dest='active'),
         dict(trigger='deactivate', source='active',   dest='inactive'),
         dict(trigger='timeout',    source='active',   dest='inactive')]
    
    # State machine behavior will be governed by a one-shot timer. 
    # Define lambda functions to act as placeholders, until the timer is 
    # available.
    start_timer = lambda: timer.reset()
    stop_timer = lambda: timer.cancel()
    
    # Define a simple action model for the state machine.
    # These behaviors run the timer during the active state, and trigger 
    # an automatic transition from the inactive state to the active state.
    # This accomplishes a regular, cyclical pattern of active waiting, 
    # interspersed with brief transitions to the inactive state.
    class Model:
        def on_enter_active(self, event_data): start_timer()
        def on_exit_active(self, event_data): stop_timer()
        def on_enter_inactive(self, event_data): self.activate()
    
    # Initialize an instance of the state machine action model.
    model = Model()
    
    # Import the pytransitions package and initialize a state machine.
    import transitions
    machine = transitions.Machine(model=model, 
                                  states=states, 
                                  transitions=state_transitions,
                                  initial='inactive')
    
    # Initialize ROS.
    import rclpy
    rclpy.init()
    
    # Associate the state machine with a ros_transitions node.
    import ros_transitions
    node = ros_transitions.Node(machine=machine)
    
    # Initialize a ROS2 timer.
    timer = node.create_timer(arguments.timeout_s, model.timeout)
    
    # Initialize a client node, to listen for published timeout trigger events.
    import rclpy.node
    client = rclpy.node.Node('client')
    
    # Initialize a new sample buffer. This will hold timestamps corresponding 
    # to timeout trigger events. Collect one more sample than the number 
    # requested, since the requested result is based on the differences between 
    # sample timestamps. This also avoids overhead due to spin-up, prior to the 
    # first sample.
    sample_buffer = [None]*(arguments.N+1)
    
    # Create a future. This is used to indicate when the client has collected 
    # the requested number of samples. The ROS2 process is terminated when this 
    # occurs.
    from rclpy.task import Future
    future = Future()
    
    # Define a sampling function. This is invoked, by the client node, each 
    # time a trigger event is reported (via the `machine/event` ROS2 topic). 
    # This function records the timestamp of the event in the sample buffer.
    # If the buffer is full, then the future result is set, to end sample 
    # collection.
    def event_callback(message):
        index = sample_buffer.index(None)
        sample_buffer[index] = client.get_clock().now().nanoseconds \
                               if message.data == 'timeout' \
                               else None
        if all(sample_buffer): future.set_result(sample_buffer)
    
    # Subscribe the client node to the ROS2 topic via which state machine 
    # trigger events are broadcast. This ensures that the client node can be 
    # aware of any timeouts that occur.
    from example_interfaces.msg import String as EventMessage
    subscription = client.create_subscription(msg_type=EventMessage,
                                              topic='machine/event',
                                              callback=event_callback,
                                              qos_profile=10)
    
    # Set the ROS2 logging level. Setting the level to `INFO` can be useful for 
    # debugging. However, log output should be minimized during benchmarking, 
    # since it will impact the measured results.
    from rclpy.logging import set_logger_level
    log_name = 'machine.node'
    log_level = getattr(rclpy.logging.LoggingSeverity, arguments.log_level)
    set_logger_level(log_name, log_level)
    
    # Initialize a ROS2 executor to spin the two nodes.
    import rclpy.executors
    executor = rclpy.executors.SingleThreadedExecutor()
    success = executor.add_node(node)
    success = executor.add_node(client)
    
    # Activate the state machine cycle and spin until the requested number of 
    # samples have been collected.
    success = model.activate()
    executor.spin_until_future_complete(future)
    success = model.deactivate()
    
    # Ensure the expected outcome. When complete, the future result should be 
    # populated with a list of timeout event timestamps (i.e., the sample 
    # buffer).
    assert future.done()
    assert future.result() == sample_buffer
    
    # Clean up and shut down ROS.
    executor.shutdown()
    node.destroy_node()
    client.destroy_node()
    rclpy.shutdown()
    
    # Compute the number of seconds between buffered sample timestamps.
    # These represent the measured durations of each `active` state epoch.
    time_t0 = sample_buffer[:-1]
    time_tT = sample_buffer[1:]
    elapsed_ns = [(t[1] - t[0]) for t in zip(time_t0, time_tT)]
    
    # Convert the measured epoch durations to seconds.
    elapsed_s = [(t / 1e9) for t in elapsed_ns]
    
    # Compute the difference between the expected and measured epoch durations.
    deviations_s  = [abs(arguments.timeout_s - d) for d in elapsed_s]
    
    # Compute statistics, in milliseconds, for deviations from expectations.
    import numpy
    max_ms     = 1e3 * numpy.max(deviations_s)
    min_ms     = 1e3 * numpy.min(deviations_s)
    mean_ms    = 1e3 * numpy.mean(deviations_s)
    timeout_ms = 1e3 * arguments.timeout_s
    
    # Prepare a data record.
    import platform
    from datetime import datetime
    record = \
      {'System ID': arguments.id if arguments.id else platform.node(),
       'Platform': platform.platform(),
       'Date': datetime.strftime(datetime.now(), '%y%m%d'),
       'ROS': 'Humble',
       'N': f'{arguments.N}',
       'Timeout (ms)': f'{timeout_ms:>4.0f}',
       'Max $\delta$ (ms)': f'{max_ms:0.3f}',
       'Min $\delta$ (ms)': f'{min_ms:0.3f}',
       'Mean $\delta$ (ms)': f'{mean_ms:0.3f}',
      }
    
    # Format the data record as a table in markdown format.
    line_0  = '| '
    line_0 += ' | '.join(record)
    line_0 += ' |'
    line_1  = '| '
    line_1 += ' | '.join(['-' * max(3, len(k)) for k in record])
    line_1 += ' |'
    line_2  = '| '
    line_2 += ' | '.join(record.values())
    line_2 += ' |'
    text = '\n'.join([line_0, line_1, line_2])
    
    # Print the markdown table.
    print(text)

 






