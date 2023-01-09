""" This module defines a ROS2 client [node] for interacting with, and testing, 
    the `Node` class in this package.

[node]: https://docs.ros.org/en/galactic/Tutorials/Understanding-ROS2-Nodes.html

Examples
--------

Initialize a ROS2 interface.

>>> import rclpy
>>> rclpy.init()

Intialize shorthand for spinning nodes.

>>> spin = lambda n: rclpy.spin_once(n, timeout_sec=0.010)

Initialize a client node. Override the logging method to ensure that the output 
is available to the doctest package.

>>> Node.log = lambda self, m: print(m, flush=True)
>>> client_node = Node()

Show the topics created by the client node.

>>> import pprint
>>> topic_map = dict(client_node.get_topic_names_and_types())
>>> pprint.pp(list(topic_map))
['/machine/event',
 '/machine/state',
 '/machine/trigger',
 '/parameter_events',
 '/rosout']

Initialize a ROS2 node that will interact with the client, standing in for a 
`ros_transitions.Node`.

>>> import rclpy.node
>>> test_node = rclpy.node.Node('ros_transitions', namespace='machine')

Initialize a subscriber to receive trigger messages.

>>> kwargs = dict(msg_type=trigger_message,
...               topic='trigger',
...               callback=lambda m: print(f'Trigger: {m.data}'),
...               qos_profile=DEFAULT_QOS)
>>> subscription = test_node.create_subscription(**kwargs)

Initialize publishers to send state and event messages.

>>> create_publisher = test_node.create_publisher
>>> state_publisher = create_publisher(state_message, 'state', DEFAULT_QOS)
>>> event_publisher = create_publisher(event_message, 'event', DEFAULT_QOS)
 
Test the trigger mechanism.

>>> client_node.trigger('melt')
>>> spin(client_node)
>>> spin(test_node)
Trigger: melt

Test the event publisher.

>>> event_publisher.publish(event_message(data='melt'))
>>> spin(test_node)
>>> spin(client_node)
Event: melt

Test the state publisher.

>>> state_publisher.publish(state_message(data='liquid'))
>>> spin(test_node)
>>> spin(client_node)
State: liquid

Cleanup.

>>> test_node.destroy_node()
>>> client_node.destroy_node()
>>> rclpy.shutdown()

"""

# Copyright 2022-2023 Carnegie Mellon University Neuromechatronics Lab (a.whit)
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
# 
# Contact: a.whit (nml@whit.contact)

# Import relevant ROS2 packages.
import rclpy
import rclpy.node

# Define ROS2 message types.
from example_interfaces.msg import String as state_message
from example_interfaces.msg import String as event_message
from example_interfaces.msg import String as trigger_message


# Define the ROS2 Quality-of-Service pre-set profiles.
DEFAULT_QOS = rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value


# Define the node class.
class Node(rclpy.node.Node):
    """ ROS2 node for interacting with a `ros_transitions` node via the 
        [ROS2 graph].
    """
    
    def __init__(self, *args, node_name='client', 
                              namespace='machine', **kwargs):
        super().__init__(*args, node_name=node_name, 
                                namespace=namespace, **kwargs)
        self.events = []
        self.states = []
        self.initialize_publishers()
        self.initialize_subscriptions()
        
    def initialize_publishers(self):
        """ Create publishers for triggering events.
        
        A publisher for the `machine/trigger` topic is initialized.
        
        If a ROS2 namespace is specified in this class' constructor, then 
        `machine` is replaced by that namespace in the topic name.        
        """
        self.trigger_publisher \
          = self.create_publisher(msg_type=trigger_message, 
                                  topic='trigger',
                                  qos_profile=DEFAULT_QOS)
        
    def initialize_subscriptions(self):
        """ Initialize subscriptions for receiving machine event and state 
            updates.
        
        Subscribers for the following topics are initialized:
        
        * `machine/state`
        * `machine/event`
        
        If a ROS2 namespace is specified in this class' constructor, then 
        `machine` is replaced by that namespace in these topic names.
        
        The subscribers use the [system default] [Quality-of-Service] settings.
        
        [system default]: https://docs.ros.org/en/galactic/Concepts
                          /About-Quality-of-Service-Settings.html#qos-profiles
        [Quality-of-Service]: https://docs.ros.org/en/galactic/Concepts
                              /About-Quality-of-Service-Settings.html
        """
        # Create a subscriber for receiving task state change updates.
        self.create_subscription(topic=f'state',
                                 msg_type=state_message,
                                 callback=self._state_callback,
                                 qos_profile=DEFAULT_QOS)
        
        # Create a subscriber for receiving task event / trigger updates.
        self.create_subscription(topic=f'event',
                                 msg_type=state_message,
                                 callback=self._event_callback,
                                 qos_profile=DEFAULT_QOS)
        
    def _state_callback(self, message):
        """ Callback function for state messages. """
        self.states.append(message.data)
        self.log(f'State: {message.data}')
        
    def _event_callback(self, message):
        """ Callback function for event messages. """
        self.events.append(message.data)
        self.log(f'Event: {message.data}')
        
    def trigger(self, key):
        """ Trigger a state transition.
        """
        self.trigger_publisher.publish(trigger_message(data=key))
        
    def log(self, message, severity=None):
        """ Record a message in the ROS log.
        
        Parameters
        ----------
        message : str
            The message to record.
        severity : str
            [ROS2 logging] severity level. Defaults to [INFO].
            
            [ROS2 logging]: https://docs.ros.org/en/galactic/Concepts
                            /About-Logging.html
            [INFO]: https://docs.ros2.org/dashing/api/rclpy/api
                    /logging.html#rclpy.logging.LoggingSeverity
        """
        log = self.get_logger()
        log_function = getattr(log, severity.lower()) if severity else log.info
        log_function(message)
    
  

if __name__ == '__main__':
    import doctest
    option_flags = doctest.ELLIPSIS | doctest.NORMALIZE_WHITESPACE
    doctest.testmod(optionflags=option_flags)

