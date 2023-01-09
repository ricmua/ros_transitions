""" Tests [fixtures] for the [ros_transitions] package that utilize the 
    [pytest] framework.

[fixtures]: https://docs.pytest.org/en/6.2.x/fixture.html
[pytest]: https://docs.pytest.org
[ros_transitions]: https://github.com/ricmua/ros_transitions

Examples
--------

>>> 

"""

# Copyright 2022-2023 Carnegie Mellon University Neuromechatronics Lab (a.whit)
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
# 
# Contact: a.whit (nml@whit.contact)


# Import pytest.
import pytest

# Import ROS.
import rclpy
import rclpy.node

# Import pytransitions.
import transitions

# Import ros_transitions
import ros_transitions


# Initialize a sample pytransitions model.
@pytest.fixture
def model():
    class Matter: pass
    model = Matter()
    yield model
    
  

# Initialize a sample pytransitions state machine.
@pytest.fixture
def machine(model):
    states = ['solid', 'liquid', 'gas']
    machine = transitions.Machine(model=model, states=states, initial='solid')
    machine.add_transition('melt', source='solid', dest='liquid')
    machine.add_transition('freeze', source='liquid', dest='solid')
    machine.add_transition('vaporize', source='liquid', dest='gas')
    machine.add_transition('condense', source='gas', dest='liquid')
    yield machine
    
  

# Initialize a ROS2 interface.
@pytest.fixture(scope='module')
def ros():
    rclpy.init()
    yield rclpy
    rclpy.shutdown()
    
  

# Initialize a ros_transitions node.
@pytest.fixture
def node(ros, machine):
    
    # Wrap the state machine with a ros_transitions node.
    node = ros_transitions.Node(machine=machine)
    
    # Yield the product.
    yield node
    
    # Cleanup.
    node.destroy_node()
    
  

# Set up a client ROS2 node for interacting with the ros_transitions node.
@pytest.fixture
def client(ros):
    
    # Initialize a ROS2 client node.
    client_node = ros_transitions.Client()
    
    # Yield the product.
    yield client_node
    
    # Clean up.
    client_node.destroy_node()
    
  

# Initialize a function for spinning ROS2 nodes once, without blocking.
SPIN_TIMEOUT_SECONDS = 0.005
@pytest.fixture
def spin(ros):
    def spin(node): rclpy.spin_once(node, timeout_sec=SPIN_TIMEOUT_SECONDS)
    yield spin
    
  

# Effect a state transition by triggering an event.
@pytest.fixture
def trigger(node, client, spin):
    
    # Define an event trigger function.
    def trigger(key):
      
      # Publish a trigger request to the ROS2 graph.
      client.trigger(key)
      spin(client)
      
      # Allow the ros_transitions node to process the request.
      spin(node)
      state = node.machine.model.state
      
      # Allow the ros_transitions node to publish the results.
      spin(node)
      spin(client)
      spin(node)
      spin(client)
      client_event = client.events[-1]
      client_state = client.states[-1]
      
      # Return the result record.
      return dict(state=state, 
                  client_state=client_state, 
                  client_event=client_event)
    
    yield trigger
    
  

# __main__
if __name__ == '__main__':
    import doctest
    doctest.testmod()
    
  


