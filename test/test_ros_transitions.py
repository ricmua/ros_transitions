""" Tests for the [ros_transitions] package that utilize the [pytest] framework.

[pytest]: https://docs.pytest.org
[ros_transitions]: https://github.com/ricmua/ros_transitions

Usage examples: 

`python -m pytest path/to/ros_transitions/test`

`pytest test_package::test_module`

`pytest -k test_sample_state_sequence path/to/test_ros_transitions`

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

# Import fixtures.
from .fixtures import ros
from .fixtures import node
from .fixtures import client
from .fixtures import spin
from .fixtures import model
from .fixtures import machine
from .fixtures import trigger


# Test baseline conditions of the node and client.
def test_baseline(node, client):
    
    # Verify the initial state.
    assert node.machine.model.state == 'solid'
    
    # Verify an empty event list.
    assert client.events == []
    
    # Verify an empty state list.
    assert client.states == []
    
  

# Test initialization of an object in the environment.
def test_state_transition(node, client, spin):
    
    # Publish a trigger request to the ROS2 graph.
    client.trigger('melt')
    spin(client)
    
    # Allow the ros_transitions node to process the request.
    spin(node)
    assert node.machine.model.state == 'liquid'
    
    # Allow the ros_transitions node to publish the results.
    spin(node)
    spin(client)
    spin(node)
    spin(client)
    assert client.events[-1] == 'melt'
    assert client.states[-1] == 'liquid'
    
  

# Test initialization of an object in the environment.
def test_state_sequence(trigger):
    
    # Initialize shorthand.
    def test(event, state):
        expected = dict(state=state, client_state=state, client_event=event)
        assert trigger(event) == expected
    
    # solid->liquid
    test('melt', 'liquid')
    
    # liquid->gas
    test('vaporize', 'gas')
    
    # gas->liquid
    test('condense', 'liquid')
    
    # liquid->solid
    test('freeze', 'solid')
    
  

# __main__
if __name__ == '__main__':
    pytest.main(['test_ros_transitions.py'])
    
  


