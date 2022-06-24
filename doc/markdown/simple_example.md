---
title: A simple example
subtitle: ROS2-Pytransitions State Machine Package
author: a.whit (nml@whit.contact)
urlcolor: blue 
colorlinks: true 
---


<!-- License

Copyright 2022 Carnegie Mellon University Neuromechatronics Lab (a.whit)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.

Contact: a.whit (nml@whit.contact)
-->

This example illustrates the basic functionality and usage of the 
``ros_transitions`` package. It assumes that the Python code is executed in a 
properly configured [ROS environment][ros_environment]. See the 
end of this document for instructions for 
[running this example](#doctest).


### Initialize a state machine

The state machine chosen for this example is based on the 
[phase transitions][phase_transitions_wiki] example described in the 
``pytransitions`` [documentation][pytransitions_documentation]. More generally, ``ros_transitions`` should work with any ``pytransitions`` machine.

First, define a simple class to represent physical matter. For now, this class 
does nothing, and will simply be used as a state machine model.[^other_examples]

```python
>>> class Matter:
...     pass
```

Initialize a [pytransitions] state machine by defining a set of states and 
transitions that represent the phases of matter. Set the initial state to 
`solid`.

``` python
>>> import transitions
>>> model = Matter()
>>> states = ['solid', 'liquid', 'gas']
>>> machine = transitions.Machine(model=model, states=states, initial='solid')
>>> machine.add_transition('melt', source='solid', dest='liquid')
>>> machine.add_transition('freeze', source='liquid', dest='solid')
>>> machine.add_transition('vaporize', source='liquid', dest='gas')
>>> machine.add_transition('condense', source='gas', dest='liquid')
```

The combination of ``machine`` and ``model`` is enough to define a 
``pytransitions`` state machine. See the 
[pytransitions documentation][pytransitions_documentation] for further examples 
and usage information.

[^other_examples]: Other examples will use more complex models.

### Associate the statemachine with a ROS2 node

The next step is to make the state machine available to ROS. Initialize ROS and 
create a ``ros_transitions`` [node][ros_node], to wrap ``machine``. This makes 
the state machine available to other nodes on the ROS [graph][ros_graph].

``` python
>>> import rclpy
>>> import ros_transitions
>>> rclpy.init()
>>> node = ros_transitions.Node(machine=machine)
```

That's it. The state machine is now connected to ROS.[^spin_footnote]

[^spin_footnote]: The node still must be [spun][ros_spin] before it can process 
                  ROS messages. Refer to the next section.

### Test the state machine node

What have we accomplished? To observe the ROS state machine in action, we need 
to monitor relevant ROS2 topics and messages. Create a second ROS2 node to listen for state changes. <!-- This is done here 
to illustrate the ROS2 connection between the state machine and other nodes, 
for the purpose of this example. -->

``` python
>>> import rclpy.node
>>> client_node = rclpy.node.Node('client')
```

Subscribe the client node to the `state` topic, and define a callback function 
that will print state transitions, as they happen.[^note_about_events]

``` python
>>> from ros_transitions import state_message
>>> state_callback = lambda m: print(f'State: {m.data}', flush=True)
>>> client_node.create_subscription(msg_type    = state_message, 
...                                 topic       ='machine/state', 
...                                 callback    = state_callback,
...                                 qos_profile = 5);
```

[^note_about_events]: In addition to the `machine/state` topic, a subscription 
                      to the `machine/event` topic could also be initialized.


Test the state machine node by triggering a state transition, and then passing 
control[^spin_once] to the `ros_transitions` node, so that any relevant 
messages may be published to the ROS graph.

[^spin_once]: We pass control of the process to the ROS2 executor -- for a single cycle -- via the [spin_once][rclpy_spin_once] command.

``` python
>>> result = model.melt()
>>> rclpy.spin_once(node, timeout_sec=0.500)
```

Allow the client node to process any ROS messages that it might receive.

``` python
>>> rclpy.spin_once(client_node, timeout_sec=0.500)
State: liquid
```

This shows that a state transition message was sent from the `ros_transitions` 
node to the client node, in order to indicate that the state machine 
transitioned from the `solid` state to the `liquid` state. Trigger another 
transition.

``` python
>>> result = model.vaporize()
>>> rclpy.spin_once(node, timeout_sec=0.500)
>>> rclpy.spin_once(client_node, timeout_sec=0.500)
State: gas
```

Finally, return to the original state.

``` python
>>> result = model.condense()
>>> result = model.freeze()
>>> for n in range(2):
...     rclpy.spin_once(node, timeout_sec=0.500)
...     rclpy.spin_once(client_node, timeout_sec=0.500)
State: liquid
State: solid
```

### Cleanup

To cleanly exit, destroy the nodes and shut down ROS.

``` python
>>> node.destroy_node()
>>> client_node.destroy_node()
>>> rclpy.shutdown()
```

### ROS2 command line utiltiies

It is also possible to monitor the activity of the `ros_transitions` state 
machine using [ROS command line][ros2_command_line_tools] tools, in a 
second [configured environment][ros_environment]. For example, 
the topics created by the node can be listed as follows, at the shell command 
line.

```
> ros2 topic list
/machine/event
/machine/state
/parameter_events
/rosout
```

To print state transition messages, use the `topic echo` command.

```
ros2 topic echo /machine/state
```

Note: If command line tools are preferred, then it is not necessary to create a 
client node in the Python example.


### Doctest

The Markdown source code for this example can be run directly as a [doctest], 
using the following code.

``` python
import doctest
doctest.testfile('simple_example.md')
```


[pytransitions]: https://github.com/pytransitions/transitions

[phase_transitions_wiki]: https://en.wikipedia.org/wiki/State_of_matter#Phase_transitions

[pytransitions_documentation]: https://github.com/pytransitions/transitions#basic-initialization

[ros_node]: https://docs.ros.org/en/humble/Tutorials/Understanding-ROS2-Nodes.html

[ros_graph]: https://docs.ros.org/en/humble/Tutorials/Understanding-ROS2-Nodes.html#the-ros-2-graph

[ros_environment]: https://docs.ros.org/en/humble/Tutorials/Configuring-ROS2-Environment.html

[doctest]: https://docs.python.org/3/library/doctest.html

[rclpy_spin_once]: https://docs.ros2.org/latest/api/rclpy/api/init_shutdown.html#rclpy.spin_once

[ros2_command_line_tools]: https://docs.ros.org/en/humble/Concepts/About-Command-Line-Tools.html

[ros_spin]: https://docs.ros2.org/latest/api/rclpy/api/init_shutdown.html


