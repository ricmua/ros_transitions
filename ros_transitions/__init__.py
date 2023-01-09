"""
This package provides an interface between the ROS2 graph_ and a state machine
implemented via the pytransitions_ package.

Conceptually_, a state machine is a model of the behavior of a system. The 
system moves through various *states* via state *transitions*. States and 
transitions may also be associated with *actions* that affect the system.

This package contributes functionality that allows a state machine to act as a 
ROS node -- and to thereby send and receive information via the ROS graph_. 
This offers several advantages:

* Other ROS nodes can (directly or indirectly) trigger state machine 
  transitions;
* State machine transitions can trigger actions by other ROS nodes;
* ROS2 parameter_ management can be used to configure the state machine;
* ROS2 logging_ can be used to save a record of state trajectories and 
  behaviors.
  
.. todo:: Add parameters capability.

.. todo:: Fix logging link.

To use this package, a pytransitions state machine must be initialized by 
defining three data structures:

* A list of states_;
* A set of state transition_ records;
* A model_ that dictates the behavior and actions of the system.

Once defined, use of the package is then as simple as passing these data 
structures to a :py:class:`ros_transitions.Node`. Optionally, a derivative of 
the Node class can be implemented, in order to provide additional functionality 
to the state machine model.

.. _graph: https://docs.ros.org/en/galactic/Tutorials
           /Understanding-ROS2-Nodes.html#the-ros-2-graph
.. _pytransitions: https://github.com/pytransitions
                   /transitions#-transitions
.. _Conceptually: https://en.wikipedia.org/wiki
                  /Finite-state_machine#Concepts_and_terminology
.. _parameter: https://docs.ros.org/en/galactic/Concepts
               /About-ROS-2-Parameters.html
.. _logging: https://docs.ros.org/en/galactic/Tutorials/Ros2bag
             /Recording-And-Playing-Back-Data.html
.. _states: https://github.com/pytransitions/transitions#states
.. _transition: https://github.com/pytransitions/transitions#transitions
.. _model: https://github.com/pytransitions/transitions#basic-initialization
"""

# Copyright 2022-2023 Carnegie Mellon University Neuromechatronics Lab (a.whit)
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
# 
# Contact: a.whit (nml@whit.contact)

from .node import Node
from .msg import state_message
from .msg import event_message
from .msg import trigger_message
from .client import Node as Client

