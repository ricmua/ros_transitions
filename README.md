---
title: ROS2-Pytransitions State Machine Package
subtitle: README
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


This [ROS2 package][ros2_package] provides a mechanism for connecting 
[pytransitions] state machines to [ROS2].

A state machine can be [defined][state_machine] as follows:

> [A]n [abstract machine][abstract_machine] that can be in exactly one of a 
  finite number of states at any given time. [A state machine] can change from 
  one state to another in response to some inputs; the change from one state to 
  another is called a _transition_. [A state machine] is defined by a list of 
  its states, its initial state, and the inputs that trigger each transition.

The core functionality of this package is a wrapper that publishes information 
about state transitions and relevant inputs (i.e., triggering events) to the 
ROS2 [graph][ros_graph]. This information can then be used by other ROS2 nodes.
In the context of a ROS2 application, this can be useful for coordinating 
multiple processes with the internal state of a system.

## Getting started

This package can be added to any [ROS2 workspace]. Once the workspace has been 
built, 
the easiest way to get started is probably to review the 
[introductory example](doc/markdown/simple_example.md).

## Testing and benchmarks

Limited [autmated tests](doc/markdown/testing.md) and 
[performance benchmarks](doc/markdown/benchmarks.md) are included with this 
package.

# License

Copyright 2022 [Neuromechatronics Lab], 
Carnegie Mellon University (a.whit)

Maintained by: a. whit. (nml@whit.contact)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.

Please see [LICENSE.txt](./LICENSE.txt).

<!---------------------------------------------------------------------
   References
---------------------------------------------------------------------->


[Neuromechatronics Lab]: https://www.meche.engineering.cmu.edu/faculty/neuromechatronics-lab.html

[ROS2]: https://docs.ros.org/en/humble/index.html

[ros2_package]: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html#what-is-a-ros-2-package

[pytransitions]: https://pypi.org/project/transitions/

[state_machine]: https://en.wikipedia.org/wiki/Finite-state_machine

[ros_graph]: https://docs.ros.org/en/humble/Tutorials/Understanding-ROS2-Nodes.html#the-ros-2-graph

[abstract_machine]: https://en.wikipedia.org/wiki/Abstract_machine

[ROS2 workspace]: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html


