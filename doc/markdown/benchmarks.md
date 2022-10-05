---
title: Performance benchmarks
subtitle: ROS2-Pytransitions State Machine Package
date: October 2022
author: a.whit (nml@whit.contact)
---


<!-- License

Copyright 2022 Carnegie Mellon University Neuromechatronics Lab (a.whit)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.

Contact: a.whit (nml@whit.contact)
-->

# Performance benchmarks

In some circumstances, precise timing of state transitions might be required. 
The timing characteristics of the `ros_transitions` package are discussed here, 
and some benchmarks are provided.

A goal of the `ros_transitions` package is to introduce no lag greater than one 
millisecond. However, the package does **not** currently achieve this goal. At 
present, the average lag for the 
[one-shot timer benchmark](#one-shot-timer-benchmark) is **1.3ms** or less. The 
minimum lag is near zero, and the maximum does not exceed 4ms.


## One-shot timer benchmark

The one-shot timer test measures the difference between the measured and 
expected duration of a state that simply waits for a one-shot (ROS) timer to 
expire. The state machine is cyclical, and will transition between an `active` 
and `inactive` state continuously. Each cycle of the state machine has the 
following sequence:

1. The state machine enters the `active` state, and a timeout timer is set. The 
   state machine waits for the timeout timer to expire.
2. A `timeout` event triggers a transition from the state machine from the 
   `active` to the `inactive` state.
3. The `inactive` state is a "pass-through" state, such that a transition to 
   the `active` state is triggered automatically. The transition through the 
   `inactive` state should occur nearly instantaneously.

The benchmarking [code](benchmarks/one_shot_timer.py) sets up a 
`ros_transitions` node based on the above state machine, as well as a client 
node that listens -- via the `machine/event` ROS topic -- for timeout trigger 
events. The client node records a timestamp for each timeout event. In an 
ideal scenario, the difference between these timestamps would exactly equal the 
requested duration of the timeout timer, since the transition through the 
`inactive` state and the resetting of the timer would be instantaneous. The 
benchmark reported here are the differences between the measured 
inter-timeout intervals and the requested timer duration, across some number of 
repeated cycles.

### Results

Commit [123f588]:

| [System ID](#system-characteristics) | Platform | Date | ROS | N | Timeout (ms) | Max $\delta$ (ms) | Min $\delta$ (ms) | Mean $\delta$ (ms) |
| --------- | -------- | ---- | --- | --- | ------------ | -------- | -------- | --------- |
| bt | Linux-5.15.0-47-generic-x86_64-with-glibc2.35 | 221004 | [Humble] | 1000 | 10 | 3.866 | 0.007 | 1.042 |
| bt | Linux-5.15.0-47-generic-x86_64-with-glibc2.35 | 221004 | [Humble] | 1000 | 100 | 2.844 | 0.018 | 1.208 |
| bt | Linux-5.15.0-47-generic-x86_64-with-glibc2.35 | 221004 | [Humble] | 1000 |    5 | 1.947 | 0.062 | 0.995 |
| bt | Linux-5.15.0-47-generic-x86_64-with-glibc2.35 | 221004 | [Humble] | 1000 | 10 | 2.15 | 0.59 | 1.28 |
| bt | Linux-5.15.0-47-generic-x86_64-with-glibc2.35 | 221004 | [Humble] | 1000 | 10 | 3.29 | 0.00 | 1.20 |
| bt | Linux-5.15.0-47-generic-x86_64-with-glibc2.35 | 221004 | [Humble] | 1000 | 10 | 2.16 | 0.13 | 1.24 |

#### System characteristics

| System ID | Processor |  OS  |
| --------- | --------- | ---- |
| bt | [Intel Xeon W-2135] | Ubuntu 22.04.1 LTS |



## Other benchmarks

General ROS2 benchmarking results are also of interest. iRobot provides a 
[ROS2 performance evaluation framework][ros2_performance], and a 
[table of results][ros2_performance_results]. Note the criterion for 
classifying a ROS message as _late_:

> A message is classified as late if... its latency is greater than 
> `min(0.2*period, 5ms)`.



# License

Copyright 2022 [Neuromechatronics Lab][neuromechatronics], 
Carnegie Mellon University

Created by: a. whit. (nml@whit.contact)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.

<!---------------------------------------------------------------------
   References
---------------------------------------------------------------------->

[ros2_performance]: https://github.com/irobot-ros/ros2-performance

[ros2_performance_results]: https://github.com/irobot-ros/ros2-performance/tree/master/irobot_benchmark#evaluation-results

[Intel Xeon W-2135]: https://www.cpubenchmark.net/cpu.php?cpu=Intel+Xeon+W-2135+%40+3.70GHz&id=3121

[ros2_humble]: https://docs.ros.org/en/humble/index.html

[Humble]: https://docs.ros.org/en/humble/index.html

[123f588]: https://github.com/ricmua/ros_transitions/commit/123f58822df791e2ce495a7c08dc4662f91790ff

[neuromechatronics]: https://www.meche.engineering.cmu.edu/faculty/neuromechatronics-lab.html

