<!-- License

Copyright 2022-2023 Neuromechatronics Lab, Carnegie Mellon University (a.whit)

Contributors:
  a. whit. (nml@whit.contact)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
-->

## Integration

This package consists primarily of code and infrastructure that integrates the 
functionality of several other, stand-alone Python and ROS2 packages. These 
packages include:

* [delay_out_center_task]
* [ros_transitions]
* [ros_spheres_environment]
* [ros_parameter_collections]
* [ros_threading_timer]

The core of the package is a [pytransitions] state machine -- which defines the 
logic and actions of the "delay-out-center" behavioral task (see the 
documentation for [delay_out_center_task] for further information) -- wrapped 
in a [ros_transitions] node, in order to expose it to the [ROS2 graph]. The 
task itself mediates interaction between a subject and a behavioral environment 
that consists of interacting spherical objects, embedded in a three-dimensional 
(3D) virtual space. The [ros_spheres_environment] is integrated into this 
package in order to allow the task to interface with a remote implementation of 
the environment -- again, via the ROS2 graph.

The [ros_parameter_collections] and [ros_threading_timer] packages simply 
facilitate the substitution of ROS2 functionality for Python standard library 
functionality. The former substitutes an interface to [ROS2 parameters] 
for Python [collections]. The latter substitutes the [ROS2 clock and time]rs 
for 


### Testing

In addition to the primary functionality, [integration tests] are useful for 
illustrating the interaction between the various dependent packages. The 
[test_integration.py](test/test_integration.py) document -- along with the 
associated test fixtures in [fixtures.py](test/fixtures.py) -- should be 
especially informative. See the 
[testing documentation](doc/markdown/testing.md) for further information.

<!---------------------------------------------------------------------
   References
---------------------------------------------------------------------->

[ROS2 clock and time]: https://design.ros2.org/articles/clock_and_time.html

[delay_out_center_task]: https://github.com/ricmua/delay_out_center_task

[ros_spheres_environment]: https://github.com/ricmua/ros_spheres_environment

[ros_transitions]: https://github.com/ricmua/ros_transitions

[ros_parameter_collections]: https://github.com/ricmua/ros_parameter_collections

[ros_threading_timer]: https://github.com/ricmua/ros_threading_timer

[pytransitions]: https://github.com/pytransitions/transitions

[integration tests]: https://en.wikipedia.org/wiki/Integration_testing

[ROS2 parameters]: https://docs.ros.org/en/humble/Concepts/About-ROS-2-Parameters.html

[collections]: https://docs.python.org/3/library/collections.html

[ROS2 graph]: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html#background


