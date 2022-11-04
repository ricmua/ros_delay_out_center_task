<!-- License

Copyright 2022 Neuromechatronics Lab, Carnegie Mellon University (a.whit)

Contributors:
  a. whit. (nml@whit.contact)

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
-->

## Installation

This package can be added to any [ROS2 workspace].

```bash
git clone https://github.com/ricmua/ros_delay_out_center_task.git path/to/workspace/src/
```

The [delay_out_center_task] project is a [submodule] of this package 
repository, and it therefore must be updated after cloning.

```bash
cd path/to/workspace/src/ros_delay_out_center_task
git submodule update --init
```

ROS2 workspaces are built using [colcon].

```bash
cd path/to/workspace
source path/to/ros/setup.bash
colcon build
```

### Microsoft Windows

The above examples are tailored to Linux installations. The installation 
commands will differ slightly for the Windows Operating system.

The command to initialize a configured ROS2 environment uses the `call` 
function on a `setup.bat`, instead of `source` on `setup.bash`. The path to the 
ROS2 installation will likely also differ.

```
call path/to/ros/setup.bat
```

The `--merge-install` [colcon build flag] is recommended:

> for workspaces with many packages otherwise the environment variables might 
  exceed the supported maximum length.

```
colcon build --merge-install
```

<!---------------------------------------------------------------------
   References
---------------------------------------------------------------------->

[ROS2]: https://docs.ros.org/en/humble/index.html

[ROS2 workspace]: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html

[colcon]: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html

[ros2_build_system]: https://docs.ros.org/en/humble/Concepts/About-Build-System.html

[configure_ros2_environment]: https://docs.ros.org/en/humble/Tutorials/Configuring-ROS2-Environment.html

[build_a_ros2_package]: https://docs.ros.org/en/humble/Tutorials/Creating-Your-First-ROS2-Package.html#build-a-package

[delay_out_center_task]: https://github.com/ricmua/delay_out_center_task

[submodule]: https://git-scm.com/book/en/v2/Git-Tools-Submodules

[colcon build flag]: https://colcon.readthedocs.io/en/released/reference/verb/build.html

