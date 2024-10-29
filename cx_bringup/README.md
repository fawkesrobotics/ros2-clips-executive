# cx_bringup
---
Provides an example bringup mechanism for CX applications.

This package includes configuration files to showcase the different plugins `params` folder.

The CLIPS-Executive environment (CLIPS Environment Manager node) is configurable to specify which CLIPS environments should be loaded at startup and which plugins should be loaded in those environments.

It provides a dedicated node (`cx_node`) inside the `src` directory, which configures the lifecycle manager node per default.

Lastly, it provides a convenient launch file that can be used in order to launch the provided node including common configuration options.

## Launching The CLIPS-Executive
Make sure to source the CLIPS-Executive workspace and run the following command to see the usage of generic ROS Message integration from within CLIPS:
```
ros2 launch cx_bringup cx_launch.py manager_config:=plugin_examples/ros_msgs.yaml
```
