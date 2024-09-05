# Cerebra RViz Plugins

This repository is made to be simplify the workflow of using RViz with Avular robots.

The repository provides RViz plugins that display information specific for Avular robots. It also contains an convenience `cerebra.rviz` configuration file to set up RViz to show all information that can be used while developing for Avular robots.

## How to use

### Install in a workspace

In order to install the RViz plugins, you need to take the following steps:

1. Make sure the [Origin description](https://github.com/avular-robotics/avular_origin_description) is available in your workspace (see instructions in that project's README)
2. Clone this repository into your [ROS2 workspace](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
   ```bash
   # in the src folder of your ROS workspace
   git clone https://github.com/avular-robotics/cerebra_rviz_plugins.git
   cd cerebra_rviz_plugins
   git lfs pull
   ```
3. Install dependencies required for the plugins using the provided Debian packages:
   ```bash
   # in the root of this repository
   sudo apt install ./interface_dependencies/autonomy-msgs_amd64_2.2.0.deb ./interface_dependencies/origin-msgs_amd64_1.0.0.deb ./interface_dependencies/cmake-avular_amd64_3.0.0.deb ./interface_dependencies/ament-copyright-avular_amd64_3.0.0.deb
   ```
4. Build and re-source your ROS workspace to make the cerebra_rviz_plugins package available for RViz to use. The commands below are based on the [ROS2 documentation](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#build-the-workspace-with-colcon); your workspace might need a different set of commands in order to build it.
   ```bash
   # In the root directory of your ROS workspace.
   source /opt/ros/humble/setup.bash
   colcon build
   source install/setup.bash
   ```

### Launch default configuration file

Once the RViz plugins are installed, RViz can be launch with the `cerebra.rviz` configuration file using the following command:

```bash
ros2 launch cerebra_rviz_plugins cerebra.launch.py
```
