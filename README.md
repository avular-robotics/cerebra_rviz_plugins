# Cerebra RViz Plugins

This repository is made to be simplify the workflow of using RViz with Avular robots.

The repository provides RViz plugins that display information specific for Avular robots. It also contains an convenience `cerebra.rviz` configuration file to set up RViz to show all information that can be used while developing for Avular robots.

## How to use

In order to use the RViz plugins, you need to take the following steps:

1. Clone this repository into your [ROS2 workspace](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
   ```bash
   git clone https://github.com/avular-robotics/cerebra_rviz_plugins.git
   cd cerebra_rviz_plugins
   git lfs pull
   ```

2. Make sure the [Origin description](https://github.com/avular-robotics/avular_origin_description) is available in your workspace (see instructions in that project's README)
3. Install the message types required for the plugins using the provided Debian packages:
   ```bash
   sudo apt install ./interface_dependencies/autonomy-msgs_amd64_2.2.0.deb ./interface_dependencies/origin-msgs_amd64_1.0.0.deb
   ```

The `cerebra.rviz` configuration file can be used by using the following command:

```bash
rviz2 -d cerebra.rviz
```
