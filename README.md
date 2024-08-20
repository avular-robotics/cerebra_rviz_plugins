# Cerebra RViz Plugins

This repository is made to be simplify the workflow of using RViz with Avular robots.

The repository provides RViz plugins that display information specific for Avular robots. It also contains an example `cerebra.rviz` configuration file to set up RViz to show all information that can be used while developing for Avular robots.

You can use the RViz plugins by cloning this repository into your [ROS2 workspace](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html). To use the example RViz configuration, you can run RViz using `rviz2 -d cerebra.rviz` from this directory. The configuration also uses plugins from the cerebra_rviz_plugins project. The configuration assumes that the [Origin description](https://github.com/avular-robotics/avular_origin_description) is available in your ROS workspace.
