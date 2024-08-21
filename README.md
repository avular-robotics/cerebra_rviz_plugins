# Cerebra RViz Plugins

This repository is made to be simplify the workflow of using RViz with Avular robots.

The repository contains two main projects:

1. cerebra_rviz_plugins
    - This project contains RViz plugins that provide extra information or interaction with Avular robots. These plugins can be used to create a unified robot development view, allowing a user to interact with the internals of a robot in a familiar environment.
2. cerebra_rviz
    - This project builds a Debian package, which can be used to easily launch RViz with a configuration that shows all relevant information on an Avular robot. The configuration can be launched by running the `cerebra-rviz` command, after installation. The configuration also uses plugins from the cerebra_rviz_plugins project. The configuration assumes that the [Origin description](https://github.com/avular-robotics/avular_origin_description) is available in your ROS workspace.
