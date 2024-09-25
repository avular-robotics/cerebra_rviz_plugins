# Cerebra RViz Plugins

This repository is made to simplify the workflow of using RViz with Avular robots.

The repository provides RViz plugins that display information specific for Avular robots. It also contains a convenience `cerebra.rviz` configuration file to set up RViz with displays and panels to visualize topics that are available on Avular robots.

## How to use

### Run directly in Docker container

This repository comes with a Dockerfile which makes it easy to run Cerebra RViz, even if you do not have ROS installed on your own computer. [You do need Docker to be installed on your machine](https://docs.docker.com/engine/install). The Dockerfile is set up to connect directly to a robot.


You can build the Docker container using the following command in the root of this repository:

```bash
docker build -t cerebra_rviz .
```

After building the container, you can use the following command to launch Cerebra RViz:

```bash
docker run --rm -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=:0  cerebra_rviz <ip address of robot>
```

This can also be used on Windows, [as long as you have Docker installed with the WSL2 backend](https://docs.docker.com/engine/install) (more on WSL2 can be found [here](https://learn.microsoft.com/en-us/windows/wsl/install)). The launch command is a bit different in this case:

```bash
 docker run --rm -v /run/desktop/mnt/host/wslg/.X11-unix:/tmp/.X11-unix -e DISPLAY=:0 cerebra_rviz <ip address of robot>
```

### Install in a workspace

In order to install the RViz plugins in an existing ROS workspace, you need to take the following steps:

0.
1. Make sure the [Origin description](https://github.com/avular-robotics/avular_origin_description) is available in your ROS workspace
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
   sudo apt install ./interface_dependencies/autonomy-msgs_amd64_2.2.0.deb ./interface_dependencies/origin-msgs_amd64_1.0.0.deb ./interface_dependencies/cmake-avular_amd64_3.0.0.deb ./interface_dependencies/ament-copyright-avular_amd64_3.0.0.deb ros-humble-ament-cmake-clang-format
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

## Tips and Tricks

### View manipulation

The `cerebra.rviz` configuration file supplies two default views, which can easily be selected by clicking on them in the `Views` panel at the right of Cerebra RViz. There is an Orbit view, which orbits the robot's starting point, and there is a ThirdPersonFollower view, which follows behind the robot as it moves around.

### Local map does not show up when the display is enabled

In order to view the Local map, you must set the fixed frame to the `map` frame. If this frame does not exist, there is also no Local map available.
