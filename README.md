# RG2021_projects

This repository houses the games for the final project of the lecture Robotic Games.
The games are implemented using the [rogata_engine](https://rogata-engine.readthedocs.io/en/latest/what_is_rogata.html) which can be installed by following the tutorials [here](https://rogata-engine.readthedocs.io/en/latest/usage.html)

## Requirements
To run the code, you first need to install the stand-alone version of the [`global_planner`](wiki.ros.org/global_planner) which is normally included in the [`move_base`](http://wiki.ros.org/move_base?distro=noetic) package of the [ROS `navigation` umbrella](http://wiki.ros.org/navigation?distro=noetic).
You can do so by typing `[sudo] apt install ros-noetic-global-planner` in your preferred shell.

## Running the `gruppe2` package
Just include the `gruppe2` launch file in one of or both of the catch launch files and run them.
```
<include file="$(find gruppe2)/launch/gruppe2.launch"/>
```
