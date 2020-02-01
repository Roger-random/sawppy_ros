# ROS software for Sawppy the Rover

## STATUS: Abandoned.
Rhys Mainwaring, a member of the Sawppy builders community, has stepped up to deliver a [full ROS Melodic software stack](https://github.com/srmainwaring/curio). I see no point in duplicating effort.

----

This repository hosts software for driving [Sawppy the Rover](http://sawppy.com) using ROS ([Robot Operating System](http://ros.org)).

This branch is focused on [ROS Melodic Morenia](http://wiki.ros.org/melodic). Functionality on other ROS distributions are not guaranteed.

Installation:
* Install ROS Melodic Morenia [as per instructions](http://wiki.ros.org/melodic/Installation).
* Create ROS workspace [as per instructions](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment#Create_a_ROS_Workspace). (By default `~/catkin_ws`)
* Go into the `/src` subdirectory of your workspace and clone this repository. (By default `~/catkin_ws/src/sawppy_ros`)
* Return to the root of ROS workspace (by default `~/catkin_ws`) and run `catkin_make`
* (More steps to follow once it is actually running...)
