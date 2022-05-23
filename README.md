# Nearness Diagram - Final Project, Autnomous Robots Course

Implementation of the Nearness Diagram local planner [[1]](#1) in ROS Melodic.
The results can be seen in this [video](https://www.youtube.com/watch?v=lLzuejBaobA).

## Authors
- Ignacio Herrera Seara, 756920
- Juan García-Lechuz Sierra, 736161

## Compilation
The package has only been tested on ROS Melodic.

Make sure you have the tf2 package installed.

```bash
catkin build nd
```

After compiling make sure that "nd" appears in the nav_core plugin list.

```bash
rospack plugins --attrib=plugin nav_core
```

If you get a tf2::doTransform not found error (or something similar) during execution, try install the tf2_geometry_msgs package.

## Execution

```bash
roslaunch nd navigation_rviz.launch map:=<map>
```

- <map\>: name of a subfolder of /world containing a stage map. By default the empty map is launched.
    The folder must contain a sample.world file and a sample.yaml file.
- ND parameters can be configured in /config/move_base/planners/nd_local_planner_params.yaml

Make sure that the initial position of the robot in the .launch file matches the position of the robot in the sample.world file.

## References
<a id="1">[1]</a> 
Javier Minguez, Luis Montano. (2004).
Nearness diagram (ND) navigation: collision avoidance in troublesome scenarios.
IEEE Transactions on Robotics and Automation.
