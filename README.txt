# Nearness Diagram - Final Project, Autnomous Robots Course

Authors:
    - Ignacio Herrera Seara, 756920
    - Juan García-Lechuz Sierra, 736161

The package has only been tested on ROS Melodic.

Compilation:
    - Make sure you have the tf2 package installed.
    - $ catkin build nd
    - After compiling make sure that "nd" appears in the nav_core plugin list. $ rospack plugins --attrib=plugin nav_core
    - if you get a tf2::doTransform not found error (or something similar) during execution, try install the tf2_geometry_msgs package.

Execution:
    - $ roslaunch nd navigation_rviz.launch map:=<map>
    - <map>: name of a subfolder of /world containing a stage map. By default the empty map is launched.
        The folder must contain a sample.world file and a sample.yaml file.
    - Make sure that the initial position of the robot in the .launch file matches the position of the robot in the sample.world file.
    - ND parameters can be configured in /config/move_base/planners/nd_local_planner_params.yaml