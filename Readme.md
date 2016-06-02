# Trollnode
ROS package for communicating with the trollface over TCP.

The Trollnode is made for [ROS Hydro](http://wiki.ros.org/hydro) on Ubuntu 12.04, and probably won't work on other setups. It is written in C++03, as that's what ROS Hydro uses, and trying to use something newer would most likely lead to compatibility problems or mess up Catkin. Use catkin_make to build it.

# Files
The ROS modules are located in the src folder. sendVideo and setExpression are the nodes sending messages  to the Trollserver, while estimateInterest estimates interest. Person.cpp defines classes used by estimateInterest.

The facial expressions used by the Trollserver are defined in /include/trollnode/expression_templates.h

Looking node and greeting node uses estimate interest and setexpression to look at people and greet them if they show interest in the Cyborg, but lookingnode is not functional at the moment.



