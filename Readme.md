# estimate_interest
ROS node for estimating if a person is interested in interacting with the cyborg or not. It uses positional data gathered from kinect, but could in theory work just as well with some adaptation with other sensors that publish the positional data of a person in [x,y,z] coordinates.


The estimate_interest node is made for [ROS Hydro](http://wiki.ros.org/hydro) on Ubuntu 12.04, and probably won't work on other setups. It is written in C++03, as that's what ROS Hydro uses, and trying to use something newer would most likely lead to compatibility problems or mess up Catkin. Use catkin_make to build it.

# Dependencies
* [k2_client](https://github.com/thentnucyborg/k2_cyborg_client)

# Functionality
Classifies people into either: Interested, Indecisive, Hesitating or Not interested depending on their spatial relationship with the Cyborg. 

There are problems with the Kinect spending time establishing tracking on people who are moving, and have trouble picking up people passing it. A persons classification can fluctuate if they make large or sudden movements, or changes position. 

It is therefore advisable to do some filtering of the classification, and make sure the classification is stable before acting on it. People approaching the Cyborg directly from the front are generally correctly classified as interested, while people leaving it are correctly classified as interested. The detector are made to detect interest of people before interaction have been initiated. After interaction have begun the results of the interest detection should no longer be trusted.



# Incomplete nodes
Two nodes that uses the interest classification from this node to give behaviour for the [Trollface](https://www.ntnu.no/wiki/display/cyborg/f.+Trollface) are included, greetingNode and lookingNode. They are currently not functional and set to not compile, but were included to give an example for how the interest detection could be used with the Trollface. greetingNode would make the Trollface greet a person who showed interest in people, or try to encourage them if they are hesitating, while lookingNode would make the Trollface look in the direction of people showing interest.

greetingnode should be mostly functional, but depends on messages defined in the Trollnode. Lookingnode is not functional, and need to subscribe to k2_client and track the location of people so it can know in which direction the person is.




