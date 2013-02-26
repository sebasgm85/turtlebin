turtlebin
=========


This tutorial guides you through the process of getting Turtlebin working on your Turtlebot.

* Previous assumes:

	* You have a working Turtlebot running ROS Groovy
	* Your were able to successfully go through the Turtlebot Beginners Tutorials
	* You were able to download and install the following dependencies installed:

		ros-groovy-ros-http-video-streamer 

* Creating catkin workspace:

	cd ~/ 
	mkdir -p ros/src ; cd ros/src/
	catkin_init_workspace 
	Getting packages from sources:

* Getting and building additional neede repositories:

        cd ~/ros/src/

	git clone git@github.com:adamantivm/turtlebin.git src/turtlebin
	git clone https://github.com/willowgarage/task_manager.git 
	git clone https://github.com/RobotWebTools/rosbridge_suite.git 
	
	cd ~/ros ; catkin_make
    

TODO Move this repositories to some more appropiate place


