1)install ros noetic
2)intall rtabmap-ros package using:
https://github.com/introlab/rtabmap_ros/tree/noetic-devel
	steps in summary are:
		sudo apt install ros-$ROS_DISTRO-rtabmap*
		sudo apt remove ros-$ROS_DISTRO-rtabmap*
	anywhere except the ros workspace directory do:
		cd ~
		git clone https://github.com/introlab/rtabmap.git rtabmap
		cd rtabmap/build
		cmake ..
		make -j6
		sudo make install
	now inside the ros workspace do:
		cd ~/catkin_ws
		git clone https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros
		catkin_make -j4

3)download the bag file that has all data required to run the project:
https://drive.google.com/file/d/176SBeRDatx6JYyyNQzliMLqBfOSAUb1y/view?usp=sharing

4)put the ba file in the folder named 'bags' inside the ros package uploaded named 'final_one'

5)source the workspace and build it.

6)open one terminal and run :
	roscore
7)in another terminal write:
	roslaunch final_one LAUNCH_ALL_2d.launch
now the SLAM part of the project should be running on the recorded data from our robot in our robotics lab.

8)to run the path planning you need to also do the following:
rosrun final_one path_planning_rtabmap2.py
roslaunch final_one test_dynamic.launch
rviz      
	[then select (open config) file and select the file named '2D_SLAM_PP_5.rviz' that is inside the 'final_one' directory]
9) then in rviz window after loading the config file you can click on the button in toolbar named '2D Nav Point' then click on any white region on the occupancy grid then hold the mouse button to also select the orietation, after doing this the global (blue) a local(yellow) paths will be drawn on the map.



