# jackal_control
ROS package to control Jackal mobile robot and its Gazebo simulation.

*** NOT YET UPDATED ***

## 1. Environment
- Ubuntu 20.04, ROS Noetic Ninjemys
  
## 2. Package Documentation

**TurtleBot 3**  
https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/

## 3. Package Installation

- **TurtleBot 3**  
	1.Follow the Quick Start guide from the ROBOTIS website.
	https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/

- **TurtleBot 3 on Gazebo**  
	Follow the simulation instructions from the ROBOTIS website.

- **tb3_control**  
	1. Bring *tb3_control* package into workspace  
		`cd ~/catkin_ws/src`  
		`git clone https://github.com/rnitin/tb3_control.git`
	2. Build catkin workspace directory  
		`cd ~/catkin_ws`
		`catkin_make`
	3. Install python dependencies 
		`pip install -r requirements.txt`

## 4. Executing ROS Packages
- **Waypoint Navigation on TurtleBot 3**  
	Package: *tb3_control*
	1. Start the TurtleBot3  
	2. Execute the python script
	`rosrun tb3_control waypoint.py`  

- **Waypoint Navigation on Gazebo 3**  
	Package: *tb3_control*
	1. Start the TurtleBot3  Gazebo simulation
	2. Execute the python script
	`rosrun tb3_control waypoint.py`  
