wsdift
======

catkin workspace for the dift code project
*** Individual Files:

** Human input from controller (logitech rumbapad 2)
	$rosrun joy joy_node
	http://wiki.ros.org/joy
	publishes topic joy

** From Human input to Robot/Simulation
	$rosrun haws joy2turtle.py
	maps right stick from joystick to turtle 
	linear and angular velocities

** From Robot/Simulation to GUI
	$rosrun haws gui_haws.py

*** Launch Files:

** tele_tsim
	joy, joy2turtle,gui_haws
