wsdift
======

catkin workspace for the dift code project

Individual Files
----------------

file      | pkg  | function
--------- | ---  | ----------
joy       | joy  | publishes joystick to joy
joy2turtle.py| haws | topic joy to turtle linear and angular velocities
gui.py  | haws | maps robot and dift to GUI
dift.py | haws | tracks information from inputs/sensors and tags it based on haws metric
haws.py | haws | assigns a metric to information tracked by dift
hasimpy.py | haws | for 'hybrid automata simulation python' provides classes/methods for HA models
racetrack.py | haws | an example of a HA model
