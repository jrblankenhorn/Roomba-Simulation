# Roomba-Simulation
Simple Roomba simulation with basic avoidance behavior

Dependencies: ROS Kinetic, OpenCV 3.1

compile with catkin_make

run simulation with roslaunch simulator simulator.launch after properly sourcing

velocity of roomba is in pixels/s, controlled in rqt


max obstacle range refers to how close (in pixels) the roomba must get to an object before attempting to avoid it


eight ultrasonic or IR sensors are simulated, each with 45 degrees of separation. These can be visualized by selecting "show_scan" in rqt
