This is a very simple drone simulator

Require a "/joy" topic, should use XBox controller and 

rosrun joy joy\_node

other dependency:
ros-indigo-desktop-full, eigen3, boost


--2016-10-Feb--

add roslaunch file 

roslaunch sim\_drone sim\_test\_quad 

Then rviz and joy will automatically started

--2017-16-Aug--

Solved a bug causing drone to topping over

Plan to add a special two rotor drone called KAKA

--2017-17-Aug--

Install robot markers to display moving parts on the drone

https://github.com/jstnhuang/robot_markers

Since ROS kinetic does not contrain robot_markers and robot_markers' dependency transform_graph, we have to instll transform_graph outside in toy_ros_space manually
