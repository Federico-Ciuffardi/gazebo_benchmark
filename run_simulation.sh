cd ~/catkin_ws 
catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1 || exit 1
rqt_console & 
rviz -d /home/fede/catkin_ws/src/gazebo_benchmark/rviz/rviz.rviz & 
roslaunch gazebo_benchmark benchmark.launch  
