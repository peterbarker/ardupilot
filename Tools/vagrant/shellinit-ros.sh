# Init that is run every time a new session starts up

# export PATH=$PATH:$HOME/jsbsim/src

echo "ROS environment ready"
echo "ArduPilot: sim_vehicle.py --console --map --mavproxy-args='--out 127.0.0.1:14570' --gdb --debug"
echo "ArduPilot+Gazebo: sim_vehicle.py -f gazebo-iris -D --console --map --mavproxy-args='--out 127.0.0.1:14570 --gdb --debug'"
echo "Gazebo: gazebo --verbose worlds/iris_arducopter_demo.world"
echo "ROS: cd ~/ardupilot_ws/src/launch; roslaunch apm.launch"
