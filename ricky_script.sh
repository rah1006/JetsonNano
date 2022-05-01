#! /bin/bash
# The above line is good practice even though bash is likely the default

echo "Starting Ricky's main script!"
echo "I will open two ROS terminals in the background, then display the messenger's words"
#open terminal 2 and launch the detectnet demo
gnome-terminal -x bash -c "cd catkin_ws; source devel/setup.bash;roslaunch ros_deep_learning detectnet.ros1.launch input:=/dev/video2 output:=display://0;exec $SHELL"

#open terminal 3 to run the talker_RH.py Python script (messenger)
#This takes the detectnet output, processes the data, then publishes the data we care about
gnome-terminal -x bash -c "cd ricky_ws; source devel/setup.bash;rosrun beginner_tutorials talker_RH.py; exec $SHELL"

#Back in terminal 1
#This terminal proves that the data is publishing, so the quad could listen in
echo "Waiting 15 seconds for ROS to open"...
sleep 15
rostopic list
rostopic echo targetData

# Don't forget to chmod +x yourfile.sh when you are finished, to make it executable! -RH
