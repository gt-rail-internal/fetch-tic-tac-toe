roslaunch fetch_moveit_config move_group.launch &
cd logic; python gamelogic.py &
cd ../; python vision/vision_ros.py &
cd ../; python ./run.py
