# fetch-tic-tac-toe
Fetch plays Tic Tac Toe! For the Science and Engineering Day 2022 outreach event

## Freedrive on robot
`sudo service robot stop`

`sudo service robot start`

# To use:

Pro tip: Open a tmux session on the robot and split it into 4 quadrants

`tmux new -s <session_name>`

## 1. Launch robot moveit
`roslaunch fetch_moveit_config move_group.launch`

## 2. Run vision (new terminal)
`cd vision`
`python vision_ros.py`

## 3. Run motion module (new terminal)
`python run.py`

## 4. Launch the game logic ROS node (another new terminal)
`cd logic`
`python gamelogic.py`

