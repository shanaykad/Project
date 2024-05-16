MAZE_HARDWARE PACKAGE:




MAZE_SIMULATION PACKAGE:

Navigate to ENAE450_ws and run the following two commands
colcon build --symlink install
source install/setup.bash

Start tmux with at least two panes and run the following commands in seperate windows
ros2 launch maze_simulation simulation.launch.py
ros2 run maze_simulation mazesolver

To change the maze type, open the launch file and comment out unwanted mazes leaving the one you want on lines 40-42