MAZE_HARDWARE PACKAGE:

SSH into the turtlebot, set environmental variables, and launch bringup in one terminal

In a separate terminal set environmental variables once again and then navigate to ENAE450_ws and run the following two commands
colcon build --symlink install
source install/setup.bash

Then to run the code use
ros2 run maze_hardware mazesolver





MAZE_SIMULATION PACKAGE:

Navigate to ENAE450_ws and run the following two commands
colcon build --symlink install
source install/setup.bash

Start tmux with at least two panes and run the following commands in separate windows
ros2 launch maze_simulation simulation.launch.py
ros2 run maze_simulation mazesolver

To change the maze type, open the launch file and comment out unwanted mazes leaving the one you want on lines 40-42 (Default: maze_2)