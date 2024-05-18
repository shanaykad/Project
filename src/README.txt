MAZE_SIMULATION PACKAGE:

Navigate to ENAE450_ws and run the following two commands
colcon build --symlink install
source install/setup.bash

Start tmux with at least two panes and run the following commands in separate windows
ros2 launch maze_simulation simulation.launch.py
ros2 run maze_simulation mazesolver

To change the maze type, open the launch file and comment out unwanted mazes leaving the one you want on lines 40-42 (Default: maze_2)



MAZE_SIMULATION VIDEO:

So there were some issues with recording with gazebo behaving differently with recording software active (OBS), I was able to get the bot to behave normally but I had to lower its angular and linear speeds, with the lowered speeds it was able to complete the maze in 2:43 (see video titled SimulationRun2.43). But with our maximum speeds it could complete the maze in 2:08 (see video titled SimulationRun2.08) although I had to record it on my phone instead.







MAZE_HARDWARE PACKAGE:

SSH into the turtlebot, set environmental variables, and launch bringup in one terminal

In a separate terminal set environmental variables once again and then navigate to ENAE450_ws and run the following two commands
colcon build --symlink install
source install/setup.bash

Then to run the code use
ros2 run maze_hardware mazesolver



BEST COMPETITION RUN:

Our best competition run was in 0:28, see the video titled CompetitionRun0.28



FINAL REPORT:

The report is attached in the ELMS submission as well as in the archive titled FinalReport