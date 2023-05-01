# bug0

Instructions:
1. CLone the files into the src folder of a catkin workspace.
2. Open one terminal and run 'roscore'
3. Open a new terminal and navigate to /stage/bug-test.world and type the command 'rosrun stage_ros stageros bug-test.world'
4. Open a third terminal and navigate to stage_mover.py
5. RUn the command 'python stage_mover.py goal_x_coordinate goal_y_coordinate'. Eg. python stage_mover.py -1.0 -18.0
