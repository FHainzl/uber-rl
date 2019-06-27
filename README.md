# Uber-RL
Connect angle information, robot state and clock, send resulting observation to RL-machine via socket connection.
Receive select actions and forward to robot.

## Run
Run `connector.sh` or `roslaunch uber-rl connector.launch`.
To recover the robot from an error and move it to the start position, run `moveit.sh`.
