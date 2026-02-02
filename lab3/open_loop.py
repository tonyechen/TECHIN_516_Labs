import os
import time


# TODO: fill in your selected velocity and time values
# Move forward for 1.5m using open-loop strategy.
timenow = time.time()
os.system("ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'")
print("Speed: 0.2, Start timer: 0.0")
# the first pub latches for 3 seconds before continuing with next line
while time.time() - timenow < 10: # TODO: fill in the time_value
    pass
print("Speed 0.0 Stop timer: ", time.time() - timenow)
os.system("ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'")
