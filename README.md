# AprilTag-Tracker

## Bring up instructions
### PI
To ssh into the pi, connect to the router and go to 192.168.1.1, scroll down and you should see the ip address for
 the pi assuming it's connected
```
# ssh into pi (username: nrmc, password: brnrmc)
ssh nrmc@192.168.1.XXX

# Get into workspace
cd ws/rs
tmux

# Build
catkin_make
source devel/setup.bash

# Setup networking (192.168.1.YYY is the ip address of the main computer running ros)
# The main computer should already have roscore running and the networking script sourced
# First terminal
source src/apriltag_tracker/scripts/network.sh 192.168.1.YYY
rosrun apriltag_tracker apriltag_tracker

# Second terminal
source src/apriltag_tracker/scripts/network.sh 192.168.1.YYY
rosrun static_transform_publisher static_transform_publisher

```
