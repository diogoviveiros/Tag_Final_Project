# Turtlebot Tag

## Project Description

_Describe the goal of your project, why it's interesting, what you were able to make your robot do, and what the main components of your project are and how they fit together - please include diagrams and gifs when appropriate_

Our project consists of two robots playing tag, a chaser, and a runner. The chaser uses a path prediction algorithm to estimate where the runner will be, and catch it at that future location. The chaser achieves this by storing a history of the runner's locations, and using statistical extrapolation algorithms to predict the path that the runner may be taking.

TODO - finish description


## System Architecture

_Describe in detail the robotics algorithm you implemented and each major component of your project, highlight what pieces of code contribute to these main components_


### Runner Detection

TODO

### Path Prediction

All of the features of path prediction are handled in prediction.py.

Our prediction algorithm involves collecting a history of positions of the runner robot as x and y coordinates, and then using those coordinates and basic statistical modelling approaches to extrapolate a possible path for the robot.

![xy.png](images/xy.png)

As the chaser moves, prediction.py uses odometry data to update the x and y coordinates of the robot, keeping constant track of its location. This is handled in the callback function to laser scan updates, `scan_callback`.

### Movement

TODO

### Runner Behavior

TODO

### Bumping and Switching

TODO

## Execution

_Describe how to run your code, e.g., step-by-step instructions on what commands to run in each terminal window to execute your project code._

**Launching the Chaser**:

1. `roscore`
2. SSH into a robot with `ssh pi@192.168.0.???`, then run `set_ip ???` and `bringup`
3. SSH into robot again, this time running `set_ip ???` and `bringup_cam`
4. `rosrun image_transport republish compressed in:=raspicam_node/image raw out:=camera/rgb/image_raw`
5. Finally, run `roslaunch tag_final_project prediction.launch`

**Launching the Runner**:
TODO

## Challenges, Future Work, and Takeaways

_These should take a similar form and structure to how you approached these in the previous projects (1 paragraph each for the challenges and future work and a few bullet points for takeaways)_

TODO
