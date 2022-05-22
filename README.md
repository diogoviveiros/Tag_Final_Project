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

TODO

### Movement

TODO

### Runner Behavior

TODO

### Bumping and Switching

TODO

## Execution

_Describe how to run your code, e.g., step-by-step instructions on what commands to run in each terminal window to execute your project code._

**Launching the Chaser**:

1. `roslaunch`
2. SSH into a robot with `ssh pi@192.168.0.???`, then run `set_ip ???` and `bringup`
3. SSH into robot again, this time running `set_ip ???` and `bringup_cam`
4. Finally, run `roslaunch Tag_Final_Project prediction.launch`

**Launching the Runner**:
TODO

## Challenges, Future Work, and Takeaways

_These should take a similar form and structure to how you approached these in the previous projects (1 paragraph each for the challenges and future work and a few bullet points for takeaways)_

TODO
