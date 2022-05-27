# Turtlebot Tag

## Project Description

_Describe the goal of your project, why it's interesting, what you were able to make your robot do, and what the main components of your project are and how they fit together - please include diagrams and gifs when appropriate_

Our project consists of two robots (a chaser and a runner) playing tag. The runner moves in random directions and maintains that direction for a period of time. The chaser uses a path prediction algorithm to estimate where the runner will be, and catch it at that future location. The chaser achieves this by storing a history of the runner's locations, and uses statistical extrapolation algorithms to predict the path that the runner may be taking. Then, the chaser uses proportional control to move towards the predicted cooreEEEdinate of the runner. Once the chaser tags the runner with its bumper sensor, the chaser will stop.

![1_0_GIF_0](https://user-images.githubusercontent.com/66953378/170405315-794b7e3c-18ed-4998-9fe2-27fd32794203.GIF)
![1_0_GIF_0 2](https://user-images.githubusercontent.com/66953378/170405335-2f934a4d-2aec-46a8-bf0b-03f7f20dbee7.GIF)

Our project is interesting because it is able to solve problems that the person follower code you can't solve. Specifically, our project can use path prediction model to predict the future path of the runner much more accurately and can let the chaser tag the runner even when the chaser is operating at a lower speed than the runner.


## System Architecture

_Describe in detail the robotics algorithm you implemented and each major component of your project, highlight what pieces of code contribute to these main components_

### Runner Detection

We implement our object tracking algorithm in `detect_runner.py` and specifically in the `process_scan()` and `image_callback()` functions. Our runner has 3 different AR tags on each of its sides, allowing the chaser to detect it when close up, and in future extensions of this project, to detect the orientation of the runner. When one or more AR tags are detected in the chaser's camera field of view, we check for an average location of the visible AR tags in the camera (eg. whether it is on the left or right). Using this average location as a guide, we can estimate a LIDAR angle to check for the runner, and we use this measure to get a more precise distance measurement to the runner.

<img width="341" alt="Screen Shot 2022-05-27 at 1 33 52 AM" src="https://user-images.githubusercontent.com/66953378/170644235-cb4b5a60-9745-48ba-8b27-9303b018cc1e.png">

Finally, the estimated distance and angle to the runner robot are published along with a timestamp to the topic `angle_vectors`, to be used by `prediction.py` in the path prediction process.

### Path Prediction

All of the features of path prediction are handled in `prediction.py` and almost all of the functions in that file. Our prediction algorithm involves collecting a history of positions of the runner robot as x and y coordinates, and then using those coordinates and basic statistical modelling approaches to extrapolate a possible path for the robot. In order to collect x & y positions of the runner, we must also track the current x & y position of our chaser robot. We do this by setting its starting location to be the origin, (0,0), and as the chaser moves, we use odometry data to update its stored x and y coordinates of the robot, keeping constant track of its location. This is handled in the callback function to laser scan updates, `scan_callback`. We then use this information, combined with the angle and distance sent by the ardetect.py module, to calculate a coordinate for the runner robot.

The following diagram shows how we're calculating these coordinates from the given information. 

![xy.png](images/xy.png)

In addition to the calculations shown in the above diagram, we also compensate for the rotation of the chaser by combining the angle from chaser to target with the angle between the chaser and the x-axis (robot odom and RVIZ consider robot facing the positive x-axis to be 0 degree rotation).

Once we calculate a coordiante position for the runner, we add it, with a timestamp, to our runner location history array. This array is published to RVIZ as a series of yellow pose arrows so that we can debug the process. Once we have our array of coordinates for the robot, we perform a linear regression on the most recent < 20 points against their timestamps. We feed a future time into our regression model, with the time dependent on our distance from the target robot and our maximum speed, and we set the runner's predicted position at that time to be a target. We also plot this point on RVIZ in green.

### Chaser Movement

Once the predicted coordinate of the runner's future location is known, the chaser uses proportional control to move towards the predicted coordinate of the runner. The code for chaser's movement in implemented in `prediction.py` as well.

### Runner Behavior

The runner moves in straight line, and at a random interval, will rotate to a random angle, and drive forward again. When it encounters an obstacle an obstacles such as a wall, it would attempt to avoid it. The code for the runner is implemented in the `runner.py` file.

### Bumping

The detection of whether the chaser's bumper sensor is touched or not is implemented in the `prediction.py`. In the file, we subscribe to the `sensor_state` topic and inside of our `bumper_callback()` function we detect whether the bumper state has changed or not (ie meaning whether the sensor is touched or not). If it is touched, we set the `self.bumped` flag to true, which will stop the chasing action of the chaser.

## Execution

_Describe how to run your code, e.g., step-by-step instructions on what commands to run in each terminal window to execute your project code._

**Launching the Chaser**:

1. `roscore`
2. SSH into a robot with `ssh pi@192.168.0.???`, then run `set_ip ???` and `bringup`
3. SSH into robot again, this time running `set_ip ???` and `bringup_cam`
4. `rosrun image_transport republish compressed in:=raspicam_node/image raw out:=camera/rgb/image_raw`
5. Finally, run `roslaunch tag_final_project prediction.launch` This launch file will launch the ardetect.py and prediction.py nodes
6. (OPTIONAL) If you wish to launch without RVIZ, you can run `roslaunch tag_final_project prediction_no_rviz.launch` instead of `prediction.launch`

**Launching the Runner**:
1. `roscore`
2. SSH into a robot with `ssh pi@192.168.0.???`, then run `set_ip ???` and `bringup`
3. `rosrun tag_final_project runner.py`

The `drive.py`, `bump_sensor.py`, `test.py` are files we kept for debug purpose only.

## Challenges, Future Work, and Takeaways

_These should take a similar form and structure to how you approached these in the previous projects (1 paragraph each for the challenges and future work and a few bullet points for takeaways)_

### Challenges
When we implement our project, we encounter the following challenges: 
- Due to the lag of the camera, the AR tags on the runner are sometimes not visible to the chaser robot. This will cause us to record some missing travel history points of the runner, which will in turn cause us to be unable to make prediction on the runner's path. Whenever this happens, we will need to move close to csil5 and hope that the router's connection is strong and stable.
- Since the chaser's LiDAR range is only 3 meters, we can't really predict the location of the runner if the runner moves outside of chaser’s LiDAR. Thus, to let our model works, we need to ensure that the runner is within the LiDAR range of the chaser.
- Because we are only able to add a bumper sensor at the front of the chaser, it would sometimes encounter difficulty to ensure the chaser's front side is hitting the runner. Yet, by adding extra card board layers in front of the touch sensor, we are able to expand the surface area of the touch sensor and increase the successful tag rate. 

### Future Work
For the sake of time, we had to make a few concessions to simplify our project, but there are many interesting extensions here that we would've liked to pursue. To detect runner direction, our current approach to predicting the runner's trajectory involves guessing their direction by extrapolating from the history of their past locations, but our approach does not attempt to estimate or use information about the current orientation of the runner. We had discussed a method for doing this that would involve a few smaller AR tags, one on each side of the runner, as well as prediction algorithms that would take advantage of this data, but it quickly grew to beyond the scope of what we could tackle within the timeline. Thus, incorporating the information about the current orientation of the runner in our prediction modle will definitely be something that we want to do in the future.

Additionally, we would definitely want to add more complexity into our project. For example, we would like the chaser to be able to distinguish obstacles from runner. We would also like the implment more non-linear paths prediction (e.g. zig-zagging or Hidden Markov Model) of the runner to see if those would be better than our current linear predicion model. Last but not least, we definitely want to create a full game of tag where runner and chaser can switch role after the chaser tag the runner. This would imply that we need to add bumper sensor on the runner and add AR tags to the chaser in order to implement the full game.

### Takeaways
- We learned how to attach new sensors (e.g. touch sensor) to the OpenCR board
- We learned how to utilize rviz to debug our project code and use it to show the travel history of the runner and the prediction path of the runner
- We learned how to employ a prediction model to predict the future location of a runner given a set of its past travel history coordinates




