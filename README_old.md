# Steps to run the code

Running the code in this project takes a several terminal instances. 

For each assignment, please first build the project, and source the setup script on the installation folder as follows:

#### Build and source the ROS packages

Change your directory to `<PROJECT_DIR>/<ASSIGNMENT_DIR>` and run the command to build the project:
```
colcon build
```

On the same directory, source the installation to overlay the built ROS packages by running the command:
```
source install/setup.bash
```

In the following instructions, a terminal called **ROS Sourced Terminal** when it is assumed to have sourced the installation setup script.

#### Build the xeno thread

On a **ROS Sourced Terminal**, Change your directory to `<PROJECT_DIR>/<ASSIGNMENT_DIR>/xeno_thread/` and run the command to create a build directory:
```
mkdir build
```

Change to the build directory and run:
```
cmake ..
```

and build by:
```
make
```

## Assignment 3.2

Under the `assignment3_2` directory:

### ROS Sourced Terminal #1

Run the cam2image node, light position node, sequence controller node, as well as the RELbot simulator node by the command:
```
ros2 launch asdfr_group7 sequence_controller.launch.py
```

### Output

There will be two image frames that display the webcam video feed for each process, and for the node **RELbot_simulator**, the video feed displayed in half dimensions. 

There are multiple topics published by the **sequence_controller** node, the left motor and right motor values are supplied as setpoint motor values, specified by RELbot_simulator node itself and the open loop controller target values are published on a topic called `dest/robot_pose`, which can be queried by running the command in a new **ROS sourced terminal**:
```
ros2 topic echo dest/robot_pose
```

## Assignment 3.3

Under the `assignment3_3` directory:

### ROS Sourced Terminal #1

Run the image node, light position node as well as the sequence controller nodes by the command:
```
ros2 launch asdfr_group7 sequence_controller.launch.py
```

### ROS Sourced Terminal #2

Run the ROS-Xenomai bridge node by the command:
```
bash RosXenoBridge.bash
```

### Terminal #3

Under `src/xeno_thread/build`, run the process that runs on the Xenomai core by the command:
```
sudo ./xenoThread
```

### Output

The communication between ROS2 and Xenomai thread is established, and printed out in each terminal.

The PWM values (by default 0, so the wheels wont move) sent from the sequence controller (for assignment3_3, sequence controller sends PWM values as opposed to angular velocity) can be configured as follows:
```
ros2 param set sequence_controller left_motor 100.0 
```

```
ros2 param set sequence_controller right_motor -100.0 
```

After running these commands, the wheels should rotate such that the robot moves forward. 

## Assignment 3.4

Under the `assignment3_4` directory:

### ROS Sourced Terminal #1

Run the image node, light position node as well as the sequence controller nodes by the command:
```
ros2 launch asdfr_group7 sequence_controller.launch.py
```

### ROS Sourced Terminal #2

Run the ROS-Xenomai bridge node by the command:
```
bash RosXenoBridge.bash
```

### Terminal #3

Under `src/xeno_thread/build`, run the process that runs on the Xenomai core by the command:
```
sudo ./xenoThread
```

### Output

The communication between ROS2 and Xenomai thread is established, and printed out in each terminal.

**By default, the robot moves forward and makes a 90 degree turn.**

**If desired**, it can be manually overridden by the commands below.

#### Move forward test
To test the move forward case, the setpoint sent from the sequence controller can be configured as follows:
```
ros2 param set sequence_controller left_motor 1
```
**AND**
```
ros2 param set sequence_controller right_motor -1
```

After running these commands, the wheels should rotate at 1 deg/s such that the robot moves forward.

#### 90 degree cornering test

To test the 90 degree turn, the setpoint sent from the sequence controller can be configured as follows:
```
ros2 param set sequence_controller left_motor 1
```
**AND**
```
ros2 param set sequence_controller right_motor 1
```

After running these commands, the wheels should rotate at 1 deg/s such that the robot makes a 90 degree turn.

## Assignment 3.5

### ROS Sourced Terminal #1

Run the image node, light position node as well as the sequence controller nodes by the command:
```
ros2 launch asdfr_group7 sequence_controller.launch.py
```

### ROS Sourced Terminal #2

Run the ROS-Xenomai bridge node by the command:
```
bash RosXenoBridge.bash
```

### Terminal #3

Under `src/xeno_thread/build`, run the process that runs on the Xenomai core by the command:
```
sudo ./xenoThread
```

### Output

The robot should track the green object in its field of view. 

Note that the image processing is optimized at the lab using the HSV color space
where the minimum saturation value of 150, which seems to work well in room light but not with sunlight which may adjustments
that luckily can be performed on run time by the command 
```
ros2 param set /lightposition s_lo <VALUE>
```

where <VALUE> can be set between 0 and 255, though the maximum saturation searched in the color space already limited to 255.
In our tests, we saw that:
- a low value of 75-100 works better for sunlight
- 150+ works better in darker/room light lit environments.

To see the filtered image by the light position node, you can run:
 ```
ros2 param set /lightposition debug true
```