# Dual-Steering-Self-Driving-Vehicle-Using-Reinforcement-Learning
This repository is made as a back up for our Graduation Project.

<!------------------------------------- Version 3.1 ------------------------------------->

## Version 3.1:

Parking using PPO.

### How to use

Befor training make sure to move the 4 packages into your `~/catkin_ws/src` then run the following commands.

```
cd ~/catkin_ws
chmod -R 777 src/
```
The above command will make sure that you give the permissoin to all the files in your packages recursively.
Then you need to start rosmaster using `roscore` command
```
roscore
```
In a new tab run the following command to start publishing odometry values to `/odom` topic.

```
roslaunch my_odom_publisher start_odom.launch
```

Now you are ready to start training by running `ppo_train.py`. You can use the following command.

```
python3 ~/catkin_ws/src/my_robot_training/ppo/ppo_train.py
```

<!------------------------------------- Version 3.0 ------------------------------------->

## Version 3.0:

Obstacle avoidance using PPO.

<!------------------------------------- Version 2.1 ------------------------------------->

## Version 2.1:

This is the a modified version for the obstacle avoidance agent.
The first try of this version streamed on 26/6.

### What's new

1. Observation state was enhanced.
2. Robot URDF is updated.
3. Lidar reads clockwise.

### How to use

Start new training using:

```
roslaunch my_robot_training start_training.launch
```

Save the model:

```
rosrun my_robot_training qtable_sub.py
```

### To be updated later

1. Saving the trained data still needs more attention.

<!------------------------------------- Version 2.0 ------------------------------------->

## Version 2.0:

This is the base version for RL agent.

### What's new

This is the first ever try for RL which is not totaly succefull.

### How to use

No need. Use later versions instead.

<!------------------------------------- Version 1.2 ------------------------------------->

## Version 1.2:

### What's new

1. Rear steering
2. The URDF was rewritten to be well-fomated
3. Robot dimensions
4. Inertia values was modified for more realistic motion

### How to use

The same as [Version 1.1](https://github.com/Mu99-M/Dual-Steering-Self-Driving-Vehicle-Using-Reinforcement-Learning-on-ROS#how-to-use).

<!------------------------------------- Version 1.1 ------------------------------------->

## Version 1.1:

### Description

Another package named `motion_plan` was added for the algorithm.

### What's new

1. Laser Scan
2. Brick World
3. Inertia values have been modified for the urdf (Still requires more attention)

### How to use

The commands used are the same as in [Version 1.0](https://github.com/Mu99-M/Dual-Steering-Self-Driving-Vehicle-Using-Reinforcement-Learning-on-ROS#how-to-use-1). Note that you need to move the folder `motion_plan` into the `src` folder in your catkin workspace as well.

For the laser scan readings run this command after spawning the robot.

```
rosrun motion_plan reading_laser.py 
```

### To be updated later

1. Rear steering
2. Dimensions

<!------------------------------------- Version 1.0 ------------------------------------->

## Version 1.0:

### Description

This is the milestone version, it contains the earliest urdf model that could be shown in Gazebo and rviz using the nodes `spawn.launch` and `rviz.launch`.

### How to use
Move the folder named `my_robot_description` into the `src` folder in your catkin workspace and run the following two commands.

```
cd ~/catkin_ws
catkin_make
```

Launch the Gazebo simulation using this command in the terminal.

```
roslaunch my_robot_description spawn.launch
```

In another terminal run this command to start rviz for more interaction with the environment.

```
roslaunch my_robot_description rviz.launch
```

In another terminal run this command to make the robot move. 

```
rosrun my_robot_description main.py 
```

### To be updated later
1. Rear steering
2. Dimensions
3. Input components
