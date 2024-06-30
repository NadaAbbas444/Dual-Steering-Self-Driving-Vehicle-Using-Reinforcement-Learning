import rospy
import numpy
import time
import math
from gym import spaces
from openai_ros.robot_envs import parking_env
from gym.envs.registration import register
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from openai_ros.task_envs.task_commons import LoadYamlFileParamsTest
from openai_ros.openai_ros_common import ROSLauncher
import os

class ParkingTaskENV(parking_env.ParkingENV):
    def __init__(self):
        """
        This Task Env is designed for having the robot in some kind of maze.
        It will learn how to move around the maze without crashing.
        """

        # This is the path where the simulation files, the Task and the Robot gits will be downloaded if not there
        ros_ws_abspath = rospy.get_param("/parking/ros_ws_abspath", None)
        assert ros_ws_abspath is not None, "You forgot to set ros_ws_abspath in your yaml file of your main RL script. Set ros_ws_abspath: \'YOUR/SIM_WS/PATH\'"
        assert os.path.exists(ros_ws_abspath), "The Simulation ROS Workspace path " + ros_ws_abspath + \
                                               " DOESNT exist, execute: mkdir -p " + ros_ws_abspath + \
                                               "/src;cd " + ros_ws_abspath + ";catkin_make"

        ROSLauncher(rospackage_name="my_robot_description",
                    launch_file_name="agent.launch",
                    ros_ws_abspath=ros_ws_abspath)

        # Load Params from the desired Yaml file
        LoadYamlFileParamsTest(rospackage_name="openai_ros",
                               rel_path_from_package_to_file="src/openai_ros/task_envs/parking/config",
                               yaml_file_name="parking.yaml")

        # Here we will add any init functions prior to starting the MyRobotEnv
        super(ParkingTaskENV, self).__init__(ros_ws_abspath)

        # Only variable needed to be set here
        number_actions_speed = rospy.get_param('/parking/n_actions_speed')
        number_actions_angle = rospy.get_param('/parking/n_actions_angle')
        self.action_space = spaces.Discrete(numpy.prod([number_actions_speed, number_actions_angle]))

        self.list_of_actions = []
        speeds = [4, 5, 6, 7, 8]
        angles = [-0.5, -0.35, -0.15, 0.0, 0.15, 0.35, 0.5]
        for i in speeds:
            for j in angles:
                self.list_of_actions.append([i, j])

        self.list_of_actions
        # We set the reward range, which is not compulsory but here we do it.
        self.reward_range = (-numpy.inf, numpy.inf)


        #number_observations = rospy.get_param('/parking/n_observations')
        """
        We set the Observation space for the 6 observations
        cube_observations = [
            round(current_disk_roll_vel, 0),
            round(y_distance, 1),
            round(roll, 1),
            round(pitch, 1),
            round(y_linear_speed,1),
            round(yaw, 1),
        ]
        """

        # Actions and Observations
        self.dec_obs = rospy.get_param("/parking/number_decimals_precision_obs", 1)
        self.linear_forward_speed = rospy.get_param('/parking/linear_forward_speed')
        self.steering_angle = rospy.get_param('/parking/steering_angle')
        self.angular_speed = rospy.get_param('/parking/angular_speed')
        self.init_linear_forward_speed = rospy.get_param('/parking/init_linear_forward_speed')
        self.init_steering_angle = rospy.get_param('/parking/init_steering_angle')

        self.n_observations = rospy.get_param('/parking/n_observations')
        self.min_range = rospy.get_param('/parking/min_range')
        self.max_laser_value = rospy.get_param('/parking/max_laser_value')
        self.min_laser_value = rospy.get_param('/parking/min_laser_value')

        # Get Desired Point to Get
        self.desired_point = Point()
        self.desired_point.x = rospy.get_param("/parking/desired_pose/x")
        self.desired_point.y = rospy.get_param("/parking/desired_pose/y")
        self.desired_point.z = rospy.get_param("/parking/desired_pose/z")

        # We create two arrays based on the binary values that will be assigned
        # In the discretization method.
        #laser_scan = self._check_laser_scan_ready()
        laser_scan = self.get_laser_scan()
        rospy.logdebug("laser_scan len===>"+str(len(laser_scan.ranges)))

        # Laser data
        self.laser_scan_frame = laser_scan.header.frame_id

        # Number of laser reading jumped
        self.new_ranges = int(
            math.ceil(float(len(laser_scan.ranges)) / float(self.n_observations)))

        rospy.logdebug("n_observations===>"+str(self.n_observations))
        rospy.logdebug(
            "new_ranges, jumping laser readings===>"+str(self.new_ranges))

        high = numpy.full((self.n_observations), self.max_laser_value)
        low = numpy.full((self.n_observations), self.min_laser_value)

        # We only use two integers
        self.observation_space = spaces.Box(-10, 20, shape=(6,), dtype=numpy.float32)

        rospy.logdebug("ACTION SPACES TYPE===>"+str(self.action_space))
        rospy.logdebug("OBSERVATION SPACES TYPE===>" + str(self.observation_space))

        # Rewards
        self.forwards_reward = rospy.get_param("/parking/forwards_reward")
        self.turn_reward = rospy.get_param("/parking/turn_reward")
        self.end_episode_points = rospy.get_param("/parking/end_episode_points")

        self.cumulated_steps = 0.0
        self.prev_steering_angle = 0.0
        self.laser_filtered_pub = rospy.Publisher(
            '/parking/laser/scan_filtered', LaserScan, queue_size=1)

    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        self.move_base(self.init_linear_forward_speed, self.init_linear_forward_speed, self.init_steering_angle)

        return True

    def _init_env_variables(self):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        :return:
        """
        # For Info Purposes
        self.cumulated_reward = 0.0
        # Set to false Done, because its calculated asyncronously
        self._episode_done = False

        # We wait a small ammount of time to start everything because in very fast resets, laser scan values are sluggish
        # and sometimes still have values from the prior position that triguered the done.
        time.sleep(1.0)

        # TODO: Add reset of published filtered laser readings
        laser_scan = self.get_laser_scan()
        discretized_ranges = laser_scan.ranges
        self.publish_filtered_laser_scan(laser_original_data=laser_scan,
                                         new_filtered_laser_range=discretized_ranges)

        odometry = self.get_odom()
        self.previous_distance_from_des_point = self.get_distance_from_desired_point(odometry.pose.pose.position)

    def _set_action(self, action):
        """
        This set action will Set the linear and angular speed of the turtlebot2
        based on the action number given.
        :param action: The action integer that set s what movement to do next.
        """
        rospy.logdebug("Start Set Action ==>"+str(action))
        # We convert the actions to speed movements to send to the parent class CubeSingleDiskEnv
        linear_speed = self.list_of_actions[action][0]
        angular_speed = linear_speed
        steering_angle = self.list_of_actions[action][1]

        self.last_action = "FORWARDS"
        # We tell TurtleBot2 the linear and angular speed to set to execute
        self.move_base(linear_speed, angular_speed, steering_angle)

        self.delta_steering_angle = steering_angle - self.prev_steering_angle
        self.prev_steering_angle = steering_angle

        rospy.logdebug("END Set Action ==>"+str(action) +
                       ", NAME="+str(self.last_action))

    def _get_obs(self):
        """
        Here we define what sensor data defines our robots observations
        To know which Variables we have acces to, we need to read the API DOCS
        :return:
        """
        rospy.logdebug("Start Get Observation ==>")
        # We get the laser scan data
        laser_scan = self.get_laser_scan()

        rospy.logdebug("BEFORE DISCRET _episode_done==>" +
                       str(self._episode_done))

        discretized_laser_scan = self.discretize_observation(laser_scan, self.new_ranges)

        # We get the odometry so that SumitXL knows where it is.
        odometry = self.get_odom()
        x_position = odometry.pose.pose.position.x
        y_position = odometry.pose.pose.position.y
        z_orient = odometry.pose.pose.orientation.z

        # We round to only two decimals to avoid very big Observation space
        odometry_array = [round(x_position, 2), round(y_position, 2), round(z_orient, 2)]

        # We only want the X and Y position and the Yaw

        observations = discretized_laser_scan + odometry_array

        rospy.logdebug("Observations==>"+str(observations))
        rospy.logdebug("END Get Observation ==>")
        return observations

    def _is_done(self, observations):

        if self._episode_done:
            rospy.logerr("The robot is Too Close to wall==>" +
                           str(self._episode_done))
        else:
            rospy.logerr("The robot hasn't crashed ==>")

            current_position = Point()
            current_position.x = observations[-3]
            current_position.y = observations[-2]
            # current_position.z = observations[-1]

            MAX_X = 4.0
            MIN_X = -1.0
            MAX_Y = 5.0
            MIN_Y = -1.0

            # We see if we are outside the Learning Space

            if current_position.x <= MAX_X and current_position.x > MIN_X:
                if current_position.y <= MAX_Y and current_position.y > MIN_Y:
                    rospy.logdebug("The robot position is OK ==>["+str(current_position.x)+","+str(current_position.y)+"]")

                    # We see if it got to the desired point
                    if self.is_in_desired_position(current_position):
                        self._episode_done = True

                else:
                    rospy.logerr("The robot is too far from Y Pos ==>"+str(current_position.x))
                    self._episode_done = True
            else:
                rospy.logerr("The robot is too far from X Pos ==>"+str(current_position.x))
                self._episode_done = True

        return self._episode_done

    def _compute_reward(self, observations, done):

        factor = min(self.discretized_ranges[:3])

        current_position = Point()
        current_position.x = observations[-3]
        current_position.y = observations[-2]
        # current_position.z = observations[-1]

        distance_from_des_point = self.get_distance_from_desired_point(current_position)
        distance_difference =  distance_from_des_point - self.previous_distance_from_des_point

        if not done:
            if self.last_action == "FORWARDS":
                reward = self.forwards_reward * factor
            else:
                reward = self.turn_reward * factor

            # If there has been a decrease in the distance to the desired point, we reward it
            if distance_difference < 0.0:
                rospy.logwarn("DECREASE IN DISTANCE GOOD")
                reward += self.forwards_reward * 10 / (distance_from_des_point + 0.001)
            else:
                rospy.logerr("ENCREASE IN DISTANCE BAD")
                reward += 0
            
            if -0.25 < self.delta_steering_angle < 0.25:
                reward += 5
            else:
                reward += 0

        else:
            if self.is_in_desired_position(current_position):
                reward = 2000
            else:
                reward = -1 * self.end_episode_points

        self.previous_distance_from_des_point = distance_from_des_point

        rospy.logdebug("reward=" + str(reward))
        self.cumulated_reward += reward
        rospy.logdebug("Cumulated_reward=" + str(self.cumulated_reward))
        self.cumulated_steps += 1
        rospy.logdebug("Cumulated_steps=" + str(self.cumulated_steps))

        return reward

    # Internal TaskEnv Methods
    '''
    def discretize_observation(self, data, new_ranges):
        """
        Discards all the laser readings that are not multiple in index of new_ranges
        value.
        """
        self._episode_done = False

        discretized_ranges = []
        filtered_range = []
        mod = new_ranges

        max_laser_value = data.range_max
        min_laser_value = data.range_min

        rospy.logdebug("data=" + str(data))
        rospy.logwarn("mod=" + str(mod))

        for i, item in enumerate(data.ranges):
            if (i % mod == 0):
                if item == float('Inf') or numpy.isinf(item):
                    # discretized_ranges.append(self.max_laser_value)
                    discretized_ranges.append(
                        round(max_laser_value, self.dec_obs))
                elif numpy.isnan(item):
                    # discretized_ranges.append(self.min_laser_value)
                    discretized_ranges.append(
                        round(min_laser_value, self.dec_obs))
                else:
                    # discretized_ranges.append(int(item))
                    discretized_ranges.append(round(item, self.dec_obs))

                if (self.min_range > item > 0):
                    rospy.logerr("done Validation >>> item=" +
                                 str(item)+"< "+str(self.min_range))
                    self._episode_done = True
                else:
                    rospy.logwarn("NOT done Validation >>> item=" +
                                  str(item)+"> "+str(self.min_range))
                # We add last value appended
                filtered_range.append(discretized_ranges[-1])
            else:
                # We add value zero
                filtered_range.append(0.1)

        rospy.logdebug(
            "Size of observations, discretized_ranges==>"+str(len(discretized_ranges)))

        self.publish_filtered_laser_scan(laser_original_data=data,
                                         new_filtered_laser_range=discretized_ranges)

        return discretized_ranges
        '''

    def discretize_observation(self, data, new_ranges):
        """
        Discards all the laser readings that are not multiple in index of new_ranges
        value.
        """
        self._episode_done = False

        self.discretized_ranges = []
        filtered_range = []
        mod = new_ranges

        max_laser_value = data.range_max
        min_laser_value = data.range_min

        self.discretized_ranges = [round(min(min(data.ranges[0:59]), 15)*5)/5,
                        round(min(min(data.ranges[60:119]), 15)*5)/5,
                        round(min(min(data.ranges[120:179]), 15)*5)/5,
                        # round(min(min(data.ranges[270:359]), 15), 2),
                        ]

        rospy.logdebug("data=" + str(data))
        rospy.logwarn("mod=" + str(mod))
        '''
        for i, item in enumerate(data.ranges):
            if item == float('Inf') or numpy.isinf(item):
                # discretized_ranges.append(self.max_laser_value)
                discretized_ranges.append(
                    round(max_laser_value, self.dec_obs))
            elif numpy.isnan(item):
                # discretized_ranges.append(self.min_laser_value)
                discretized_ranges.append(
                    round(min_laser_value, self.dec_obs))
            else:
                # discretized_ranges.append(int(item))
                discretized_ranges.append(round(item, self.dec_obs))
'''
        for item in self.discretized_ranges:
            if (self.min_range >= item > 0):
                rospy.logerr("done Validation >>> item=" +
                                str(item)+"< "+str(self.min_range))
                self._episode_done = True
            else:
                rospy.logwarn("NOT done Validation >>> item=" +
                                str(item)+"> "+str(self.min_range))
            # We add last value appended
            filtered_range.append(self.discretized_ranges[-1])


        rospy.logdebug(
            "Size of observations, discretized_ranges==>"+str(len(self.discretized_ranges)))

        self.publish_filtered_laser_scan(laser_original_data=data,
                                         new_filtered_laser_range=self.discretized_ranges)

        return self.discretized_ranges

    def publish_filtered_laser_scan(self, laser_original_data, new_filtered_laser_range):

        rospy.logdebug("new_filtered_laser_range==>" +
                       str(new_filtered_laser_range))

        laser_filtered_object = LaserScan()

        h = Header()
        # Note you need to call rospy.init_node() before this will work
        h.stamp = rospy.Time.now()
        h.frame_id = laser_original_data.header.frame_id

        laser_filtered_object.header = h
        laser_filtered_object.angle_min = laser_original_data.angle_min
        laser_filtered_object.angle_max = laser_original_data.angle_max

        new_angle_incr = abs(laser_original_data.angle_max -
                             laser_original_data.angle_min) / len(new_filtered_laser_range)

        #laser_filtered_object.angle_increment = laser_original_data.angle_increment
        laser_filtered_object.angle_increment = new_angle_incr
        laser_filtered_object.time_increment = laser_original_data.time_increment
        laser_filtered_object.scan_time = laser_original_data.scan_time
        laser_filtered_object.range_min = laser_original_data.range_min
        laser_filtered_object.range_max = laser_original_data.range_max

        laser_filtered_object.ranges = []
        laser_filtered_object.intensities = []
        for item in new_filtered_laser_range:
            if item == 0.0:
                laser_distance = 0.1
            else:
                laser_distance = item
            laser_filtered_object.ranges.append(laser_distance)
            laser_filtered_object.intensities.append(item)

        self.laser_filtered_pub.publish(laser_filtered_object)

    def is_in_desired_position(self,current_position, epsilon=0.5):
        """
        It return True if the current position is similar to the desired poistion
        """
        is_in_desired_pos = False

        x_pos_plus = self.desired_point.x + epsilon
        x_pos_minus = self.desired_point.x - epsilon
        y_pos_plus = self.desired_point.y + epsilon
        y_pos_minus = self.desired_point.y - epsilon

        x_current = current_position.x
        y_current = current_position.y

        x_pos_are_close = (x_current <= x_pos_plus) and (x_current > x_pos_minus)
        y_pos_are_close = (y_current <= y_pos_plus) and (y_current > y_pos_minus)

        is_in_desired_pos = x_pos_are_close and y_pos_are_close

        return is_in_desired_pos

    def get_distance_from_desired_point(self, current_position):
        """
        Calculates the distance from the current position to the desired point
        :param start_point:
        :return:
        """
        distance = self.get_distance_from_point(current_position,
                                                self.desired_point)

        return distance

    def get_distance_from_point(self, pstart, p_end):
        """
        Given a Vector3 Object, get distance from current position
        :param p_end:
        :return:
        """
        a = numpy.array((pstart.x, pstart.y))
        b = numpy.array((p_end.x, p_end.y))

        distance = numpy.linalg.norm(a - b)

        return distance