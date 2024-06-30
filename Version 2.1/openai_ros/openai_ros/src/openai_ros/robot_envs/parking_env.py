import numpy
import rospy
import time
from openai_ros import robot_gazebo_env
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from openai_ros.openai_ros_common import ROSLauncher

class ParkingENV(robot_gazebo_env.RobotGazeboEnv):
    
    def __init__(self, ros_ws_abspath):
        """
        Initializes a new environment.

        Sensor Topic List:
        /my_robot_description/laser/scan

        Actuators Topic List:
        front_left_wheel_revolute_controller
        front_right_wheel_revolute_controller
        back_left_wheel_revolute_controller
        back_right_wheel_revolute_controller
        front_left_wheel_rotate_controller
        front_right_wheel_rotate_controller
        back_left_wheel_rotate_controller
        back_right_wheel_rotate_controller
        """
        rospy.logdebug("Start ParkingENV INIT...")
        # Variables that we give through the constructor.
        # None in this case

        # We launch the ROSlaunch that spawns the robot into the world
        ROSLauncher(rospackage_name="my_robot_description",
                    launch_file_name="env.launch",
                    ros_ws_abspath=ros_ws_abspath)

        # Internal Vars
        # Doesnt have any accesibles
        self.controllers_list = []

        # It doesnt use namespace
        self.robot_name_space = ""

        # We launch the init function of the Parent Class robot_gazebo_env.RobotGazeboEnv
        super(ParkingENV, self).__init__(controllers_list=self.controllers_list,
                                        robot_name_space=self.robot_name_space,
                                        reset_controls=False,
                                        start_init_physics_parameters=False,
                                        reset_world_or_sim="WORLD")

        self.gazebo.unpauseSim()
        #self.controllers_object.reset_controllers()
        self._check_all_sensors_ready()

        # We Start all the ROS related Subscribers and publishers
        rospy.Subscriber("/my_robot_description/laser/scan", LaserScan, self._laser_scan_callback)

        front_left_wheel_revolute_topic  = '/my_robot/front_left_wheel_revolute_controller/command'
        front_right_wheel_revolute_topic = '/my_robot/front_right_wheel_revolute_controller/command'
        back_left_wheel_revolute_topic   = '/my_robot/back_left_wheel_revolute_controller/command'
        back_right_wheel_revolute_topic  = '/my_robot/back_right_wheel_revolute_controller/command'

        front_left_wheel_rotate_topic  = '/my_robot/front_left_wheel_rotate_controller/command'
        front_right_wheel_rotate_topic = '/my_robot/front_right_wheel_rotate_controller/command'
        back_left_wheel_rotate_topic   = '/my_robot/back_left_wheel_rotate_controller/command'
        back_right_wheel_rotate_topic  = '/my_robot/back_right_wheel_rotate_controller/command'

        self.pub_front_left_wheel_revolute  = rospy.Publisher(front_left_wheel_revolute_topic, Float64, queue_size=10)
        self.pub_front_right_wheel_revolute = rospy.Publisher(front_right_wheel_revolute_topic, Float64, queue_size=10)
        self.pub_back_left_wheel_revolute   = rospy.Publisher(back_left_wheel_revolute_topic, Float64, queue_size=10)
        self.pub_back_right_wheel_revolute  = rospy.Publisher(back_right_wheel_revolute_topic, Float64, queue_size=10)

        self.pub_front_left_wheel_rotate  = rospy.Publisher(front_left_wheel_rotate_topic, Float64, queue_size=10)
        self.pub_front_right_wheel_rotate = rospy.Publisher(front_right_wheel_rotate_topic, Float64, queue_size=10)
        self.pub_back_left_wheel_rotate   = rospy.Publisher(back_left_wheel_rotate_topic, Float64, queue_size=10)
        self.pub_back_right_wheel_rotate  = rospy.Publisher(back_right_wheel_rotate_topic, Float64, queue_size=10)

        self._check_publishers_connection()

        self.gazebo.pauseSim()

        rospy.logdebug("Finished ParkingENV INIT...")

    # Methods needed by the RobotGazeboEnv
    # ----------------------------
    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers and other simulation systems are operational.
        """
        self._check_all_sensors_ready()
        return True

    # CubeSingleDiskEnv virtual methods
    # ----------------------------
    def _check_all_sensors_ready(self):
        rospy.logdebug("START ALL SENSORS READY")
        # We dont need to check for the moment, takes too long
        self._check_laser_scan_ready()
        rospy.logdebug("ALL SENSORS READY")

    def _check_laser_scan_ready(self):
        self.laser_scan = None
        rospy.logdebug("Waiting for /my_robot_description/laser/scan to be READY...")
        while self.laser_scan is None and not rospy.is_shutdown():
            try:
                self.laser_scan = rospy.wait_for_message("/my_robot_description/laser/scan", LaserScan, timeout=5.0)
                rospy.logdebug("Current /my_robot_description/laser/scan READY=>")

            except:
                rospy.logerr("Current /my_robot_description/laser/scan not ready yet, retrying for getting laser_scan")
        return self.laser_scan

    def _laser_scan_callback(self, data):
        self.laser_scan = data

    def _check_publishers_connection(self):
        """
        Checks that all the publishers are working
        """
        rate = rospy.Rate(10)  # 10hz
        while self.pub_front_left_wheel_revolute.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug("No susbribers to pub_front_left_wheel_revolute yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("pub_front_left_wheel_revolute Publisher Connected")

        while self.pub_front_right_wheel_revolute.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug("No susbribers to pub_front_right_wheel_revolute yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("pub_front_right_wheel_revolute Publisher Connected")

        while self.pub_back_left_wheel_revolute.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug("No susbribers to pub_back_left_wheel_revolute yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("pub_back_left_wheel_revolute Publisher Connected")

        while self.pub_back_right_wheel_revolute.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug("No susbribers to pub_back_right_wheel_revolute yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("pub_back_right_wheel_revolute Publisher Connected")

        while self.pub_front_left_wheel_rotate.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug("No susbribers to pub_front_left_wheel_rotate yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("pub_front_left_wheel_rotate Publisher Connected")

        while self.pub_front_right_wheel_rotate.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug("No susbribers to pub_front_right_wheel_rotate yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("pub_front_right_wheel_rotate Publisher Connected")

        while self.pub_back_left_wheel_rotate.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug("No susbribers to pub_back_left_wheel_rotate yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("pub_back_left_wheel_rotate Publisher Connected")

        while self.pub_back_right_wheel_rotate.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug("No susbribers to pub_back_right_wheel_rotate yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("pub_back_right_wheel_rotate Publisher Connected")
        rospy.logdebug("All Publishers READY")

    # Methods that the TrainingEnvironment will need to define here as virtual
    # because they will be used in RobotGazeboEnv GrandParentClass and defined in the
    # TrainingEnvironment.
    # ----------------------------
    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        raise NotImplementedError()

    def _init_env_variables(self):
        """Inits variables needed to be initialised each time we reset at the start
        of an episode.
        """
        raise NotImplementedError()

    def _compute_reward(self, observations, done):
        """Calculates the reward to give based on the observations given.
        """
        raise NotImplementedError()

    def _set_action(self, action):
        """Applies the given action to the simulation.
        """
        raise NotImplementedError()

    def _get_obs(self):
        raise NotImplementedError()

    def _is_done(self, observations):
        """Checks if episode done based on observations given.
        """
        raise NotImplementedError()

    # Methods that the TrainingEnvironment will need.
    # ----------------------------
    def move_base(self, speed, angular_speed, angle):
        """
        It will move the base based on the speed and angle given.
        It will wait untill those values are achived.
        param ==> speed: the given speed to the robot to move in rad/s.
        param ==> angle: the given angle for the steering in rad.
        :return:
        """

        # Steering
        self.pub_front_left_wheel_revolute.publish(angle)
        self.pub_front_right_wheel_revolute.publish(angle)
        self.pub_back_left_wheel_revolute.publish(-angle/3)
        self.pub_back_right_wheel_revolute.publish(-angle/3)

        # Moving
        move = speed if angle == 0 else angular_speed
        self.pub_front_left_wheel_rotate.publish(move)
        self.pub_front_right_wheel_rotate.publish(move)
        self.pub_back_left_wheel_rotate.publish(move)
        self.pub_back_right_wheel_rotate.publish(move)

        self._check_publishers_connection()
        time.sleep(0.2)     # 0.02

    def has_crashed(self, min_laser_distance):
        """
        It states based on the laser scan if the robot has crashed or not.
        Crashed means that the minimum laser reading is lower than the
        min_laser_distance value given.
        If min_laser_distance == -1, it returns always false, because its the way
        to deactivate this check.
        """
        robot_has_crashed = False

        if min_laser_distance != -1:
            laser_data = self.get_laser_scan()
            for item in enumerate(laser_data.ranges):
                if item == float ('Inf') or numpy.isinf(item):
                    pass
                elif numpy.isnan(item):
                   pass
                else:
                    # Has a Non Infinite or Nan Value
                    if (item < min_laser_distance):
                        rospy.logerr("ROBOT HAS CRASHED >>> item=" + str(item)+"< "+str(min_laser_distance))
                        robot_has_crashed = True
                        break
        return robot_has_crashed

    def get_laser_scan(self):
        return self.laser_scan

    def reinit_sensors(self):
        """
        This method is for the tasks so that when reseting the episode
        the sensors values are forced to be updated with the real data
        """
