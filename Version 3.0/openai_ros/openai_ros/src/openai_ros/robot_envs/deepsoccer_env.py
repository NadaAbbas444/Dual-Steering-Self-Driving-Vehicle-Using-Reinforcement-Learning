import numpy
import rospy
import time
from openai_ros import robot_gazebo_env
from std_msgs.msg import Float64, Float32
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from openai_ros.openai_ros_common import ROSLauncher

from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState
from sensor_msgs.msg import Image
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose

from cv_bridge import CvBridge, CvBridgeError
import cv2


class DeepSoccerEnv(robot_gazebo_env.RobotGazeboEnv):
    """Superclass for all CubeSingleDisk environments.
    """
    def __init__(self, ros_ws_abspath):
        """
        Initializes a new DeepSoccer environment.
        DeepSoccer doesnt use controller_manager, therefore we wont reset the
        controllers in the standard fashion. For the moment we wont reset them.

        To check any topic we need to have the simulations running, we need to do two things:
        1) Unpause the simulation: without that th stream of data doesnt flow. This is for simulations
        that are pause for whatever the reason
        2) If the simulation was running already for some reason, we need to reset the controlers.
        This has to do with the fact that some plugins with tf, dont understand the reset of the simulation
        and need to be reseted to work properly.

        The Sensors: The sensors accesible are the ones considered usefull for AI learning.

        Sensor Topic List:
        * /odom : Odometry readings of the Base of the Robot
        * /camera/depth/image_raw: 2d Depth image of the depth sensor.
        * /camera/depth/points: Pointcloud sensor readings
        * /camera/rgb/image_raw: RGB camera
        * /kobuki/laser/scan: Laser Readings

        Actuators Topic List: /cmd_vel,

        Args:
        """
        rospy.logdebug("Start DeepSoccerEnv INIT...")
        # Variables that we give through the constructor.
        # None in this case

        # We launch the ROSlaunch that spawns the robot into the world
        ROSLauncher(rospackage_name="deepsoccer_gazebo",
                    launch_file_name="main_soccer.launch",
                    ros_ws_abspath=ros_ws_abspath)

        # Internal Vars
        # Doesnt have any accesibles
        self.controllers_list = []

        # It doesnt use namespace
        self.robot_name_space = ""

        # We launch the init function of the Parent Class robot_gazebo_env.RobotGazeboEnv
        super(DeepSoccerEnv, self).__init__(controllers_list=self.controllers_list,
                                            robot_name_space=self.robot_name_space,
                                            reset_controls=False,
                                            start_init_physics_parameters=False,
                                            reset_world_or_sim="WORLD")

        self.gazebo.unpauseSim()
        #self.controllers_object.reset_controllers()
        self._check_all_sensors_ready()

        # We Start all the ROS related Subscribers and publishers
        #rospy.Subscriber("/robot1/camera1/image_raw", Image, self.image_callback)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self._model_state_callback)
        rospy.Subscriber('/robot1/laser/scan', LaserScan, self._laser_scan_callback)
        rospy.Subscriber('/robot1/ir/scan', LaserScan, self._ir_scan_callback)
        rospy.Subscriber("/robot1/camera/image_raw", Image, self._camera_rgb_image_raw_callback)
        #rospy.Subscriber("/odom", Odometry, self._odom_callback)
        #rospy.Subscriber("/camera/depth/image_raw", Image, self._camera_depth_image_raw_callback)
        #rospy.Subscriber("/camera/depth/points", PointCloud2, self._camera_depth_points_callback)
        #rospy.Subscriber("/kobuki/laser/scan", LaserScan, self._laser_scan_callback)

        #self._cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        #rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.wheel1 = rospy.Publisher('/robot1/wheel1_velocity_controller/command', Float64, queue_size=5)
        self.wheel2 = rospy.Publisher('/robot1/wheel2_velocity_controller/command', Float64, queue_size=5)
        self.wheel3 = rospy.Publisher('/robot1/wheel3_velocity_controller/command', Float64, queue_size=5)
        self.wheel4 = rospy.Publisher('/robot1/wheel4_velocity_controller/command', Float64, queue_size=5)
        self.roller = rospy.Publisher('/robot1/roller_velocity_controller/command', Float64, queue_size=5)
        self.stick = rospy.Publisher('/robot1/SolenoidElectromagnet/vel_cmd', Float32, queue_size=5)
        #self.stick = rospy.Publisher('/robot1/stick_velocity_controller/command', Float64, queue_size=5)

        self._check_publishers_connection()

        self.gazebo.pauseSim()

        rospy.logdebug("Finished MyDeepSoccerSingleEnv INIT...")

    def _model_state_callback(self, data):
        self.model_state = data

    # Methods needed by the RobotGazeboEnv
    # ----------------------------
    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers and other simulation systems are
        operational.
        """
        self._check_all_sensors_ready()
        return True

    # CubeSingleDiskEnv virtual methods
    # ----------------------------
    def _check_all_sensors_ready(self):
        rospy.logdebug("START ALL SENSORS READY")
        #self._check_odom_ready()
        # We dont need to check for the moment, takes too long
        #self._check_camera_depth_image_raw_ready()
        self._check_ir_scan_ready()
        self._check_camera_rgb_image_raw_ready()
        self._check_laser_scan_ready()
        rospy.logdebug("ALL SENSORS READY")

    def _check_odom_ready(self):
        self.odom = None
        rospy.logdebug("Waiting for /odom to be READY...")
        while self.odom is None and not rospy.is_shutdown():
            try:
                self.odom = rospy.wait_for_message("/odom", Odometry, timeout=5.0)
                rospy.logdebug("Current /odom READY=>")

            except:
                rospy.logerr("Current /odom not ready yet, retrying for getting odom")

        return self.odom

    def _check_camera_depth_image_raw_ready(self):
        self.camera_depth_image_raw = None
        rospy.logdebug("Waiting for /camera/depth/image_raw to be READY...")
        while self.camera_depth_image_raw is None and not rospy.is_shutdown():
            try:
                self.camera_depth_image_raw = rospy.wait_for_message("/camera/depth/image_raw", Image, timeout=5.0)
                rospy.logdebug("Current /camera/depth/image_raw READY=>")

            except:
                rospy.logerr("Current /camera/depth/image_raw not ready yet, retrying for getting camera_depth_image_raw")

        return self.camera_depth_image_raw

    def _check_camera_depth_points_ready(self):
        self.camera_depth_points = None
        rospy.logdebug("Waiting for /camera/depth/points to be READY...")
        while self.camera_depth_points is None and not rospy.is_shutdown():
            try:
                self.camera_depth_points = rospy.wait_for_message("/camera/depth/points", PointCloud2, timeout=10.0)
                rospy.logdebug("Current /camera/depth/points READY=>")

            except:
                rospy.logerr("Current /camera/depth/points not ready yet, retrying for getting camera_depth_points")

        return self.camera_depth_points

    def _check_camera_rgb_image_raw_ready(self):
        self.camera_rgb_image_raw = None
        rospy.logdebug("Waiting for /robot1/camera/image_raw to be READY...")
        while self.camera_rgb_image_raw is None and not rospy.is_shutdown():
            try:
                #self.camera_rgb_image_raw = rospy.wait_for_message("/camera/rgb/image_raw", Image, timeout=5.0)
                self.camera_rgb_image_raw = rospy.wait_for_message("/robot1/camera/image_raw", Image, timeout=5.0)
                rospy.logdebug("Current /robot1/camera/image_raw READY=>")

            except:
                rospy.logerr("Current /robot1/camera/image_raw not ready yet, retrying for getting camera_rgb_image_raw")

        return self.camera_rgb_image_raw

    def _check_ir_scan_ready(self):
        # rospy.Subscriber('/robot1/ir/scan', LaserScan, self.laser_callback)
        self.laser_scan = None
        rospy.logdebug("Waiting for /robot1/ir/scan to be READY...")
        while self.laser_scan is None and not rospy.is_shutdown():
            try:
                self.laser_scan = rospy.wait_for_message("/robot1/ir/scan", LaserScan, timeout=5.0)
                rospy.logdebug("Current /robot1/ir/scan READY=>")

            except:
                rospy.logerr("Current /robot1/ir/scan not ready yet, retrying for getting laser_scan")

        return self.laser_scan

    def _check_laser_scan_ready(self):
        # rospy.Subscriber('/robot1/laser/scan', LaserScan, self.laser_callback)
        self.laser_scan = None
        rospy.logdebug("Waiting for /robot1/laser/scan to be READY...")
        while self.laser_scan is None and not rospy.is_shutdown():
            try:
                self.laser_scan = rospy.wait_for_message("/robot1/laser/scan", LaserScan, timeout=5.0)
                rospy.logdebug("Current /robot1/laser/scan READY=>")

            except:
                rospy.logerr("Current /robot1/laser/scan not ready yet, retrying for getting laser_scan")

        return self.laser_scan

    def _odom_callback(self, data):
        self.odom = data

    def _camera_depth_image_raw_callback(self, data):
        self.camera_depth_image_raw = data

    def _camera_depth_points_callback(self, data):
        self.camera_depth_points = data

    def _camera_rgb_image_raw_callback(self, data):
        self.camera_rgb_image_raw = data

    def _laser_scan_callback(self, data):
        self.laser_scan = data

    def _ir_scan_callback(self, data):
        self.ir_scan = data

    def _check_publishers_connection(self):
        """
        Checks that all the publishers are working
        :return:
        """

        rate = rospy.Rate(10)  # 10hz
        while self.wheel1.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug("No susbribers to wheel1 yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass

        while self.wheel2.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug("No susbribers to wheel2 yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass

        while self.wheel3.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug("No susbribers to wheel3 yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass

        while self.wheel4.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug("No susbribers to wheel4 yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass

        while self.roller.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug("No susbribers to roller yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass

        while self.stick.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug("No susbribers to stick yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass

        rospy.logdebug("wheel1 Publisher Connected")
        rospy.logdebug("wheel2 Publisher Connected")
        rospy.logdebug("wheel3 Publisher Connected")
        rospy.logdebug("wheel4 Publisher Connected")
        rospy.logdebug("roller Publisher Connected")
        rospy.logdebug("stick Publisher Connected")
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
    def move_base(self, wheel1_speed, wheel2_speed, wheel3_speed, wheel4_speed, roller_speed, stick_speed):
        """
        It will move the base based on the linear and angular speeds given.
        It will wait untill those twists are achived reading from the odometry topic.
        :param linear_speed: Speed in the X axis of the robot base frame
        :param angular_speed: Speed of the angular turning of the robot base frame
        :param epsilon: Acceptable difference between the speed asked and the odometry readings
        :param update_rate: Rate at which we check the odometry.
        :return:
        """
        #rospy.logdebug("DeepSoccer Base Speed Cmd>>" + str(cmd_vel_value))
        self.wheel1.publish(wheel1_speed)
        self.wheel2.publish(wheel2_speed)
        self.wheel3.publish(wheel3_speed)
        self.wheel4.publish(wheel4_speed)
        self.roller.publish(roller_speed)
        self.stick.publish(stick_speed)
        self._check_publishers_connection()
        #self._cmd_vel_pub.publish(cmd_vel_value)
        time.sleep(0.2)
        """
        self.wait_until_twist_achieved(cmd_vel_value,
                                        epsilon,
                                        update_rate,
                                        min_laser_distance)
        """


    def wait_until_twist_achieved(self, cmd_vel_value, epsilon, update_rate, min_laser_distance=-1):
        """
        We wait for the cmd_vel twist given to be reached by the robot reading
        from the odometry.
        :param cmd_vel_value: Twist we want to wait to reach.
        :param epsilon: Error acceptable in odometry readings.
        :param update_rate: Rate at which we check the odometry.
        :return:
        """
        rospy.logwarn("START wait_until_twist_achieved...")

        rate = rospy.Rate(update_rate)
        start_wait_time = rospy.get_rostime().to_sec()
        end_wait_time = 0.0
        epsilon = 0.05

        rospy.logdebug("Desired Twist Cmd>>" + str(cmd_vel_value))
        rospy.logdebug("epsilon>>" + str(epsilon))

        linear_speed = cmd_vel_value.linear.x
        angular_speed = cmd_vel_value.angular.z

        linear_speed_plus = linear_speed + epsilon
        linear_speed_minus = linear_speed - epsilon
        angular_speed_plus = angular_speed + epsilon
        angular_speed_minus = angular_speed - epsilon
        while not rospy.is_shutdown():
            crashed_into_something = self.has_crashed(min_laser_distance)

            current_odometry = self._check_odom_ready()
            odom_linear_vel = current_odometry.twist.twist.linear.x
            odom_angular_vel = current_odometry.twist.twist.angular.z

            rospy.logdebug("Linear VEL=" + str(odom_linear_vel) + ", ?RANGE=[" + str(linear_speed_minus) + "," + str(linear_speed_plus) + "]")
            rospy.logdebug("Angular VEL=" + str(odom_angular_vel) + ", ?RANGE=[" + str(angular_speed_minus) + "," + str(angular_speed_plus) + "]")

            linear_vel_are_close = (odom_linear_vel <= linear_speed_plus) and (odom_linear_vel > linear_speed_minus)
            angular_vel_are_close = (odom_angular_vel <= angular_speed_plus) and (odom_angular_vel > angular_speed_minus)

            if linear_vel_are_close and angular_vel_are_close:
                rospy.logwarn("Reached Velocity!")
                end_wait_time = rospy.get_rostime().to_sec()
                break

            if crashed_into_something:
                rospy.logerr("TurtleBot has crashed, stopping movement!")
                break

            rospy.logwarn("Not there yet, keep waiting...")
            rate.sleep()

        delta_time = end_wait_time - start_wait_time
        rospy.logdebug("[Wait Time=" + str(delta_time) + "]")

        rospy.logwarn("END wait_until_twist_achieved...")

        return delta_time

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
            for i, item in enumerate(laser_data.ranges):
                if item == float ('Inf') or numpy.isinf(item):
                    pass
                elif numpy.isnan(item):
                   pass
                else:
                    # Has a Non Infinite or Nan Value
                    if (item < min_laser_distance):
                        rospy.logerr("TurtleBot HAS CRASHED >>> item=" + str(item) + "< " + str(min_laser_distance))
                        robot_has_crashed = True
                        break

        return robot_has_crashed

    def get_model_state(self):
        return self.model_state

    def get_odom(self):
        return self.odom

    def get_camera_depth_image_raw(self):
        return self.camera_depth_image_raw

    def get_camera_depth_points(self):
        return self.camera_depth_points

    def get_camera_rgb_image_raw(self):
        return self.camera_rgb_image_raw

    def get_laser_scan(self):
        return self.laser_scan

    def get_ir_scan(self):
        return self.ir_scan

    def reinit_sensors(self):
        """
        This method is for the tasks so that when reseting the episode
        the sensors values are forced to be updated with the real data and

        """
