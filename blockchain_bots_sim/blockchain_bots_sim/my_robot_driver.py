import rclpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose, Pose2D
from sensor_msgs.msg import Imu
import math as m


HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025

class MyRobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__left_motor = self.__robot.getDevice('left wheel motor')
        self.__right_motor = self.__robot.getDevice('right wheel motor')

        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)

        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)

        self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')
        self.__node.create_subscription(Twist, 'robot_1/cmd_vel', self.__cmd_vel_callback, 1)
        self.__node.create_subscription(Imu, 'robot_1/imu', self.__imu_callback, 1)
        self.__node.create_subscription(PointStamped, 'robot_1/gps', self.__gps_sensor_callback, 1)        
        self.__publisher = self.__node.create_publisher(Pose2D, 'robot_1/odom_2D', 1)
        

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z

        command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        self.__left_motor.setVelocity(command_motor_left)
        self.__right_motor.setVelocity(command_motor_right)
        self.calculate_odom_2D()

    def __imu_callback(self, imu_msg):
        self._q = imu_msg.orientation
        self._yaw = 2*m.acos(self._q.w)
        # self.__node.get_logger().info("IMU ROBOT_%s YAW: %f" % ("robot_1", self._yaw))

    def __gps_sensor_callback(self, gps_msg):
        self._gps_coords = gps_msg.point


    def calculate_odom_2D(self):
        odom_msg = Pose2D()
        # self.__ps_values[0] = self.__left_ps.getValue()
        # self.__ps_values[1] = self.__left_ps.getValue()
        odom_msg.x = self._gps_coords.x
        odom_msg.y = self._gps_coords.y
        odom_msg.theta = self._yaw

        self.__publisher.publish(odom_msg)



class MyRobotDriver2:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__left_motor = self.__robot.getDevice('left wheel motor')
        self.__right_motor = self.__robot.getDevice('right wheel motor')

        self.__left_ps = self.__robot.getDevice('ps_left')
        self.__right_ps = self.__robot.getDevice('ps_right')

        self.__left_ps.enable(64)
        self.__right_ps.enable(64)

        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)

        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)

        self.__ps_values = [0, 0]

        self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver2')
        self.__node.create_subscription(Twist, 'robot_2/cmd_vel', self.__cmd_vel_callback, 1)
        self.__node.create_subscription(Imu, 'robot_2/imu', self.__imu_callback, 1)
        self.__node.create_subscription(PointStamped, 'robot_2/gps', self.__gps_sensor_callback, 1)        
        self.__publisher = self.__node.create_publisher(Pose2D, 'robot_2/odom_2D', 1)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z

        command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        self.__left_motor.setVelocity(command_motor_left)
        self.__right_motor.setVelocity(command_motor_right)
        self.calculate_odom_2D()

    def __imu_callback(self, imu_msg):
        self._q = imu_msg.orientation
        self._yaw = 2*m.asin(self._q.z) 
        # self.__node.get_logger().info("IMU ROBOT_%s YAW: %f" % ("robot_2", self._yaw))

    def __gps_sensor_callback(self, gps_msg):
        self._gps_coords = gps_msg.point


    def calculate_odom_2D(self):
        odom_msg = Pose2D()
        # self.__ps_values[0] = self.__left_ps.getValue()
        # self.__ps_values[1] = self.__left_ps.getValue()
        odom_msg.x = self._gps_coords.x
        odom_msg.y = self._gps_coords.y
        odom_msg.theta = self._yaw

        self.__publisher.publish(odom_msg)