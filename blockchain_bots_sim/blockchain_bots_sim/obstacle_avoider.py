import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist, PointStamped
import numpy as np
import math as m

MAX_RANGE = 0.15





class ObstacleAvoider(Node):
    def __init__(self, node_name='obstacle_avoider', robot_name='robot_1'):
        super().__init__(node_name)
        self._goals = {0 : np.array([0.5 , 0.5]),
                        1 : np.array([0.5 , -0.5]),
                        2 : np.array([-0.5 , -0.5]),
                        3 : np.array([-0.5 , 0.5])}
        self._robot_name = robot_name
        self._initialize_goals()
        self._kappa_r = 0.5
        self._kappa_theta = 0.2
        self.__publisher = self.create_publisher(Twist, robot_name+'/cmd_vel', 1)
        self.create_subscription(PointStamped, robot_name+'/gps', self.__gps_sensor_callback_robot, 1)

    def _initialize_goals(self):
        if self._robot_name == 'robot_1':
            self._currect_goal_idx = 3
        else:
            self._currect_goal_idx = 1


    def __gps_sensor_callback_robot(self, message):
        state_cart = np.array([message.point.x, message.point.y])
        goal_cart = self._goals[self._currect_goal_idx]
        self.get_logger().info("ROBOT_%s: at %f %f" % (self._robot_name, state_cart[0], state_cart[1]))

        k_r = self._kappa_r
        k_theta = self._kappa_theta

        # control law
        errors_cart = goal_cart - state_cart
        # u_cart = 0.1*errors
        # u_polar = self._cartesian_to_polar(u_cart)

        state_polar = self._cartesian_to_polar(state_cart)
        goal_polar = self._cartesian_to_polar(goal_cart)
        errors =  goal_polar - state_polar

        command_message = Twist()
        euclidean_dist = np.linalg.norm(errors,2)
        self.get_logger().info("EUCLIDEAN DIST FOR %s: %f" % (self._robot_name, euclidean_dist))

        if np.linalg.norm(errors_cart,2) > .05:
            command_message.linear.x = k_r*errors[0]
            command_message.angular.z = k_theta*errors[1]
            # self.get_logger().info("SENDING COMMAND FOR %s: x_vel:%f theta_vel:%f" % (self._robot_name, u_polar[0], u_polar[1]))

            # command_message.linear.x = u_polar[0]
            # command_message.angular.z = u_polar[1]
        else:
            command_message.linear.x = 0.0
            command_message.angular.z = 0.0
            # here sends command to sawtooth
            self._currect_goal_idx = (self._currect_goal_idx+1) % 4


        self.__publisher.publish(command_message)
        


    def _cartesian_to_polar(self,x,y):
        r = np.sqrt(x**2+y**2)
        theta = m.atan2(y,x)
        return r,theta
    
    def _cartesian_to_polar(self,x):
        r = np.sqrt(x[0]**2+x[1]**2)
        theta = m.atan2(x[1],x[0])
        return np.array([r,theta])



def main(args=None):
    rclpy.init(args=args)
    avoider1 = ObstacleAvoider(node_name='obstacle_avoider_1', robot_name='robot_1')
    avoider2 = ObstacleAvoider(node_name='obstacle_avoider_2', robot_name='robot_2')
    while(True):
        rclpy.spin_once(avoider1)
        rclpy.spin_once(avoider2)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    avoider.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()