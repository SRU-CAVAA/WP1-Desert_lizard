import rclpy
#from geometry_msgs.msg import Twist
from webots_msgs_pkg.msg import EpuckMotors


class MyEpuckAgent:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__left_motor = self.__robot.getDevice('left wheel motor')
        self.__right_motor = self.__robot.getDevice('right wheel motor')

        #self.epuck_leds = [self.__robot.getDevice('left wheel motor')]

        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)

        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)

        self.__epuck_motors = EpuckMotors()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('epuck_agent')
        self.__node.create_subscription(EpuckMotors, 'allostatic_controller/epuck_motors', self.__cmd_vel_callback, 1)

        self.__node.get_logger().info('------------- ----------------- epuck_agent -------------- -------------')

    def __cmd_vel_callback(self, data):
        self.__epuck_motors.right_motor = data.right_motor
        self.__epuck_motors.left_motor = data.left_motor


    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        command_motor_right = self.__epuck_motors.right_motor
        command_motor_left = self.__epuck_motors.left_motor

        self.__left_motor.setVelocity(command_motor_left)
        self.__right_motor.setVelocity(command_motor_right)