import rclpy
import random
from sensor_msgs.msg import Range
from webots_msgs_pkg.msg import ExpStatus


class MyEpuckPeer:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__left_motor = self.__robot.getDevice('left wheel motor')
        self.__right_motor = self.__robot.getDevice('right wheel motor')

        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)

        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)

        rclpy.init(args=None)
        self.__node = rclpy.create_node('epuck_peer')

        self.__node.get_logger().info('------------- ----------------- epuck_peer -------------- -------------')

        self.obstacle = False
        self.action_interval_counter = 0
        self.wheel_speed_r, self.wheel_speed_l = 3.0, 3.0
        self.ps0, self.ps1, self.ps2, self.ps3, self.ps4, self.ps5, self.ps6, self.ps7 = 0,0,0,0,0,0,0,0

        self.experiment_status = 0


        self.__node.create_subscription(Range, 'epuck_peer/ps0', self.__ps0_sensor_callback, 1)
        self.__node.create_subscription(Range, 'epuck_peer/ps1', self.__ps1_sensor_callback, 1)
        self.__node.create_subscription(Range, 'epuck_peer/ps2', self.__ps2_sensor_callback, 1)
        self.__node.create_subscription(Range, 'epuck_peer/ps3', self.__ps3_sensor_callback, 1)
        self.__node.create_subscription(Range, 'epuck_peer/ps4', self.__ps4_sensor_callback, 1)
        self.__node.create_subscription(Range, 'epuck_peer/ps5', self.__ps5_sensor_callback, 1)
        self.__node.create_subscription(Range, 'epuck_peer/ps6', self.__ps6_sensor_callback, 1)
        self.__node.create_subscription(Range, 'epuck_peer/ps7', self.__ps7_sensor_callback, 1)
        self.__node.create_subscription(ExpStatus, '/Experiment/status', self.experiment_status_callback, 1)
        

    def __ps0_sensor_callback(self, message):
        self.ps0 = message.range

    def __ps1_sensor_callback(self, message):
        self.ps1 = message.range

    def __ps2_sensor_callback(self, message):
        self.ps2 = message.range

    def __ps3_sensor_callback(self, message):
        self.ps3 = message.range

    def __ps4_sensor_callback(self, message):
        self.ps4 = message.range

    def __ps5_sensor_callback(self, message):
        self.ps5 = message.range

    def __ps6_sensor_callback(self, message):
        self.ps6 = message.range

    def __ps7_sensor_callback(self, message):
        self.ps7 = message.range

    def experiment_status_callback(self, message):
        self.experiment_status = message.status

        

    def obstacle_avoidance(self):
        if self.obstacle == True:
            if self.obstacle_right == False:
                if self.wheel_speed_r < 4:
                    self.wheel_speed_r += 0.1
                if self.wheel_speed_l > 1:
                    self.wheel_speed_l -= 0.1

            else:
                if self.wheel_speed_l < 4:
                    self.wheel_speed_l += 0.1
                if self.wheel_speed_r > 1:
                    self.wheel_speed_r -= 0.1


    def random_action(self, action_interval):
        self.action_interval_counter += 1
        if self.action_interval_counter > action_interval:

            self.detect_obstacle()
            if self.obstacle == True:
                self.obstacle_avoidance()

            else:
                    
                self.action_interval_counter = 0
                self.wheel_speed_r = self.wheel_speed_r + random.gauss(0,1)
                self.wheel_speed_l = self.wheel_speed_l + random.gauss(0,1)

                if self.wheel_speed_r < 3.0: self.wheel_speed_r = 3.0
                if self.wheel_speed_r > 6.0: self.wheel_speed_r = 6.0
                if self.wheel_speed_l < 3.0: self.wheel_speed_l = 3.0
                if self.wheel_speed_l > 6.0: self.wheel_speed_l = 6.0

        return(self.wheel_speed_r, self.wheel_speed_l)



    def detect_obstacle(self):
        self.prox_sensors = [self.ps0, self.ps1, self.ps2, self.ps3, self.ps4, self.ps5, self.ps6, self.ps7]

        self.__right_sensor_value = (self.ps0 + self.ps1 + self.ps2) / 3
        self.__left_sensor_value = (self.ps5 + self.ps6 + self.ps7) / 3

        if any(sensor < 0.29  for sensor in self.prox_sensors):
            self.obstacle = True
            if self.__right_sensor_value > self.__left_sensor_value:
                self.obstacle_right = True

            else:
                self.obstacle_right = False

        else:
            self.obstacle = False



    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        if self.experiment_status == 5:
            if self.obstacle == True:
                action_interval = 1
            else:
                action_interval = 10

            command_motor_right, command_motor_left = self.random_action(action_interval = action_interval)
            self.__left_motor.setVelocity(command_motor_left)
            self.__right_motor.setVelocity(command_motor_right)