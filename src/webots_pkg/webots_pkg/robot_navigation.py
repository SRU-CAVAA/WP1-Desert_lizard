import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Range
from rosgraph_msgs.msg import Clock
from webots_msgs_pkg.msg import EpuckMotors, LocalViews, AttOutput, ExpStatus
from webots_ros2_msgs.msg import FloatStamped
import random

from pathlib import Path
ws_path = str(Path(__file__).parents[6])
print(ws_path)

import configparser
parameters = configparser.ConfigParser()
parameters.read(ws_path + '/src/config.ini')

#########################
# Robot Navigation node. This nodes send the motors commands to the allostatic agent based on:
# - Allostatic model outputs.
# - An obstacle-avoidance mechanisms.


class Robot_Navigation(Node):
    def __init__(self):
        super().__init__('robot_navigation')

        self.get_logger().info('------------- ----------------- Robot_Navigation -------------- -------------')

        self.action_rate = float(parameters.get('Navigation', 'action_rate'))
        self.avoidance_speed = float(parameters.get('Navigation', 'avoidance_speed'))
        self.dV = float(parameters.get('Allostatic_Model', 'setpoint'))
        self.allostatic_agent = int(parameters.get('General', 'allostatic_agent'))

        #----------------- ROS TOPIC SUBSCRIPTIONS AND PUBLICATIONS -----------------
        self.Motor__publisher = self.create_publisher(EpuckMotors, 'allostatic_controller/epuck_motors', 1)
        self.pub_exp_status = self.create_publisher(ExpStatus, '/Experiment/status', 1)

        self.create_subscription(Clock, '/clock', self.clock_callback, 1)
        self.create_subscription(ExpStatus, '/Experiment/status', self.experiment_status_callback, 1)
        self.create_subscription(Range, 'epuck_agent/ps0', self.__ps0_sensor_callback, 1)
        self.create_subscription(Range, 'epuck_agent/ps1', self.__ps1_sensor_callback, 1)
        self.create_subscription(Range, 'epuck_agent/ps6', self.__ps6_sensor_callback, 1)
        self.create_subscription(Range, 'epuck_agent/ps7', self.__ps7_sensor_callback, 1)
        self.create_subscription(FloatStamped, '/epuck_agent/compass/bearing', self.agent_Compass_callback, 1)
        self.create_subscription(LocalViews, '/Gradients/Local_views', self.LV__callback, 1)
        if self.allostatic_agent:
            self.create_subscription(AttOutput, '/allostatic_controller/Attractor_outputs', self.attractor_outputs_callback, 1)

        

        self.experiment_status = 0

        self.agent_orientation = 0

        self.prev_action_time = 0

        self.aV_temperature = 0.0
        self.aV_thirst = 0.0
        self.aV_food = 0.0
        self.aV_predator = 0.0
        self.aV_mate = 0.0

        self.total_force_temperature = 0
        self.total_force_thirst = 0
        self.total_force_food = 0
        self.total_force_predator = 0
        self.total_force_mate = 0

        self.ps0, self.ps1, self.ps6, self.ps7 = 0,0,0,0
        self.hsign_temperature = 0
        self.hsign_thirst = 0
        self.hsign_food = 0
        self.hsign_predator = 0
        self.hsign_mate = 0
        self.adsign_temperature = 0
        self.adsign_thirst = 0
        self.adsign_food = 0
        self.adsign_predator = 0
        self.adsign_mate = 0

        self.wheel_r = 0.1
        self.wheel_l = 0.1

        self.q0_temperature = 0
        self.q1_temperature = 0
        self.q2_temperature = 0
        self.q3_temperature = 0

        self.q0_thirst = 0
        self.q1_thirst = 0
        self.q2_thirst = 0
        self.q3_thirst = 0

        self.q0_food = 0
        self.q1_food = 0
        self.q2_food = 0
        self.q3_food = 0

        self.q0_mate = 0
        self.q1_mate = 0
        self.q2_mate = 0
        self.q3_mate = 0

        self.q0_predator = 0
        self.q1_predator = 0
        self.q2_predator = 0
        self.q3_predator = 0

        self.time = 0

        self.wheel_speed_r = 0
        self.wheel_speed_l = 0




    def experiment_status_callback(self, message):
        self.experiment_status = message.status
        print('experiment_status =', self.experiment_status)


    def clock_callback(self, message): #----------   ----------   CLOCK   ----------   ----------
        self.clock = message.clock
        self.secs = self.clock.sec
        self.nanosecs = self.clock.nanosec/1000000000
        self.time = self.secs + self.nanosecs

        if self.experiment_status == 0:
            self.clean_variables()

        if self.experiment_status == 3:
            self.update_experiment_status(4)

        if self.experiment_status == 5:
            if self.allostatic_agent:
                self.adsign()
                self.hsign()
            self.wheel_turning()


    def clean_variables(self):
        self.agent_orientation = 0

        self.prev_action_time = 0
        self.time = 0

        self.ps0, self.ps1, self.ps6, self.ps7 = 0,0,0,0
        self.wheel_r = 0.1
        self.wheel_l = 0.1
        

        self.wheel_speed_r = 0.0
        self.wheel_speed_l = 0.0


    def __ps0_sensor_callback(self, message):
        self.ps0 = message.range

    def __ps1_sensor_callback(self, message):
        self.ps1 = message.range

    def __ps6_sensor_callback(self, message):
        self.ps6 = message.range

    def __ps7_sensor_callback(self, message):
        command_message = EpuckMotors()

        self.ps7 = message.range

        self.prox_sensors = [self.ps0, self.ps1, self.ps6, self.ps7]

        self.__right_sensor_value = (self.ps0 + self.ps1) / 2
        self.__left_sensor_value = (self.ps6 + self.ps7) / 2

        command_message.right_motor, command_message.left_motor = self.navigation(time = self.time)

        self.Motor__publisher.publish(command_message)

    def agent_Compass_callback(self, message):
        self.agent_orientation = message.data


    def LV__callback(self, message):

        self.q0_temperature = message.q0_temperature
        self.q1_temperature = message.q1_temperature
        self.q2_temperature = message.q2_temperature
        self.q3_temperature = message.q3_temperature
        self.aV_temperature = (self.q0_temperature + self.q1_temperature + self.q2_temperature + self.q3_temperature) / 4
        self.diff_temperature = abs(self.dV - self.aV_temperature)

        self.q0_thirst = message.q0_thirst
        self.q1_thirst = message.q1_thirst
        self.q2_thirst = message.q2_thirst
        self.q3_thirst = message.q3_thirst
        self.aV_thirst = (self.q0_thirst + self.q1_thirst + self.q2_thirst + self.q3_thirst) / 4
        self.diff_thirst = abs(self.dV - self.aV_thirst)

        self.q0_food = message.q0_food
        self.q1_food = message.q1_food
        self.q2_food = message.q2_food
        self.q3_food = message.q3_food
        self.aV_food = (self.q0_food + self.q1_food + self.q2_food + self.q3_food) / 4
        self.diff_food = abs(self.dV - self.aV_food)

        self.q0_predator = message.q0_predator
        self.q1_predator = message.q1_predator
        self.q2_predator = message.q2_predator
        self.q3_predator = message.q3_predator
        self.aV_predator = (self.q0_predator + self.q1_predator + self.q2_predator + self.q3_predator) / 4
        self.diff_predator = abs(0 - self.aV_predator)

        self.q0_mate = message.q0_mate
        self.q1_mate = message.q1_mate
        self.q2_mate = message.q2_mate
        self.q3_mate = message.q3_mate
        self.aV_mate = (self.q0_mate + self.q1_mate + self.q2_mate + self.q3_mate) / 4
        self.diff_mate =abs (self.dV - self.aV_mate)


    def attractor_outputs_callback(self, message):
        self.total_force_temperature = message.att_temperature
        self.total_force_thirst = message.att_thirst
        self.total_force_food = message.att_food
        self.total_force_predator = message.att_predator
        self.total_force_mate = message.att_peer


    def update_experiment_status(self, exp_status):
        self.experiment_status = exp_status
        msg = ExpStatus()
        msg.status = self.experiment_status
        self.pub_exp_status.publish(msg)
        print('Setting experiment status to', exp_status)


    def detect_obstacle(self):
        if any(sensor < 0.29  for sensor in self.prox_sensors):
            self.obstacle = True
            if self.__right_sensor_value > self.__left_sensor_value:
                self.obstacle_right = True

            else:
                self.obstacle_right = False

        else:
            self.obstacle = False


    def obstacle_avoidance(self):
        if self.obstacle == True:
            if self.obstacle_right == False:
                if self.wheel_speed_l > -6:
                    self.wheel_speed_l -= self.avoidance_speed
                else:
                    self.wheel_speed_r -= self.avoidance_speed


            else:
                if self.wheel_speed_r > -6:
                    self.wheel_speed_r -= self.avoidance_speed
                else:
                    self.wheel_speed_l -= self.avoidance_speed

        else:
            self.wheel_speed_r, self.wheel_speed_l = 3.0, 3.0

        print('Avoiding obstacle with motor cmd =', [self.wheel_speed_r, self.wheel_speed_l])


    def random_navigation(self):
        self.wheel_speed_r += random.gauss(0,1)
        self.wheel_speed_l += random.gauss(0,1)

    def navigation(self, time):
        
        if time >= self.prev_action_time + self.action_rate:
            self.prev_action_time = time

            self.detect_obstacle()
            if self.obstacle == True:
                self.obstacle_avoidance()

            else:
                self.wheel_speed_r = self.wheel_r
                self.wheel_speed_l = self.wheel_l

        if not self.allostatic_agent:
            self.random_navigation()

        if self.wheel_speed_r > 6.20: self.wheel_speed_r = 6.20
        if self.wheel_speed_l > 6.20: self.wheel_speed_l = 6.20
        if self.wheel_speed_r < -6.20: self.wheel_speed_r = -6.20
        if self.wheel_speed_l < -6.20: self.wheel_speed_l = -6.20

        return(self.wheel_speed_r, self.wheel_speed_l)



    ########################### ORIENTATION ###########################

    def adsign(self):
        self.adsign_temperature = np.sign(self.dV - self.aV_temperature)
        self.adsign_thirst = np.sign(self.dV - self.aV_thirst)
        self.adsign_food = np.sign(self.dV - self.aV_food)
        self.adsign_predator = np.sign(0 - self.aV_predator)
        self.adsign_mate = np.sign(self.dV - self.aV_mate)


    def hsign(self):
        theta = self.agent_orientation
        if theta <= 292 and theta > 247: #UP
            self.hsign_temperature = np.sign(self.q0_temperature - self.q1_temperature)
            self.hsign_thirst = np.sign(self.q0_thirst - self.q1_thirst)
            self.hsign_food = np.sign(self.q0_food - self.q1_food)
            self.hsign_predator = np.sign(self.q0_predator - self.q1_predator)
            self.hsign_mate = np.sign(self.q0_mate - self.q1_mate)

        elif theta <= 247 and theta > 202: #UP-L
            self.hsign_temperature = np.sign(((self.q0_temperature + self.q2_temperature)/2) - ((self.q0_temperature + self.q1_temperature)/2)) 
            self.hsign_thirst = np.sign(((self.q0_thirst + self.q2_thirst)/2) - ((self.q0_thirst + self.q1_thirst)/2)) 
            self.hsign_food = np.sign(((self.q0_food + self.q2_food)/2) - ((self.q0_food + self.q1_food)/2)) 
            self.hsign_predator = np.sign(((self.q0_predator + self.q2_predator)/2) - ((self.q0_predator + self.q1_predator)/2)) 
            self.hsign_mate = np.sign(((self.q0_mate + self.q2_mate)/2) - ((self.q0_mate + self.q1_mate)/2)) 

        elif theta <= 202 and theta > 157: #L
            self.hsign_temperature = np.sign(self.q2_temperature - self.q0_temperature)
            self.hsign_thirst = np.sign(self.q2_thirst - self.q0_thirst)
            self.hsign_food = np.sign(self.q2_food - self.q0_food)
            self.hsign_predator = np.sign(self.q2_predator - self.q0_predator)
            self.hsign_mate = np.sign(self.q2_mate - self.q0_mate)

        elif theta <= 157 and theta > 112: #DOWN-L
            self.hsign_temperature = np.sign(((self.q2_temperature + self.q3_temperature)/2) - ((self.q2_temperature + self.q0_temperature)/2))
            self.hsign_thirst = np.sign(((self.q2_thirst + self.q3_thirst)/2) - ((self.q2_thirst + self.q0_thirst)/2))
            self.hsign_food = np.sign(((self.q2_food + self.q3_food)/2) - ((self.q2_food + self.q0_food)/2))
            self.hsign_predator = np.sign(((self.q2_predator + self.q3_predator)/2) - ((self.q2_predator + self.q0_predator)/2))
            self.hsign_mate = np.sign(((self.q2_mate + self.q3_mate)/2) - ((self.q2_mate + self.q0_mate)/2))

        elif theta <= 112 and theta > 77: #DOWN
            self.hsign_temperature = np.sign(self.q3_temperature - self.q2_temperature)
            self.hsign_thirst = np.sign(self.q3_thirst - self.q2_thirst)
            self.hsign_food = np.sign(self.q3_food - self.q2_food)
            self.hsign_predator = np.sign(self.q3_predator - self.q2_predator)
            self.hsign_mate = np.sign(self.q3_mate - self.q2_mate)

        elif theta <= 77 and theta > 22: #DOWN-R
            self.hsign_temperature = np.sign(((self.q3_temperature + self.q1_temperature)/2) - ((self.q3_temperature + self.q2_temperature)/2))
            self.hsign_thirst = np.sign(((self.q3_thirst + self.q1_thirst)/2) - ((self.q3_thirst + self.q2_thirst)/2))
            self.hsign_food = np.sign(((self.q3_food + self.q1_food)/2) - ((self.q3_food + self.q2_food)/2))
            self.hsign_predator = np.sign(((self.q3_predator + self.q1_predator)/2) - ((self.q3_predator + self.q2_predator)/2))
            self.hsign_mate = np.sign(((self.q3_mate + self.q1_mate)/2) - ((self.q3_mate + self.q2_mate)/2))

        elif theta <= 22 and theta > 337: #R
            self.hsign_temperature = np.sign(self.q1_temperature - self.q3_temperature)
            self.hsign_thirst = np.sign(self.q1_thirst - self.q3_thirst)
            self.hsign_food = np.sign(self.q1_food - self.q3_food)
            self.hsign_predator = np.sign(self.q1_predator - self.q3_predator)
            self.hsign_mate = np.sign(self.q1_mate - self.q3_mate)

        elif theta <= 337 and theta > 292: #UP-R
            self.hsign_temperature = np.sign(((self.q1_temperature + self.q0_temperature)/2) - ((self.q1_temperature + self.q3_temperature)/2))
            self.hsign_thirst = np.sign(((self.q1_thirst + self.q0_thirst)/2) - ((self.q1_thirst + self.q3_thirst)/2))
            self.hsign_food = np.sign(((self.q1_food + self.q0_food)/2) - ((self.q1_food + self.q3_food)/2))
            self.hsign_predator = np.sign(((self.q1_predator + self.q0_predator)/2) - ((self.q1_predator + self.q3_predator)/2))
            self.hsign_mate = np.sign(((self.q1_mate + self.q0_mate)/2) - ((self.q1_mate + self.q3_mate)/2))


    def wheel_turning(self):
        self.wheel_r = 1 + ((self.hsign_temperature * self.adsign_temperature* self.total_force_temperature) + (self.hsign_thirst * self.adsign_thirst* self.total_force_thirst) + (self.hsign_food * self.adsign_food* self.total_force_food) + (self.hsign_predator * self.adsign_predator* self.total_force_predator) + (self.hsign_mate * self.adsign_mate* self.total_force_mate)) * (1/5)
        self.wheel_l = 1 - ((self.hsign_temperature * self.adsign_temperature* self.total_force_temperature) + (self.hsign_thirst * self.adsign_thirst* self.total_force_thirst) + (self.hsign_food * self.adsign_food* self.total_force_food) + (self.hsign_predator * self.adsign_predator* self.total_force_predator) + (self.hsign_mate * self.adsign_mate* self.total_force_mate)) * (1/5)
        self.wheel_r *= 5 + np.random.uniform(-0.5,0.5)
        self.wheel_l *= 5 + np.random.uniform(-0.5,0.5)

        if self.wheel_r > 6.2: self.wheel_r = 6.2
        if self.wheel_l > 6.2: self.wheel_l = 6.2


def main(args=None):
    rclpy.init(args=args)
    robot_navigation = Robot_Navigation()
    rclpy.spin(robot_navigation)
    robot_navigation.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()