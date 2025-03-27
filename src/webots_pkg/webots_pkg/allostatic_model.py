import rclpy
import random
import numpy as np
from rclpy.node import Node
import matplotlib.pyplot as plt
from rosgraph_msgs.msg import Clock
from webots_msgs_pkg.msg import ExpStatus, LocalViews, InternalState, AttOutput
from webots_pkg.multiattractor import Multiattractor

from pathlib import Path
ws_path = str(Path(__file__).parents[6])
print(ws_path)

import configparser
parameters = configparser.ConfigParser()
parameters.read(ws_path + '/src/config.ini')

#########################
# Allostatic model functionalities
# - This script imports the Multiattractor class
# - Homeostatic dynamics are generated to define the internal state of the agent.
# - 5 needs are contemplated: Hydration, Energy, Thermoregulation, Mating and Security.
# - The Multiattractor model implements a winner-take-all mechanism defining one motivational state for the agent.




class Allostatic_model(Node):

    def __init__(self):
        super().__init__('allostatic_model')
        self.get_logger().info('------------- ----------------- Allostatic model -------------- -------------')


        #----------------- ROS TOPIC SUBSCRIPTIONS AND PUBLICATIONS -----------------
        self.create_subscription(Clock, '/clock', self.clock_callback, 1)
        self.create_subscription(LocalViews, '/Gradients/Local_views', self.LV__callback, 1)
        self.create_subscription(ExpStatus, '/Experiment/status', self.experiment_status_callback, 1)


        self.Homeostasis__publisher = self.create_publisher(InternalState, 'allostatic_controller/InternalState', 1)
        self.Attractor__publisher = self.create_publisher(AttOutput, 'allostatic_controller/Attractor_outputs', 1)
        self.pub_exp_status = self.create_publisher(ExpStatus, '/Experiment/status', 1)

        #----------------- ALLOSTATIC MODEL HYPERPAPRAMETERS -----------------
        self.plotting = int(parameters.get('General', 'plotting'))
        self.aVhomeo_temperature = float(parameters.get('Allostatic_Model', 'init_temperature'))
        self.aVhomeo_thirst = float(parameters.get('Allostatic_Model', 'init_hydration'))
        self.aVhomeo_food = float(parameters.get('Allostatic_Model', 'init_energy'))
        self.aVhomeo_peer = float(parameters.get('Allostatic_Model', 'init_mating'))
        self.dV = float(parameters.get('Allostatic_Model', 'setpoint'))

        self.temperature_discount = float(parameters.get('Allostatic_Model', 'temperature_discount'))
        self.thirst_discount = float(parameters.get('Allostatic_Model', 'thirst_discount'))
        self.food_discount = float(parameters.get('Allostatic_Model', 'food_discount'))
        self.peer_discount = float(parameters.get('Allostatic_Model', 'peer_discount'))
        self.homeo_bonus = float(parameters.get('Allostatic_Model', 'homeo_bonus'))

        self.att_amplifier = int(parameters.get('Allostatic_Model', 'att_amplifier'))


        self.multiattractor = Multiattractor()
        self.att_normal_factor = 20

        self.experiment_status = 0

        self.total_LV_temperature = 0.0
        self.total_LV_thirst = 0.0
        self.total_LV_food = 0.0
        self.total_LV_predator = 0.0
        self.total_LV_peer = 0.0

        self.diff_temperature = 1.0
        self.diff_thirst = 1.0
        self.diff_food = 1.0
        self.diff_predator = 1.0
        self.diff_mate = 1.0

        self.aVtemperature_list = []
        self.aVthirst_list = []
        self.aVfood_list = []
        self.aVsecurity_list = []
        self.aVpeer_list = []

        self.TFtemperature_list = []
        self.TFthirst_list = []
        self.TFfood_list = []
        self.TFpredator_list = []
        self.TFpeer_list = []


        if self.plotting == True:
            plt.ion()
            plt.style.use('seaborn')
            self.fig2, self.ax2 = plt.subplots(1, 2,figsize=(20,4))
            self.fig2.tight_layout(pad=2.0)


    #----------------- ROS CALLBACK FUNCTIONS -----------------

    def experiment_status_callback(self, message):
        self.experiment_status = message.status


    def clock_callback(self, message): #----------   ----------   CLOCK   ----------   ----------
        self.clock = message.clock

        if self.experiment_status == 0:
            self.restart_variables()

        if self.experiment_status == 2: # Gradients are initiated, starts allostatic dynamics
            self.update_experiment_status(3)

        if self.experiment_status == 5:
            self.homeostasis()
            self.attractor_dynamics()

            if self.plotting == True:
                self.plot_internal_state()


    def restart_variables(self):
        self.aVhomeo_temperature = float(parameters.get('Allostatic_Model', 'init_temperature'))
        self.aVhomeo_thirst = float(parameters.get('Allostatic_Model', 'init_hydration'))
        self.aVhomeo_food = float(parameters.get('Allostatic_Model', 'init_energy'))
        self.aVhomeo_peer = float(parameters.get('Allostatic_Model', 'init_mating'))

        self.multiattractor.U = []
        self.att_normal_factor = 20

        self.total_LV_temperature = 0.0
        self.total_LV_thirst = 0.0
        self.total_LV_food = 0.0
        self.total_LV_predator = 0.0
        self.total_LV_peer = 0.0

        self.aVtemperature_list = []
        self.aVthirst_list = []
        self.aVfood_list = []
        self.aVsecurity_list = []
        self.aVpeer_list = []

        self.TFtemperature_list = []
        self.TFthirst_list = []
        self.TFfood_list = []
        self.TFpredator_list = []
        self.TFpeer_list = []


    def LV__callback(self, message):
        self.q0_temperature = message.q0_temperature
        self.q1_temperature = message.q1_temperature
        self.q2_temperature = message.q2_temperature
        self.q3_temperature = message.q3_temperature
        self.total_LV_temperature = (self.q0_temperature + self.q1_temperature + self.q2_temperature + self.q3_temperature) / 4
        self.diff_temperature = abs(self.dV - self.total_LV_temperature)

        self.q0_thirst = message.q0_thirst
        self.q1_thirst = message.q1_thirst
        self.q2_thirst = message.q2_thirst
        self.q3_thirst = message.q3_thirst
        self.total_LV_thirst = (self.q0_thirst + self.q1_thirst + self.q2_thirst + self.q3_thirst) / 4
        self.diff_thirst = abs(self.dV - self.total_LV_thirst)

        self.q0_food = message.q0_food
        self.q1_food = message.q1_food
        self.q2_food = message.q2_food
        self.q3_food = message.q3_food
        self.total_LV_food = (self.q0_food + self.q1_food + self.q2_food + self.q3_food) / 4
        self.diff_food = abs(self.dV - self.total_LV_food)

        self.q0_predator = message.q0_predator
        self.q1_predator = message.q1_predator
        self.q2_predator = message.q2_predator
        self.q3_predator = message.q3_predator
        self.total_LV_predator = (self.q0_predator + self.q1_predator + self.q2_predator + self.q3_predator) / 4
        self.diff_predator = abs(0 - self.total_LV_predator)

        self.q0_mate = message.q0_mate
        self.q1_mate = message.q1_mate
        self.q2_mate = message.q2_mate
        self.q3_mate = message.q3_mate
        self.total_LV_mate = (self.q0_mate + self.q1_mate + self.q2_mate + self.q3_mate) / 4
        self.diff_mate =abs(self.dV - self.total_LV_mate)


    def update_experiment_status(self, exp_status):
        self.experiment_status = exp_status
        msg = ExpStatus()
        msg.status = self.experiment_status
        self.pub_exp_status.publish(msg)
        print('Setting experiment status to', exp_status)


    def plot_internal_state(self):
        self.ax2[0].cla()
        self.ax2[0].grid(False)
        self.ax2[0].set_ylim(-0.1, 1.1)
        self.ax2[0].set_title("Internal state")
        if len(self.aVtemperature_list) > 100:
            self.ax2[0].plot(self.aVtemperature_list[-100:], color='orange', label='Temperature')
            self.ax2[0].plot(self.aVthirst_list[-100:], color='blue', label='Hydration')
            self.ax2[0].plot(self.aVfood_list[-100:], color='red', label='Energy')
            self.ax2[0].plot(self.aVsecurity_list[-100:], color='green', label='Security')
            self.ax2[0].plot(self.aVpeer_list[-100:], color='purple', label='Mating')
        else:
            self.ax2[0].plot(self.aVtemperature_list, color='orange', label='Temperature')
            self.ax2[0].plot(self.aVthirst_list, color='blue', label='Hydration')
            self.ax2[0].plot(self.aVfood_list, color='red', label='Energy')
            self.ax2[0].plot(self.aVsecurity_list, color='green', label='Security')
            self.ax2[0].plot(self.aVpeer_list, color='purple', label='Mating')

        self.ax2[1].cla()
        self.ax2[1].grid(False)
        self.ax2[1].set_title("Mean Firing Rate")
        self.ax2[1].set_ylim(-0.1, 1.2)
        if len(self.TFtemperature_list) > 100:
            self.ax2[1].plot(self.TFtemperature_list[-100:], color='orange', label='Temperature')
            self.ax2[1].plot(self.TFthirst_list[-100:], color='blue', label='Thirst')
            self.ax2[1].plot(self.TFfood_list[-100:], color='red', label='Energy')
            self.ax2[1].plot(self.TFpredator_list[-100:], color='green', label='Security')
            self.ax2[1].plot(self.TFpeer_list[-100:], color='purple', label='Mating')
        else:
            self.ax2[1].plot(self.TFtemperature_list, color='orange', label='Temperature')
            self.ax2[1].plot(self.TFthirst_list, color='blue', label='Thirst')
            self.ax2[1].plot(self.TFfood_list, color='red', label='Energy')
            self.ax2[1].plot(self.TFpredator_list, color='green', label='Security')
            self.ax2[1].plot(self.TFpeer_list, color='purple', label='Mating')
        self.ax2[1].legend(loc="upper left")

        self.fig2.canvas.flush_events()



    def homeostasis(self):
        self.aVhomeo_temperature -= self.temperature_discount
        self.aVhomeo_thirst -= self.thirst_discount
        self.aVhomeo_food -= self.food_discount
        self.aVhomeo_peer -= self.peer_discount
        self.aVhomeo_predator = self.total_LV_predator

        self.aVtemperature_list.append(self.aVhomeo_temperature)
        self.aVthirst_list.append(self.aVhomeo_thirst)
        self.aVfood_list.append(self.aVhomeo_food)
        self.aVsecurity_list.append(self.dV - self.aVhomeo_predator)
        self.aVpeer_list.append(self.aVhomeo_peer)

        if len(self.aVtemperature_list) > 100:
            self.aVtemperature_list.pop(0)
            self.aVthirst_list.pop(0)
            self.aVfood_list.pop(0)
            self.aVsecurity_list.pop(0)
            self.aVpeer_list.pop(0)


        if self.aVhomeo_temperature < 0: self.aVhomeo_temperature = 0.0
        if self.aVhomeo_thirst < 0: self.aVhomeo_thirst = 0.0
        if self.aVhomeo_food < 0: self.aVhomeo_food = 0.0
        if self.aVhomeo_peer < 0: self.aVhomeo_peer = 0.0

        if self.diff_temperature<0.8:
            self.aVhomeo_temperature += self.homeo_bonus

        if self.diff_thirst<0.02:
            self.aVhomeo_thirst += self.homeo_bonus

        if self.diff_food<0.02:
            self.aVhomeo_food += self.homeo_bonus

        if self.diff_mate<0.02:
            self.aVhomeo_peer += self.homeo_bonus

        if self.aVhomeo_temperature > self.dV: self.aVhomeo_temperature = 1.0
        if self.aVhomeo_thirst > self.dV: self.aVhomeo_thirst = 1.0
        if self.aVhomeo_food > self.dV: self.aVhomeo_food = 1.0
        if self.aVhomeo_peer > self.dV: self.aVhomeo_peer = 1.0

        #We use homeostatic errors as inputs for the attractor model (dV - aV). The Actual value is 0 when the system is satisfied
        self.Itemp_attractor = self.dV - self.aVhomeo_temperature
        self.Ithi_attractor = self.dV - self.aVhomeo_thirst
        self.Ifood_attractor = self.dV - self.aVhomeo_food
        self.Ipeer_attractor = self.dV - self.aVhomeo_peer
        self.Ipredator_attractor = self.aVhomeo_predator

        command_message = InternalState()
        command_message.is_temperature = self.aVhomeo_temperature
        command_message.is_thirst = self.aVhomeo_thirst
        command_message.is_food = self.aVhomeo_food
        command_message.is_security = self.dV - self.aVhomeo_predator
        command_message.is_peer = self.aVhomeo_peer

        self.Homeostasis__publisher.publish(command_message)


    def attractor_dynamics(self):
        self.Itemp_attractor *= self.att_amplifier
        self.Ithi_attractor *= self.att_amplifier
        self.Ifood_attractor *= self.att_amplifier
        self.Ipeer_attractor *= self.att_amplifier
        self.Ipredator_attractor *= self.att_amplifier * 2
        self.total_force_temperature, self.total_force_thirst, self.total_force_food, self.total_force_peer, self.total_force_predator  = self.multiattractor.advance(self.Itemp_attractor, self.Ithi_attractor, self.Ifood_attractor, self.Ipeer_attractor, self.Ipredator_attractor)

        att_outputs = [self.total_force_temperature, self.total_force_thirst, self.total_force_food, self.total_force_predator, self.total_force_peer]

        if any(output > self.att_normal_factor  for output in att_outputs):
            self.att_normal_factor = max(att_outputs)

        self.total_force_temperature /= self.att_normal_factor
        self.total_force_thirst /= self.att_normal_factor
        self.total_force_food /= self.att_normal_factor
        self.total_force_predator /= self.att_normal_factor
        self.total_force_peer /= self.att_normal_factor


        self.TFtemperature_list.append(self.total_force_temperature)
        self.TFthirst_list.append(self.total_force_thirst)
        self.TFfood_list.append(self.total_force_food)
        self.TFpredator_list.append(self.total_force_predator)
        self.TFpeer_list.append(self.total_force_peer)

        if len(self.TFtemperature_list) > 100:
            self.TFtemperature_list.pop(0)
            self.TFthirst_list.pop(0)
            self.TFfood_list.pop(0)
            self.TFpredator_list.pop(0)
            self.TFpeer_list.pop(0)


        command_message = AttOutput()
        command_message.att_temperature = self.total_force_temperature
        command_message.att_thirst = self.total_force_thirst
        command_message.att_food = self.total_force_food
        command_message.att_predator = self.total_force_predator
        command_message.att_peer = self.total_force_peer

        self.Attractor__publisher.publish(command_message)



def main(args=None):
    rclpy.init(args=args)
    allostatic_model = Allostatic_model()
    rclpy.spin(allostatic_model)
    allostatic_model.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()