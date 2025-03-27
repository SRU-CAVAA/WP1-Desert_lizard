import csv
import ast
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PointStamped
from webots_ros2_msgs.msg import FloatStamped
from webots_msgs_pkg.msg import ExpStatus, InternalState, AttOutput

from pathlib import Path
ws_path = str(Path(__file__).parents[6])
print(ws_path)

import configparser
parameters = configparser.ConfigParser()
parameters.read(ws_path + '/src/config.ini')

#########################
# Data gathering node functionalities
# data_classes = [Trial_number, Trial_time, Gradient_T, X, Y, Z, IS_temperature, IS_hydration, IS_energy, IS_security, IS_mating, Att_temperature, Att_hydration, Att_energy, Att_security, Att_mating]




class Data_gathering(Node):

    def __init__(self):
        super().__init__('data_gathering')
        self.get_logger().info('------------- ----------------- Data gathering -------------- -------------')


        #----------------- ROS TOPIC SUBSCRIPTIONS AND PUBLICATIONS -----------------
        self.create_subscription(Clock, '/clock', self.clock_callback, 1)
        self.create_subscription(Float32, '/Ros2Supervisor/sim_time', self.sim_time_callback, 1)
        self.create_subscription(ExpStatus, '/Experiment/status', self.experiment_status_callback, 1)

        self.create_subscription(Float32, '/Gradients/Mean_temperature', self.gradient_temp_callback, 1)

        self.create_subscription(PointStamped, '/epuck_agent/gps', self.agent_GPS_callback, 1)
        self.create_subscription(FloatStamped, '/epuck_agent/compass/bearing', self.agent_Compass_callback, 1)

        self.create_subscription(InternalState, '/allostatic_controller/InternalState', self.internal_states_callback, 1)
        self.create_subscription(AttOutput, '/allostatic_controller/Attractor_outputs', self.attractor_outputs_callback, 1)

        self.pub_exp_status = self.create_publisher(ExpStatus, '/Experiment/status', 1)


        self.data_classes = ast.literal_eval(parameters.get('Data_gathering', 'data_classes').replace('[', '["').replace(']', '"]').replace(', ', '", "'))
        self.allostatic_agent = int(parameters.get('General', 'allostatic_agent'))


        self.trial = 0
        self.trial_list = []

        self.mean_gradient_temperature = 0
        self.mean_gradient_temperature_list = []

        self.sim_time_list = []
        self.agent_X_list = []
        self.agent_Y_list = []
        self.agent_Z_list = []

        self.IS_temperature_list = []
        self.IS_hydration_list = []
        self.IS_energy_list = []
        self.IS_security_list = []
        self.IS_mating_list = []

        self.Att_temperature_list = []
        self.Att_hydration_list = []
        self.Att_energy_list = []
        self.Att_security_list = []
        self.Att_mating_list = []

        self.experiment_status = 0
        self.sim_time = 0
        self.agent_x_position = 0
        self.agent_y_position = 0
        self.internal_state_temperature = 0
        self.internal_state_thirst = 0
        self.internal_state_food = 0
        self.internal_state_security = 0
        self.internal_state_mate = 0
        self.total_force_temperature = 0
        self.total_force_thirst = 0
        self.total_force_food = 0
        self.total_force_predator = 0
        self.total_force_mate = 0



    def experiment_status_callback(self, message):
        self.experiment_status = message.status
        print('experiment_status =', self.experiment_status)


    def sim_time_callback(self, message):
        self.sim_time = message.data

    def gradient_temp_callback(self, message):
        self.mean_gradient_temperature = message.data


    def agent_GPS_callback(self, message):
        self.agent_x_position = message.point.x
        self.agent_y_position = message.point.y


    def agent_Compass_callback(self, message):
        self.agent_orientation = message.data


    def internal_states_callback(self, message):
        self.internal_state_temperature = message.is_temperature
        self.internal_state_thirst = message.is_thirst
        self.internal_state_food = message.is_food
        self.internal_state_security = message.is_security
        self.internal_state_mate = message.is_peer


    def attractor_outputs_callback(self, message):
        self.total_force_temperature = message.att_temperature
        self.total_force_thirst = message.att_thirst
        self.total_force_food = message.att_food
        self.total_force_predator = message.att_predator
        self.total_force_mate = message.att_peer

        if self.experiment_status == 5:
            self.store_data()


    def clock_callback(self, message): #----------   ----------   CLOCK   ----------   ----------
        self.clock = message.clock

        if self.experiment_status == 4:
            self.update_experiment_status(5)

        if self.experiment_status == 6:
            self.trial += 1
            self.save_data()
            self.clean_data()
            self.update_experiment_status(0)

        if self.experiment_status == 7:
            print('---------- END OF THE EXPERIMENT ----------')



    def store_data(self):
        self.trial_list.append(self.trial)
        self.sim_time_list.append(round(self.sim_time, 5))
        self.mean_gradient_temperature_list.append(round(self.mean_gradient_temperature, 5))
        self.agent_X_list.append(round(self.agent_x_position, 5))
        self.agent_Y_list.append(round(self.agent_y_position, 5))
        self.agent_Z_list.append(round(self.agent_orientation, 5))

        self.IS_temperature_list.append(round(self.internal_state_temperature, 5))
        self.IS_hydration_list.append(round(self.internal_state_thirst, 5))
        self.IS_energy_list.append(round(self.internal_state_food, 5))
        self.IS_security_list.append(round(self.internal_state_security, 5))
        self.IS_mating_list.append(round(self.internal_state_mate, 5))

        self.Att_temperature_list.append(round(self.total_force_temperature, 5))
        self.Att_hydration_list.append(round(self.total_force_thirst, 5))
        self.Att_energy_list.append(round(self.total_force_food, 5))
        self.Att_security_list.append(round(self.total_force_predator, 5))
        self.Att_mating_list.append(round(self.total_force_mate, 5))


    def clean_data(self):
        self.trial_list = []
        self.sim_time_list = []
        self.mean_gradient_temperature_list = []
        self.agent_X_list = []
        self.agent_Y_list = []
        self.agent_Z_list = []

        self.IS_temperature_list = []
        self.IS_hydration_list = []
        self.IS_energy_list = []
        self.IS_security_list = []
        self.IS_mating_list = []

        self.Att_temperature_list = []
        self.Att_hydration_list = []
        self.Att_energy_list = []
        self.Att_security_list = []
        self.Att_mating_list = []


    def save_data(self):
        if self.allostatic_agent: self.data_folder = ws_path + '/data/Allostatic_agent/'
        else: self.data_folder = ws_path + '/data/Control_agent/'
        csv_namefile = self.data_folder + str(self.trial) + '.csv'

        with open(csv_namefile, mode='a') as csv_file:
            print('data_classes =', self.data_classes)

            csv_writer = csv.DictWriter(csv_file, fieldnames=self.data_classes)
            csv_writer.writeheader()
            for i in range(len(self.trial_list)):
                csv_writer.writerow({self.data_classes[0]: self.trial_list[i], self.data_classes[1]: self.sim_time_list[i], self.data_classes[2]: self.mean_gradient_temperature_list[i], 
                    self.data_classes[3]: self.agent_X_list[i], self.data_classes[4]: self.agent_Y_list[i], self.data_classes[5]: self.agent_Z_list[i], self.data_classes[6]: self.IS_temperature_list[i], 
                    self.data_classes[7]: self.IS_hydration_list[i], self.data_classes[8]: self.IS_energy_list[i], self.data_classes[9]: self.IS_security_list[i], self.data_classes[10]: self.IS_mating_list[i], 
                    self.data_classes[11]: self.Att_temperature_list[i], self.data_classes[12]: self.Att_hydration_list[i], self.data_classes[13]: self.Att_energy_list[i], 
                    self.data_classes[14]: self.Att_security_list[i], self.data_classes[15]: self.Att_mating_list[i]})



    def update_experiment_status(self, exp_status):
        self.experiment_status = exp_status
        msg = ExpStatus()
        msg.status = self.experiment_status
        self.pub_exp_status.publish(msg)
        print('Setting experiment status to', exp_status)


def main(args=None):
    rclpy.init(args=args)
    data_gathering = Data_gathering()
    rclpy.spin(data_gathering)
    data_gathering.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()