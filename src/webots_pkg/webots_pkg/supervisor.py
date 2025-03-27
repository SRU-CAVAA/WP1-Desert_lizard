import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from rosgraph_msgs.msg import Clock
from webots_msgs_pkg.msg import ExpStatus
from webots_ros2_msgs.srv import SpawnNodeFromString

from pathlib import Path
ws_path = str(Path(__file__).parents[6])
print(ws_path)

import configparser
parameters = configparser.ConfigParser()
parameters.read(ws_path + '/src/config.ini')


#########################
# Supervisor functionalities
# - Spawn robot and stimuli models
# - Monitorize simulation time and finish the experiment
# - Set experiment status to 1 = Simulation ready to create the gradients
# - Implements batches of experiments



class Supervisor(Node):

    def __init__(self):
        super().__init__('supervisor')
        self.get_logger().info('------------- ----------------- Supervisor -------------- -------------')


        #----------------- ROS TOPIC SUBSCRIPTIONS AND PUBLICATIONS -----------------
        self.create_subscription(Clock, '/clock', self.clock_callback, 1)
        self.create_subscription(ExpStatus, '/Experiment/status', self.experiment_status_callback, 1)

        self.pub_remove_Node = self.create_publisher(String, '/Ros2Supervisor/remove_node', 1)
        self.pub_exp_status = self.create_publisher(ExpStatus, '/Experiment/status', 1)
        self.pub_sim_time = self.create_publisher(Float32, '/Ros2Supervisor/sim_time', 1)


        #----------------- ROS SERVICES -----------------
        self.cli = self.create_client(SpawnNodeFromString, '/Ros2Supervisor/spawn_node_from_string')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.SpawnNode_req = SpawnNodeFromString.Request()


        #----------------- SUPERVISOR HYPERPAPRAMETERS -----------------
        self.trial_time_limit = int(parameters.get('General', 'trial_duration'))
        self.exp_batch_size = int(parameters.get('General', 'exp_batch_size'))

        self.agent_translation = parameters.get('Supervisor', 'agent_translation')
        self.agent_rotation = parameters.get('Supervisor', 'agent_rotation')
        self.mate_translation = parameters.get('Supervisor', 'mate_translation')
        self.mate_rotation = parameters.get('Supervisor', 'mate_rotation')
        self.predator_translation = parameters.get('Supervisor', 'predator_translation')
        self.predator_rotation = parameters.get('Supervisor', 'predator_rotation')

        self.food_translation = parameters.get('Supervisor', 'food_translation')
        self.water_translation = parameters.get('Supervisor', 'water_translation')

        self.sim_time = 0
        self.experiment_status = 0
        self.trial = 1


#----------------- ROS CALLBACK FUNCTIONS -----------------

    def clock_callback(self, message):
        self.clock = message.clock
        self.secs = self.clock.sec
        self.nanosecs = self.clock.nanosec/1000000000
        self.time = self.secs + self.nanosecs
        self.experiment_time(self.time)

        if self.experiment_status == 0:
            self.spawn_Agents()
            if self.trial == 1:
                self.spawn_Resources()
            time.sleep(5)            

            self.update_experiment_status(1) # When models have been spawned, Experiment Status = 1
            print("Starting Trial =", str(self.trial))

        self.end_trial()
        # Once data is gathered, experiment status will be set up to 0 and the supervisor will start a new trial.



    def experiment_status_callback(self, message):
        self.experiment_status = message.status
        print('experiment_status =', self.experiment_status)


    def experiment_time(self, time):
        if self.experiment_status == 4:
            self.start_time = self.secs + self.nanosecs

        if self.experiment_status == 5:
            self.sim_time = self.secs + self.nanosecs - self.start_time #Current simulation time in secs. Every trial it starts at 0.

            msg = Float32()
            msg.data = self.sim_time
            self.pub_sim_time.publish(msg)

            print("Simulation time:", self.sim_time)

    
    def update_experiment_status(self, exp_status):
        self.experiment_status = exp_status
        msg = ExpStatus()
        msg.status = self.experiment_status
        self.pub_exp_status.publish(msg)
        print('Setting experiment status to', exp_status)


    def send_SpawnNode_request(self, request):
        print('Request:', request)
        self.SpawnNode_req.data =  request
        self.future = self.cli.call_async(self.SpawnNode_req)
        time.sleep(2)
        

    def spawn_Agents(self):
        request1 = "E-puck { name \"epuck_agent\", translation " + self.agent_translation + ", rotation " + self.agent_rotation + ", controller \"<extern>\", distance_sensor_numberOfRays 3, turretSlot [Compass {} GPS {}]}"
        request2 = "E-puck_peer { name \"epuck_peer\", translation " + self.mate_translation + ", rotation " + self.mate_rotation + ", controller \"<extern>\", distance_sensor_numberOfRays 3, turretSlot [Compass {} GPS {}]}"
        request3 = "E-puck_predator { name \"epuck_predator\", translation " + self.predator_translation + ", rotation " + self.predator_rotation + ", controller \"<extern>\", distance_sensor_numberOfRays 3, turretSlot [Compass {} GPS {}]}"
        request = request1 + request2 + request3
        
        self.send_SpawnNode_request(request1)
        self.send_SpawnNode_request(request2)
        self.send_SpawnNode_request(request3)




    def spawn_Resources(self):
        request1 = "Apple { name \"apple\", translation " + self.food_translation + "}"
        request2 = ", WaterBottle { name \"waterbottle\", translation " + self.water_translation + "}"
        request = request1 + request2
        self.send_SpawnNode_request(request)

    def remove_Node(self, node):
        msg = String()
        msg.data = node
        self.pub_remove_Node.publish(msg)
        time.sleep(1)

    def end_trial(self):
        if self.sim_time > self.trial_time_limit:
            self.trial += 1

            print("---------- END OF THE TRIAL ----------")
            self.update_experiment_status(6)
            self.remove_Node("epuck_agent")
            self.remove_Node("epuck_peer")
            self.remove_Node("epuck_predator")
            self.sim_time = 0

            if self.trial > self.exp_batch_size:
                print("---------- END OF THE EXPERIMENT ----------")
                self.update_experiment_status(7)
                




    '''def remove_Node(self, node):
        msg = String()
        msg.data = node
        self.pub_remove_Node.publish(msg)


    def end_trial(self):
        self.trial_number += 1
        time.sleep(0.2)

        if self.trial_number >= self.total_trials or (self.img_saving == True and self.frame_ID >= self.frame_limit):
            self.update_experiment_status(4)
        else:
            self.update_experiment_status(0)
            
        self.remove_Node('epuck_agent')

        self.agent_x_position = 9 #Arbitrary value outbound of arena
        self.agent_y_position = 9
        self.agent_orientation = 0

        self.spawn_Agent()
        self.check_rewards_spawned(self.last_reward)

        self.time_list = []


    def reset_trial(self):
        self.get_logger().info("Reseting trial: " + str(self.trial_number) + "/" + str(self.total_trials))

        self.update_experiment_status(2) # 0 for not LTM uptade in fail trails
        self.remove_Node('epuck_agent')
        self.spawn_Agent()



    def unblocking_reset(self):
        self.list_X.append(self.agent_x_position)
        self.list_Y.append(self.agent_y_position)

        if self.experiment_status == 0:
            self.list_X = []
            self.list_Y = []

        if len(self.list_X) > self.block_lenght:
            self.list_X.pop(0)
            self.list_Y.pop(0)

            X_diff = max(self.list_X) - min(self.list_X)
            Y_diff = max(self.list_Y) - min(self.list_Y)

            if X_diff < 0.02 and Y_diff < 0.02 and self.experiment_status == 1:
                print("UNBLOCKING RESET")
                self.reset_trial()



    def Time_reset(self):
        if self.trial_time_limit!= 0 and self.sim_time > self.trial_time_limit:
            print("TIME RESET")
            self.reset_trial()'''





def main(args=None):
    rclpy.init(args=args)
    supervisor = Supervisor()
    rclpy.spin(supervisor)
    supervisor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()