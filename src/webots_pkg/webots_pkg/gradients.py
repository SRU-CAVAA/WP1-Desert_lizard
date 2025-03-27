import math
import rclpy
import numpy as np
from rclpy.node import Node
import matplotlib.pyplot as plt
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PointStamped
from webots_ros2_msgs.msg import FloatStamped
from webots_msgs_pkg.msg import ExpStatus, LocalViews


from pathlib import Path
ws_path = str(Path(__file__).parents[6])
print(ws_path)

import configparser
parameters = configparser.ConfigParser()
parameters.read(ws_path + '/src/config.ini')


#########################
# Gradients functionalities
# - Create a gradient for each resource.
# - From the relative position of the robot to the resource a Local View is computed for each gradient.
# - Plotting of the gradients.



class Gradients(Node):

    def __init__(self):
        super().__init__('gradients')
        self.get_logger().info('------------- ----------------- Gradients -------------- -------------')


        #----------------- ROS TOPIC SUBSCRIPTIONS AND PUBLICATIONS -----------------
        self.create_subscription(Clock, '/clock', self.clock_callback, 1)
        self.create_subscription(Float32, '/Ros2Supervisor/sim_time', self.sim_time_callback, 1)
        self.create_subscription(ExpStatus, '/Experiment/status', self.experiment_status_callback, 1)

        self.create_subscription(PointStamped, '/epuck_agent/gps', self.agent_GPS_callback, 1)
        self.create_subscription(FloatStamped, '/epuck_agent/compass/bearing', self.agent_Compass_callback, 1)
        self.create_subscription(PointStamped, '/epuck_predator/gps', self.predator_GPS_callback, 1)
        self.create_subscription(FloatStamped, '/epuck_predator/compass/bearing', self.predator_Compass_callback, 1)
        self.create_subscription(PointStamped, '/epuck_peer/gps', self.mate_GPS_callback, 1)
        self.create_subscription(FloatStamped, '/epuck_peer/compass/bearing', self.mate_Compass_callback, 1)


        self.pub_mean_temperature = self.create_publisher(Float32, '/Gradients/Mean_temperature', 1)
        self.pub_exp_status = self.create_publisher(ExpStatus, '/Experiment/status', 1)
        self.LV__publisher = self.create_publisher(LocalViews, '/Gradients/Local_views', 1)


        #----------------- GRADIENT HYPERPAPRAMETERS -----------------
        self.plotting = int(parameters.get('General', 'plotting'))
        self.cycle_time = int(parameters.get('Gradients', 'cycle_duration'))
        self.day = True
        self.cycle_starting_time = 0

        self.arena_size = 350 + 10
        self.arenaX = [0,self.arena_size]
        self.arenaY = [0,self.arena_size]
        self.grid_size=1
        self.x_grid=np.arange(self.arenaX[0],self.arenaX[1],self.grid_size)
        self.y_grid=np.arange(self.arenaY[0],self.arenaY[1],self.grid_size)
        self.x_mesh, self.y_mesh = np.meshgrid(self.x_grid,self.y_grid)
        self.xc=self.x_mesh+(self.grid_size/2)
        self.yc=self.y_mesh+(self.grid_size/2)

        self.agent_x_gradient_position = 0
        self.agent_y_gradient_position = 0
        self.predator_x_gradient_position = 0
        self.predator_y_gradient_position = 0
        self.mate_x_gradient_position = 0
        self.mate_y_gradient_position = 0

        self.sim_time = 0
        self.grad_time_rate = 0
        self.experiment_status = 0
        self.gradients_initiated = False

        self.cycle_temperature = 0

        if self.plotting == True:
            plt.ion()
            plt.style.use('seaborn')
            self.fig_gradients, self.ax1 = plt.subplots(5, 1,figsize=(3,15))


        #----------------- ROS CALLBACK FUNCTIONS -----------------

    def experiment_status_callback(self, message):
        self.experiment_status = message.status
        print('experiment_status =', self.experiment_status)

        

    def sim_time_callback(self, message):
        self.sim_time = message.data

    def agent_GPS_callback(self, message):
        self.agent_X = message.point.x
        self.agent_Y = message.point.y
        self.agent_x_gradient_position = (self.arena_size/2) + (self.agent_X * 100)
        self.agent_y_gradient_position = (self.arena_size/2) + (self.agent_Y * 100)

    def agent_Compass_callback(self, message):
        self.agent_orientation = message.data

    def predator_GPS_callback(self, message):
        self.predator_X = message.point.x
        self.predator_Y = message.point.y
        self.predator_x_gradient_position = (self.arena_size/2) + (self.predator_X * 100)
        self.predator_y_gradient_position = (self.arena_size/2) + (self.predator_Y * 100)

    def predator_Compass_callback(self, message):
        self.predator_orientation = message.data

    def mate_GPS_callback(self, message):
        self.mate_X = message.point.x
        self.mate_Y = message.point.y
        self.mate_x_gradient_position = (self.arena_size/2) + (self.mate_X * 100)
        self.mate_y_gradient_position = (self.arena_size/2) + (self.mate_Y * 100)

    def mate_Compass_callback(self, message):
        self.mate_orientation = message.data


    def clock_callback(self, message): #----------   ----------   CLOCK   ----------   ----------
        self.clock = message.clock

        if self.experiment_status == 0:
            self.gradients_initiated = False
            self.cycle_starting_time = 0
            self.day = True
            self.sim_time = 0
            self.grad_time_rate = 0

        if self.experiment_status == 1 and self.gradients_initiated == False: # When models have been spawned, initiate gradients
            self.create_temp_gradient(1)
            self.create_thirst_gradient()
            self.create_food_gradient()
            self.create_predator_gradient(self.predator_x_gradient_position, self.predator_y_gradient_position)
            self.create_mate_gradient(self.mate_x_gradient_position, self.mate_y_gradient_position)
            self.build_gradients()
            self.gradients_initiated = True

            self.update_experiment_status(2) # Gradients initiated

        if self.experiment_status == 5:
            self.update_gradients()
            self.build_gradients()
            self.temperature_cycle()

            if self.plotting == True:
                self.plot_gradient()

            self.temperature_LV()
            self.thirst_LV()
            self.food_LV()
            self.predator_LV()
            self.mate_LV()

            self.publish_Local_views()

        

    def update_experiment_status(self, exp_status):
        self.experiment_status = exp_status
        msg = ExpStatus()
        msg.status = self.experiment_status
        self.pub_exp_status.publish(msg)
        print('Setting experiment status to', exp_status)


    def update_gradients(self):
        if self.grad_time_rate != int(self.sim_time):
            self.grad_time_rate = int(self.sim_time)
            self.create_temp_gradient(int(self.cycle_temperature))

            self.create_predator_gradient(self.predator_x_gradient_position, self.predator_y_gradient_position)
            self.create_mate_gradient(self.mate_x_gradient_position, self.mate_y_gradient_position)


    def temperature_cycle(self):
        if self.day == True:
            # Temperature increases
            self.cycle_temperature = self.sim_time - self.cycle_starting_time
            if self.cycle_temperature >= self.cycle_time:
                self.day = False
                self.cycle_starting_time = self.sim_time

        else:
            # Temperature decreases
            self.cycle_temperature = self.cycle_time - (self.sim_time - self.cycle_starting_time)
            if self.cycle_temperature <= 0:
                self.day = True
                self.cycle_starting_time = self.sim_time

        print("day =", self.day)
        print("cycle_temperature =", self.cycle_temperature)
        print()

        return self.cycle_temperature




    #----------------- GRADIENTS FUNCTIONS -----------------
    def kde_quartic(self,d,h):
        dn=d/h
        P=(15/16)*(1-dn**2)**2
        return P


    def sigmoid(self, x, timestep):
        limit_xInter = 1
        increment_xInter = (limit_xInter*2)/self.cycle_time
        slope = 15

        xInter = -limit_xInter + increment_xInter*timestep
        return 1 / (1 + math.exp(-slope * (x-xInter)))


    def create_temp_gradient(self, currentTstep):
        arena_axis = np.linspace(-1, 1, self.arena_size)
        arena_axis = arena_axis[::-1]

        arena_column = []
        for i in range(self.arena_size):
            arena_column.append(self.sigmoid(arena_axis[i],currentTstep))

        for i in range(len(arena_column)):
            if arena_column[i]>0.98:
                arena_column[i] = 0.98

        temp_gradient = np.tile(arena_column, (self.arena_size,1))
        self.temperature_gradient = temp_gradient.transpose()

        self.mean_temp_gradient = np.mean(self.temperature_gradient)
        msg = Float32()
        msg.data = self.mean_temp_gradient
        self.pub_mean_temperature.publish(msg)



    def create_thirst_gradient(self):
        self.thirst_gradient=[]

        #POINT DATASET
        x= [175 + 110]
        y= [175 + 110]
        #DEFINE GRID SIZE AND RADIUS(h)
        h=280
        #PROCESSING
        for j in range(len(self.xc)):
            intensity_row=[]
            for k in range(len(self.xc[0])):
                kde_value_list=[]
                for i in range(len(x)):
                    #CALCULATE DISTANCE
                    d=math.sqrt((self.xc[j][k]-x[i])**2+(self.yc[j][k]-y[i])**2) 
                    if d<=h:
                        p=self.kde_quartic(d,h)
                    else:
                        p=0
                    kde_value_list.append(p)
                #SUM ALL INTENSITY VALUE
                p_total=sum(kde_value_list)
                intensity_row.append(p_total)
            self.thirst_gradient.append(intensity_row)


    def create_food_gradient(self):
        self.food_gradient=[]

        #POINT DATASET
        x= [175 - 110]
        y= [175 + 110]
        #DEFINE GRID SIZE AND RADIUS(h)
        h=280
        #PROCESSING
        for j in range(len(self.xc)):
            intensity_row=[]
            for k in range(len(self.xc[0])):
                kde_value_list=[]
                for i in range(len(x)):
                    #CALCULATE DISTANCE
                    d=math.sqrt((self.xc[j][k]-x[i])**2+(self.yc[j][k]-y[i])**2) 
                    if d<=h:
                        p=self.kde_quartic(d,h)
                    else:
                        p=0
                    kde_value_list.append(p)
                #SUM ALL INTENSITY VALUE
                p_total=sum(kde_value_list)
                intensity_row.append(p_total)
            self.food_gradient.append(intensity_row)


    def create_predator_gradient(self, predator_X, predator_Y):
        self.predator_gradient=[]

        #POINT DATASET
        x= [predator_X]
        y= [predator_Y]
        #DEFINE GRID SIZE AND RADIUS(h)
        h=200 #280
        #PROCESSING
        for j in range(len(self.xc)):
            intensity_row=[]
            for k in range(len(self.xc[0])):
                kde_value_list=[]
                for i in range(len(x)):
                    #CALCULATE DISTANCE
                    d=math.sqrt((self.xc[j][k]-x[i])**2+(self.yc[j][k]-y[i])**2) 
                    if d<=h:
                        p=self.kde_quartic(d,h)
                    else:
                        p=0
                    kde_value_list.append(p)
                #SUM ALL INTENSITY VALUE
                p_total=sum(kde_value_list)
                intensity_row.append(p_total)
            self.predator_gradient.append(intensity_row)


    def create_mate_gradient(self, mate_X, mate_Y):
        self.mate_gradient=[]

        #POINT DATASET
        x= [mate_X]
        y= [mate_Y]
        #DEFINE GRID SIZE AND RADIUS(h)
        h=280
        #PROCESSING
        for j in range(len(self.xc)):
            intensity_row=[]
            for k in range(len(self.xc[0])):
                kde_value_list=[]
                for i in range(len(x)):
                    #CALCULATE DISTANCE
                    d=math.sqrt((self.xc[j][k]-x[i])**2+(self.yc[j][k]-y[i])**2) 
                    if d<=h:
                        p=self.kde_quartic(d,h)
                    else:
                        p=0
                    kde_value_list.append(p)
                #SUM ALL INTENSITY VALUE
                p_total=sum(kde_value_list)
                intensity_row.append(p_total)
            self.mate_gradient.append(intensity_row)


    def build_gradients(self):

        thirst_min = 100
        thirst_max = 0
        for i in range(len(self.thirst_gradient)):
            for j in range(len(self.thirst_gradient[i])):
                if self.thirst_gradient[i][j] < thirst_min:
                    thirst_min = self.thirst_gradient[i][j]
                if self.thirst_gradient[i][j] > thirst_max:
                    thirst_max = self.thirst_gradient[i][j]

        thirst_intensity=np.array(self.thirst_gradient)
        self.thirst_intensity=thirst_intensity/thirst_max

        food_min = 100
        food_max = 0
        for i in range(len(self.food_gradient)):
            for j in range(len(self.food_gradient[i])):
                if self.food_gradient[i][j] < food_min:
                    food_min = self.food_gradient[i][j]
                if self.food_gradient[i][j] > food_max:
                    food_max = self.food_gradient[i][j]

        food_intensity=np.array(self.food_gradient)
        self.food_intensity=food_intensity/food_max

        predator_min = 100
        predator_max = 0
        for i in range(len(self.predator_gradient)):
            for j in range(len(self.predator_gradient[i])):
                if self.predator_gradient[i][j] < predator_min:
                    predator_min = self.predator_gradient[i][j]
                if self.predator_gradient[i][j] > predator_max:
                    predator_max = self.predator_gradient[i][j]

        predator_intensity=np.array(self.predator_gradient)
        self.predator_intensity=predator_intensity/predator_max


        mate_min = 100
        mate_max = 0
        for i in range(len(self.mate_gradient)):
            for j in range(len(self.mate_gradient[i])):
                if self.mate_gradient[i][j] < mate_min:
                    mate_min = self.mate_gradient[i][j]
                if self.mate_gradient[i][j] > mate_max:
                    mate_max = self.mate_gradient[i][j]

        mate_intensity=np.array(self.mate_gradient)
        self.mate_intensity=mate_intensity/mate_max


    ########################### LOCAL VIEWS ###########################

    def temperature_LV(self): #Local View
        self.q0_temperature, self.q1_temperature, self.q2_temperature, self.q3_temperature = 0,0,0,0
        self.dV_temperature = 1

        for i in range(4):
            for j in range(3):
                self.q0_temperature += self.temperature_gradient[int(self.agent_y_gradient_position) + (j + 1), int(self.agent_x_gradient_position) - (i + 1)]
                self.q1_temperature += self.temperature_gradient[int(self.agent_y_gradient_position) + (j + 1), int(self.agent_x_gradient_position) + (i + 1)]
                self.q2_temperature += self.temperature_gradient[int(self.agent_y_gradient_position) - (j + 1), int(self.agent_x_gradient_position) - (i + 1)]
                self.q3_temperature += self.temperature_gradient[int(self.agent_y_gradient_position) - (j + 1), int(self.agent_x_gradient_position) + (i + 1)]

        self.q0_temperature /= 12
        self.q1_temperature /= 12
        self.q2_temperature /= 12
        self.q3_temperature /= 12


    def thirst_LV(self):
        self.q0_thirst, self.q1_thirst, self.q2_thirst, self.q3_thirst = 0,0,0,0
        self.dV_thirst = 1

        for i in range(4):
            for j in range(3):
                self.q0_thirst += self.thirst_intensity[int(self.agent_y_gradient_position) + (j + 1), int(self.agent_x_gradient_position) - (i + 1)]
                self.q1_thirst += self.thirst_intensity[int(self.agent_y_gradient_position) + (j + 1), int(self.agent_x_gradient_position) + (i + 1)]
                self.q2_thirst += self.thirst_intensity[int(self.agent_y_gradient_position) - (j + 1), int(self.agent_x_gradient_position) - (i + 1)]
                self.q3_thirst += self.thirst_intensity[int(self.agent_y_gradient_position) - (j + 1), int(self.agent_x_gradient_position) + (i + 1)]

        self.q0_thirst /= 12
        self.q1_thirst /= 12
        self.q2_thirst /= 12
        self.q3_thirst /= 12



    def food_LV(self): #Local View
        self.q0_food, self.q1_food, self.q2_food, self.q3_food = 0,0,0,0
        self.dV_food = 1

        for i in range(4):
            for j in range(3):
                self.q0_food += self.food_intensity[int(self.agent_y_gradient_position) + (j + 1), int(self.agent_x_gradient_position) - (i + 1)]
                self.q1_food += self.food_intensity[int(self.agent_y_gradient_position) + (j + 1), int(self.agent_x_gradient_position) + (i + 1)]
                self.q2_food += self.food_intensity[int(self.agent_y_gradient_position) - (j + 1), int(self.agent_x_gradient_position) - (i + 1)]
                self.q3_food += self.food_intensity[int(self.agent_y_gradient_position) - (j + 1), int(self.agent_x_gradient_position) + (i + 1)]

        self.q0_food /= 12
        self.q1_food /= 12
        self.q2_food /= 12
        self.q3_food /= 12


    def predator_LV(self):
        self.q0_predator, self.q1_predator, self.q2_predator, self.q3_predator = 0,0,0,0
        self.dV_predator = 1

        for i in range(4):
            for j in range(3):
                self.q0_predator += self.predator_intensity[int(self.agent_y_gradient_position) + (j + 1), int(self.agent_x_gradient_position) - (i + 1)]
                self.q1_predator += self.predator_intensity[int(self.agent_y_gradient_position) + (j + 1), int(self.agent_x_gradient_position) + (i + 1)]
                self.q2_predator += self.predator_intensity[int(self.agent_y_gradient_position) - (j + 1), int(self.agent_x_gradient_position) - (i + 1)]
                self.q3_predator += self.predator_intensity[int(self.agent_y_gradient_position) - (j + 1), int(self.agent_x_gradient_position) + (i + 1)]

        self.q0_predator /= 12
        self.q1_predator /= 12
        self.q2_predator /= 12
        self.q3_predator /= 12


    def mate_LV(self):
        self.q0_mate, self.q1_mate, self.q2_mate, self.q3_mate = 0,0,0,0
        self.dV_mate = 1

        for i in range(4):
            for j in range(3):
                self.q0_mate += self.mate_intensity[int(self.agent_y_gradient_position) + (j + 1), int(self.agent_x_gradient_position) - (i + 1)]
                self.q1_mate += self.mate_intensity[int(self.agent_y_gradient_position) + (j + 1), int(self.agent_x_gradient_position) + (i + 1)]
                self.q2_mate += self.mate_intensity[int(self.agent_y_gradient_position) - (j + 1), int(self.agent_x_gradient_position) - (i + 1)]
                self.q3_mate += self.mate_intensity[int(self.agent_y_gradient_position) - (j + 1), int(self.agent_x_gradient_position) + (i + 1)]

        self.q0_mate /= 12
        self.q1_mate /= 12
        self.q2_mate /= 12
        self.q3_mate /= 12


    def publish_Local_views(self):
        command_message = LocalViews()
        command_message.q0_temperature = self.q0_temperature
        command_message.q1_temperature = self.q1_temperature
        command_message.q2_temperature = self.q2_temperature
        command_message.q3_temperature = self.q3_temperature
        command_message.q0_thirst = self.q0_thirst
        command_message.q1_thirst = self.q1_thirst
        command_message.q2_thirst = self.q2_thirst
        command_message.q3_thirst = self.q3_thirst
        command_message.q0_food = self.q0_food
        command_message.q1_food = self.q1_food
        command_message.q2_food = self.q2_food
        command_message.q3_food = self.q3_food
        command_message.q0_predator = self.q0_predator
        command_message.q1_predator = self.q1_predator
        command_message.q2_predator = self.q2_predator
        command_message.q3_predator = self.q3_predator
        command_message.q0_mate = self.q0_mate
        command_message.q1_mate = self.q1_mate
        command_message.q2_mate = self.q2_mate
        command_message.q3_mate = self.q3_mate

        self.LV__publisher.publish(command_message)


    def plot_gradient(self):
        #...........  GRADIENTS ...........
        self.ax1[0].cla()
        self.ax1[0].grid(False)
        self.ax1[0].plot(self.agent_x_gradient_position,self.agent_y_gradient_position,'go')
        self.ax1[0].set_title("Temperature")
        self.ax1[0].pcolormesh(self.x_mesh,self.y_mesh,self.temperature_gradient, cmap = plt.get_cmap('bwr'))

        self.ax1[1].cla()
        self.ax1[1].grid(False)
        self.ax1[1].plot(self.agent_x_gradient_position,self.agent_y_gradient_position,'go')
        self.ax1[1].set_title("Food")
        self.ax1[1].pcolormesh(self.x_mesh,self.y_mesh,self.food_intensity, cmap = plt.get_cmap('Reds'))

        self.ax1[2].cla()
        self.ax1[2].grid(False)
        self.ax1[2].plot(self.agent_x_gradient_position,self.agent_y_gradient_position,'go')
        self.ax1[2].set_title("Hydration")
        self.ax1[2].pcolormesh(self.x_mesh,self.y_mesh,self.thirst_intensity, cmap = plt.get_cmap('Blues'))

        self.ax1[3].cla()
        self.ax1[3].grid(False)
        self.ax1[3].plot(self.agent_x_gradient_position,self.agent_y_gradient_position,'go')
        self.ax1[3].plot(self.predator_x_gradient_position,self.predator_y_gradient_position,'ro')
        self.ax1[3].set_title("Predator")
        self.ax1[3].pcolormesh(self.x_mesh,self.y_mesh,self.predator_intensity, cmap = plt.get_cmap('Greens_r'))

        self.ax1[4].cla()
        self.ax1[4].grid(False)
        self.ax1[4].plot(self.agent_x_gradient_position,self.agent_y_gradient_position,'go')
        self.ax1[4].plot(self.mate_x_gradient_position,self.mate_y_gradient_position,'bo')
        self.ax1[4].set_title("mate")
        self.ax1[4].pcolormesh(self.x_mesh,self.y_mesh,self.mate_intensity, cmap = plt.get_cmap('Purples'))

        self.fig_gradients.canvas.flush_events()



def main(args=None):
    rclpy.init(args=args)
    gradients = Gradients()
    rclpy.spin(gradients)
    gradients.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()