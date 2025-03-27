import rclpy
import matplotlib.pyplot as plt
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PointStamped
from webots_ros2_msgs.msg import FloatStamped

from pathlib import Path
ws_path = str(Path(__file__).parents[6])
print(ws_path)

import configparser
parameters = configparser.ConfigParser()
parameters.read(ws_path + '/src/config.ini')

class Plotting(Node):

    def __init__(self):
        super().__init__('plotting')
        self.get_logger().info('------------- ----------------- Plotting -------------- -------------')

        #----------------- ROS TOPIC SUBSCRIPTIONS AND PUBLICATIONS -----------------
        self.create_subscription(Clock, '/clock', self.clock_callback, 1)

        self.create_subscription(PointStamped, '/epuck_agent/gps', self.agent_GPS_callback, 1)
        self.create_subscription(FloatStamped, '/epuck_agent/compass/bearing', self.agent_Compass_callback, 1)
        self.create_subscription(PointStamped, '/epuck_predator/gps', self.predator_GPS_callback, 1)
        self.create_subscription(FloatStamped, '/epuck_predator/compass/bearing', self.predator_Compass_callback, 1)
        self.create_subscription(PointStamped, '/epuck_peer/gps', self.mate_GPS_callback, 1)
        self.create_subscription(FloatStamped, '/epuck_peer/compass/bearing', self.mate_Compass_callback, 1)


        #----------------- PLOTTING HYPERPAPRAMETERS -----------------
        self.plotting = int(parameters.get('Experiment', 'plotting'))
        self.agent_translation = parameters.get('Environment', 'agent_translation')
        self.mate_translation = parameters.get('Environment', 'mate_translation')
        self.predator_translation = parameters.get('Environment', 'predator_translation')
        self.food_translation = parameters.get('Environment', 'food_translation')
        self.water_translation = parameters.get('Environment', 'water_translation')

        if self.plotting == True:
            plt.ion()
            plt.style.use('seaborn')
            self.fig_gradients, self.ax1 = plt.subplots(5, 1,figsize=(3,15))
            #fig2, ax2 = plt.subplots(1, 2,figsize=(20,4))
            #fig2.tight_layout(pad=2.0)


#----------------- BUILD GRADIENTS -----------------
        self.create_temp_gradient(1)
        self.create_thirst_gradient()
        self.create_food_gradient()
        self.create_predator_gradient(self.predator_X, self.predator_Y)
        self.create_mate_gradient(self.mate_X, self.mate_Y)
        self.build_gradients()


    #----------------- ROS CALLBACK FUNCTIONS -----------------

    def agent_GPS_callback(self, message):
        self.agent_X = message.point.x
        self.agent_Y = message.point.y
        self.agent_x_gradient_position = (self.arena_size/2) + (self.agent_X * 100)
        self.agent_y_gradient_position = (self.arena_size/2) + (self.agent_Y * 100)

    def predator_GPS_callback(self, message):
        self.predator_X = message.point.x
        self.predator_Y = message.point.y
        self.predator_x_gradient_position = (self.arena_size/2) + (self.predator_X * 100)
        self.predator_y_gradient_position = (self.arena_size/2) + (self.predator_Y * 100)

    def mate_GPS_callback(self, message):
        self.mate_X = message.point.x
        self.mate_Y = message.point.y
        self.mate_x_gradient_position = (self.arena_size/2) + (self.mate_X * 100)
        self.mate_y_gradient_position = (self.arena_size/2) + (self.mate_Y * 100)


    def clock_callback(self, message):
        self.clock = message.clock
        self.secs = self.clock.sec
        self.nanosecs = self.clock.nanosec/1000000000
        self.time = self.secs + self.nanosecs

        if self.plotting == True:
            self.plot_gradient()


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
        print("predator_X = ", predator_X)

        #POINT DATASET
        x= [predator_X]
        y= [predator_Y]
        #DEFINE GRID SIZE AND RADIUS(h)
        h=150
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
        h=200
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

