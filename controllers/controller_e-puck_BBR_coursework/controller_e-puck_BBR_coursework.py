from controller import Robot
from datetime import datetime
import math
import numpy as np


class Controller:
    def __init__(self, robot):        
        # Robot Parameters
        self.robot = robot
        self.time_step = 32 # ms
        self.max_speed = 1  # m/s
        self.initial_time = self.robot.getTime()
        
        #timing to sample data from sensors
        self.last_sample_time = datetime.now()
        self.sample_interval = 0.5
        
        #used to create plots of data 
        self.sensor_data_file = open("sensor_data.csv", "w")
 
        # Enable Motors
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        self.velocity_left = 0
        self.velocity_right = 0

        # Enable Distance Sensors
        self.distance_sensors = []
        for i in range(8):
            sensor_name = 'ps' + str(i)
            self.distance_sensors.append(self.robot.getDevice(sensor_name))
            self.distance_sensors[i].enable(self.time_step)
    
        # Enable Light Sensors
        self.light_sensors = []
        for i in range(8):
            sensor_name = 'ls' + str(i)
            self.light_sensors.append(self.robot.getDevice(sensor_name))
            self.light_sensors[i].enable(self.time_step)
       
        # Enable Ground Sensors
        self.left_ir = self.robot.getDevice('gs0')
        self.left_ir.enable(self.time_step)
        self.center_ir = self.robot.getDevice('gs1')
        self.center_ir.enable(self.time_step)
        self.right_ir = self.robot.getDevice('gs2')
        self.right_ir.enable(self.time_step)
        
        # Data
        self.inputs = []
        self.inputsPrevious = []
        
        # Flag
        self.flag_turn = 0
        self.flag_beacon = 0
        self.flag_obstacle = 0
        self.flag_line = 1
        
        self.count = 0
        self.direction="right"
        
    def clip_value(self,value,min_max):
        if (value > min_max):
            return min_max;
        elif (value < -min_max):
            return -min_max;
        return value;

    def sense_compute_and_actuate(self):
          
        if(len(self.inputs) > 0 and len(self.inputsPrevious) > 0):
            # Check for any possible collision
            if(max(self.inputs[3],self.inputs[10]) > 0.1):
                # Time
                self.flag_obstacle = 1
                time = datetime.now()
                #print("({} - {}) Object or walls detected!".format(time.second, time.microsecond))
                
            #Check for beacon activated
            if(np.min(self.inputs[11:19])<0.6):
                if(self.flag_beacon==0):
                    self.flag_beacon = 1
                    self.direction = "left"
                    print("Beacon activated")
                    
            if(np.min([self.inputs[3:6],self.inputs[8:11]])> 0.1):
                run_time = self.robot.getTime() - self.initial_time;
                print("Goal achieved in : {}".format(run_time)) 
            
            if(self.flag_obstacle):
                if(np.max(self.inputs[3:6])>np.max(self.inputs[8:11])): #obstacle on right
                    print("obstacle on right")
                    self.direction = "left"
                    if(self.inputs[4]>self.inputs[5]):
                        self.velocity_left = -0.3
                        self.velocity_right = 0.3
                        print("go further")
                    elif(self.inputs[3]<self.inputs[5]):
                        self.velocity_left = 1
                        self.velocity_right = 0.5
                        print("get closer")
                elif(np.max(self.inputs[3:5])<np.max(self.inputs[9:11])): #obstacle on left
                    self.direction = "right"
                    print("obstacle on left")
                    if(self.inputs[9]>self.inputs[8]):
                        self.velocity_left = 0.3
                        self.velocity_right = -0.3
                        print("go further")
                    elif(self.inputs[10]<self.inputs[8]):
                        self.velocity_left = 0.5
                        self.velocity_right = 1
                        print("get closer")
                if((np.min(self.inputs[0:3])-np.min(self.inputsPrevious[0:3])) < -0.2):
                        self.flag_obstacle = 0
                        self.count = self.count + 1
                        print("obstacle passed")
                        
            # Turn
            else: 
                if(self.flag_turn):
                    if(self.direction=="left"): #Left
                        self.velocity_left = -0.5;
                        self.velocity_right = 0.5;
                        print("turn left")
                    else: #Right
                        self.velocity_left = 0.5;
                        self.velocity_right = -0.5;
                        print("turn right")
                    if(np.min(self.inputs[0:3]) < 0.35):
                        self.flag_turn = 0
                        if(self.count == 2):
                            if(self.flag_beacon==1): self.direction="left"
                            else: self.direction = "right"
                        
                else:        
                    # Check end of line : black is 0, white is 1 
                    if((np.min(self.inputs[0:3])-np.min(self.inputsPrevious[0:3])) > 0.2):
                        self.flag_turn = 1
                    else:    
                        # Follow the line    
                        if(self.inputs[0] < self.inputs[1] and self.inputs[0] < self.inputs[2]):
                            self.velocity_left = 0.5;
                            self.velocity_right = 1;
                        elif(self.inputs[1] < self.inputs[0] and self.inputs[1] < self.inputs[2]):
                            self.velocity_left = 1;
                            self.velocity_right = 1;    
                        elif(self.inputs[2] < self.inputs[0] and self.inputs[2] < self.inputs[1]):
                            self.velocity_left = 1;
                            self.velocity_right = 0.5;           
                
        self.left_motor.setVelocity(self.velocity_left)
        self.right_motor.setVelocity(self.velocity_right)

    def run_robot(self):        
        # Main Loop
        count = 0;
        inputs_avg = []
        while self.robot.step(self.time_step) != -1:
            # Read Ground Sensors
            self.inputs = []
            left = self.left_ir.getValue()
            center = self.center_ir.getValue()
            right = self.right_ir.getValue()

            # Adjust Values
            min_gs = 0
            max_gs = 1000
            if(left > max_gs): left = max_gs
            if(center > max_gs): center = max_gs
            if(right > max_gs): right = max_gs
            if(left < min_gs): left = min_gs
            if(center < min_gs): center = min_gs
            if(right < min_gs): right = min_gs
            
            # Save Data
            self.inputs.append((left-min_gs)/(max_gs-min_gs))
            self.inputs.append((center-min_gs)/(max_gs-min_gs))
            self.inputs.append((right-min_gs)/(max_gs-min_gs))
            #print("Ground Sensors \n    left {} center {} right {}".format(self.inputs[0],self.inputs[1],self.inputs[2]))
            
            # Read Distance Sensors
            for i in range(8):
                if(i==0 or i==1 or i==2 or i==3 or i==4 or i==5 or i==6 or i==7):        
                    temp = self.distance_sensors[i].getValue()
                    # Adjust Values
                    min_ds = 0
                    max_ds = 4095
                    if(temp > max_ds): temp = max_ds
                    if(temp < min_ds): temp = min_ds
                    # Save Data
                    self.inputs.append((temp-min_ds)/(max_ds-min_ds))
                    #print("Distance Sensors - Index: {}  Value: {}".format(i,self.proximity_sensors[i].getValue()))
                    
            # Read Light Sensors
            for i in range(8):       
                temp = self.light_sensors[i].getValue()
                # Adjust Values
                min_ls = 0
                max_ls = 4095
                if(temp > max_ls): temp = max_ls
                if(temp < min_ls): temp = min_ls
                # Save Data
                self.inputs.append((temp-min_ls)/(max_ls-min_ls))
                #print("Light Sensors - Index: {}  Value: {}".format(i,self.light_sensors[i].getValue()))
      
            # Smooth filter (Average)
            smooth = 10
            if(count == smooth):
                inputs_avg = [sum(x) for x in zip(*inputs_avg)]
                self.inputs = [x/smooth for x in inputs_avg]
                # Compute and actuate
                self.sense_compute_and_actuate()
                # Reset
                count = 0
                inputs_avg = []
                self.inputsPrevious = self.inputs
            else:
                inputs_avg.append(self.inputs)
                count = count + 1
                
            
if __name__ == "__main__":
    my_robot = Robot()
    controller = Controller(my_robot)
    controller.run_robot()
    