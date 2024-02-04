from controller import Robot, Receiver, Emitter
import sys,struct,math
import numpy as np
import mlp as ntw

class Controller:
    def __init__(self, robot):
        # Robot Parameters
        # Please, do not change these parameters
        self.robot = robot
        self.time_step = 300 # ms
        self.max_speed = 1  # m/s

        self.sensor_data_file = open("sensor_data.csv", "w")
        #sensor_data = ["gleft", "gmiddle", "gright", "ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ls0", "ls1", "ls2", "ls3", "ls4", "ls5", "ls6", "ls7"]
        #self.sensor_data_file.write("\n")

        # MLP Parameters and Variables

        ###########
        ### DEFINE below the architecture of your MLP network:

        ### Add the number of neurons for input layer, hidden layer and output layer.
        ### The number of neurons should be in between of 1 to 20.
        ### Number of hidden layers should be one or two.

        self.number_input_layer = 10
        # Example with one hidden layers: self.number_hidden_layer = [5]
        # Example with two hidden layers: self.number_hidden_layer = [7,5]
        self.number_hidden_layer = [10, 10, 10]
        self.number_output_layer = 2

        # Create a list with the number of neurons per layer
        self.number_neuros_per_layer = []
        self.number_neuros_per_layer.append(self.number_input_layer)
        self.number_neuros_per_layer.extend(self.number_hidden_layer)
        self.number_neuros_per_layer.append(self.number_output_layer)

        # Initialize the network
        self.network = ntw.MLP(self.number_neuros_per_layer)
        self.inputs = []

        # Calculate the number of weights of your MLP
        self.number_weights = 0
        for n in range(1,len(self.number_neuros_per_layer)):
            if(n == 1):
                # Input + bias
                self.number_weights += (self.number_neuros_per_layer[n-1]+1)*self.number_neuros_per_layer[n]
            else:
                self.number_weights += self.number_neuros_per_layer[n-1]*self.number_neuros_per_layer[n]

        # Enable Motors
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        self.velocity_left = 0
        self.velocity_right = 0

        # Enable Proximity Sensors
        self.proximity_sensors = []
        for i in range(8):
            sensor_name = 'ps' + str(i)
            self.proximity_sensors.append(self.robot.getDevice(sensor_name))
            self.proximity_sensors[i].enable(self.time_step)

        # Enable Ground Sensors
        self.left_ir = self.robot.getDevice('gs0')
        self.left_ir.enable(self.time_step)
        self.center_ir = self.robot.getDevice('gs1')
        self.center_ir.enable(self.time_step)
        self.right_ir = self.robot.getDevice('gs2')
        self.right_ir.enable(self.time_step)
        
        # Enable Light Sensors
        self.light_sensors = []
        for i in range(8):
            sensor_name = 'ls' + str(i)
            self.light_sensors.append(self.robot.getDevice(sensor_name))
            self.light_sensors[i].enable(self.time_step)

        # Enable Emitter and Receiver (to communicate with the Supervisor)
        self.emitter = self.robot.getDevice("emitter")
        self.receiver = self.robot.getDevice("receiver")
        self.receiver.enable(self.time_step)
        self.receivedData = ""
        self.receivedDataPrevious = ""
        self.flagMessage = False

        # Fitness value (initialization fitness parameters once)
        self.fitness_values = []
        self.fitness = 0
        
        #Variable to determine if the beacon is activated
        self.tmpLight = 0 

    def check_for_new_genes(self):
        if(self.flagMessage == True):
                # Split the list based on the number of layers of your network
                part = []
                for n in range(1,len(self.number_neuros_per_layer)):
                    if(n == 1):
                        part.append((self.number_neuros_per_layer[n-1]+1)*(self.number_neuros_per_layer[n]))
                    else:
                        part.append(self.number_neuros_per_layer[n-1]*self.number_neuros_per_layer[n])

                # Set the weights of the network
                data = []
                weightsPart = []
                sum = 0
                for n in range(1,len(self.number_neuros_per_layer)):
                    if(n == 1):
                        weightsPart.append(self.receivedData[n-1:part[n-1]])
                    elif(n == (len(self.number_neuros_per_layer)-1)):
                        weightsPart.append(self.receivedData[sum:])
                    else:
                        weightsPart.append(self.receivedData[sum:sum+part[n-1]])
                    sum += part[n-1]
                for n in range(1,len(self.number_neuros_per_layer)):
                    if(n == 1):
                        weightsPart[n-1] = weightsPart[n-1].reshape([self.number_neuros_per_layer[n-1]+1,self.number_neuros_per_layer[n]])
                    else:
                        weightsPart[n-1] = weightsPart[n-1].reshape([self.number_neuros_per_layer[n-1],self.number_neuros_per_layer[n]])
                    data.append(weightsPart[n-1])
                self.network.weights = data

                #Reset fitness list
                self.fitness_values = []
                
                

    def clip_value(self,value,min_max):
        if (value > min_max):
            return min_max;
        elif (value < -min_max):
            return -min_max;
        return value;

    def sense_compute_and_actuate(self):
        # MLP:
        #   Input == sensory data
        #   Output == motors commands
        output = self.network.propagate_forward(self.inputs)
        self.velocity_left = output[0]
        self.velocity_right = output[1]

        # Multiply the motor values by 3 to increase the velocities
        self.left_motor.setVelocity(self.velocity_left*5)
        self.right_motor.setVelocity(self.velocity_right*5)

    def calculate_fitness(self):

        forwardFitness = (self.velocity_left+self.velocity_right)/2
        #print("forwardFitness : {}" .format(forwardFitness))
        
        ### Define the fitness function to encourage the robot to follow the line
        followLineFitness = (1 - np.sum(self.inputs[0:3])/3)
        #print("followLineFitness : {}".format(followLineFitness))
        
        ### Define the fitness function to avself.inputs[3:7]oid collision
        avoidCollisionFitness = (1 - max(self.inputs[3:7]))
        #print("avoidCollisionFitness : {}".format(avoidCollisionFitness))
        
        ### Define the fitness function to avoid spining behaviour
        spinningFitness = 1-math.sqrt(abs(self.velocity_right - self.velocity_left))
        #print("spinningFitness : {}".format(spinningFitness))
        
        ### Define the fitness function of this iteration which should be a combination of the previous functions         
        combinedFitness = forwardFitness * spinningFitness * avoidCollisionFitness * followLineFitness 
        #print("combinedFitness : {}".format(combinedFitness))
        
        self.fitness_values.append(combinedFitness)
        self.fitness = np.mean(self.fitness_values)

    def handle_emitter(self):
        # Send the self.fitness value to the supervisor
        data = str(self.number_weights)
        data = "weights: " + data
        string_message = str(data)
        string_message = string_message.encode("utf-8")
        #print("Robot send:", string_message)
        self.emitter.send(string_message)

        # Send the self.fitness value to the supervisor
        data = str(self.fitness)
        data = "fitness: " + data
        string_message = str(data)
        string_message = string_message.encode("utf-8")
        #print("Robot send fitness:", string_message)
        self.emitter.send(string_message)

    def handle_receiver(self):
        if self.receiver.getQueueLength() > 0:
            self.tmpLight = 0 #Light flag reset to 0 at the beginning of each simulation
            #print("New simulation")
            while(self.receiver.getQueueLength() > 0):
                # Adjust the Data to our model
                #Webots 2022:
                #self.receivedData = self.receiver.getData().decode("utf-8")
                #Webots 2023:
                self.receivedData = self.receiver.getString()
                self.receivedData = self.receivedData[1:-1]
                self.receivedData = self.receivedData.split()
                x = np.array(self.receivedData)
                self.receivedData = x.astype(float)
                #print("Controller handle receiver data:", self.receivedData)
                self.receiver.nextPacket()

            # Is it a new Genotype?
            if(np.array_equal(self.receivedDataPrevious,self.receivedData) == False):
                self.flagMessage = True

            else:
                self.flagMessage = False

            self.receivedDataPrevious = self.receivedData
        else:
            #print("Controller receiver q is empty")
            self.flagMessage = False

    def run_robot(self):
        # Main Loop
        while self.robot.step(self.time_step) != -1:
            # This is used to store the current input data from the sensors
            self.inputs = []

            # Emitter and Receiver
            # Check if there are messages to be sent or read to/from our Supervisor
            self.handle_emitter()
            self.handle_receiver()

            #Creating array for all sensors for them to be plotted
            sensor_data = []

            # Read Ground Sensors
            left = self.left_ir.getValue()
            center = self.center_ir.getValue()
            right = self.right_ir.getValue()



            #print("Ground Sensors \n    left {} center {} right {}".format(left,center,right))

            ### Please adjust the ground sensors values to facilitate learning : 0 - 1023
            min_gs = 350
            max_gs = 600

            if(left > max_gs): left = max_gs
            if(center > max_gs): center = max_gs
            if(right > max_gs): right = max_gs
            if(left < min_gs): left = min_gs
            if(center < min_gs): center = min_gs
            if(right < min_gs): right = min_gs

            # Normalize the values between 0 and 1 and save data
            self.inputs.append((left-min_gs)/(max_gs-min_gs))
            self.inputs.append((center-min_gs)/(max_gs-min_gs))
            self.inputs.append((right-min_gs)/(max_gs-min_gs))

            #print("Ground Sensors \n    left {} center {} right {}".format(self.inputs[0],self.inputs[1],self.inputs[2]))
            
            
            # Read Distance Sensors
            for i in range(8):
                ### Select the distance sensors that you will use
                if(i==0 or i==1 or i ==2 or i==5 or i==6 or i==7):
                    temp = self.proximity_sensors[i].getValue()

                    ### Please adjust the distance sensors values to facilitate learning : 0 - 4095
                    min_ds = 85
                    max_ds = 160 #145

                    if(temp > max_ds): temp = max_ds
                    if(temp < min_ds): temp = min_ds

                    # Normalize the values between 0 and 1 and save data
                    self.inputs.append((temp-min_ds)/(max_ds-min_ds))
                    sensor_data.append((temp-min_ds)/(max_ds-min_ds))
                    
                    #print("Distance Sensors - Index: {}  Value: {}".format(i,self.proximity_sensors[i].getValue()))
            

                    
            # Read Light Sensors
            
            for i in range(8):       
                temp = self.light_sensors[i].getValue()
                # Adjust Values : 0-4095
                min_ls = 1000
                max_ls = 3800
                if(temp > max_ls): temp = max_ls
                if(temp < min_ls): temp = min_ls
                # Save Data
                value = (temp-min_ls)/(max_ls-min_ls)
                sensor_data.append((temp-min_ls)/(max_ls-min_ls))
                
                #print("Light Sensors - Index: {}  Value: {}".format(i,self.light_sensors[i].getValue()))
                if(value<0.5):
                    self.tmpLight = 1
            #Append only a boolean value depeding if the beacon is ON or OFF
            self.inputs.append(self.tmpLight)
            #print(self.tmpLight)


                
            
            # GA Iteration
            # Verify if there is a new genotype to be used that was sent from Supervisor
            self.check_for_new_genes()
            # The robot's actuation (motor values) based on the output of the MLP
            self.sense_compute_and_actuate()
            # Calculate the fitnes value of the current iteration
            self.calculate_fitness()
            
            # Adding ground sensor data to be visualized
            sensor_data.append((left-min_gs)/(max_gs-min_gs))
            sensor_data.append((center-min_gs)/(max_gs-min_gs))
            sensor_data.append((right-min_gs)/(max_gs-min_gs))
            
            sensor_data.append(self.velocity_left*5) #left wheel speed data
            sensor_data.append(self.velocity_right*5) #right wheel speed data
            
            # Writing all the data on csv file to visualize data 
            # Array structure: [gleft, gmiddle, gright, ps0, ps1, ps2, ps3, ps4, ps5, ls0, ls1, ls2, ls3, ls4, ls5, ls6, ls7, leftVel, righVel]
            self.sensor_data_file.write(", ".join(map(str, sensor_data)) + "\n")
            
            # End of the iteration

if __name__ == "__main__":
    # Call Robot function to initialize the robot
    my_robot = Robot()
    # Initialize the parameters of the controller by sending my_robot
    controller = Controller(my_robot)
    # Run the controller
    controller.run_robot()

