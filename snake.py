#Imports
import numpy as np
import pybullet as p
import pyrosim.pyrosim as pyrosim
import constants as c
import random
import os
import time

class SNAKE:
    def __init__(self, nextAvailableID):
        os.system("rm brain*.nndf")
        os.system("rm fitness*.txt")
        
        self.myID = nextAvailableID
        self.weights = np.random.rand(c.numSensorNeurons,c.numMotorNeurons)
        self.weights = 2*self.weights - 1

    
    def Create_World(self):
        pyrosim.Start_SDF("world.sdf")
        pyrosim.End()


    def Create_SnakeBody(self):
        #Define length of snake
        self.length = np.random.randint(2,10)
        print("Number of links: " + str(self.length))
        
        
        #Determine how many sensors and motors
        self.sensor_coins = np.random.randint(0,2, size = self.length)
        self.motor_coins = np.random.randint(0,2, size = self.length - 1)
        
        
        #Start simulation by creating snake's head and first link
        l_parent = random.uniform(0.1,3.0)
        w_parent = random.uniform(0.1,3.0)
        h_parent = random.uniform(0.0,3.0)
        l_child = random.uniform(0.1,3.0)
        w_child = random.uniform(0.1,3.0)
        h_child = random.uniform(0.1,3.0)

        pyrosim.Start_URDF("body.urdf")
        
        print(self.sensor_coins)

        if self.sensor_coins[0] == 1:
            pyrosim.Send_Cube(name="Head", pos=[0,0,1.5] , size=[l_parent,w_parent,h_parent], g_value = 1.0, b_value = 0.0)
        elif self.sensor_coins[0] == 0:
            pyrosim.Send_Cube(name="Head", pos=[0,0,1.5] , size=[l_parent,w_parent,h_parent], g_value = 0.0, b_value = 1.0)
        
        pyrosim.Send_Joint(name = "Head_0" , parent= "Head" , child = "0" , type = "revolute", position = [l_parent/2,0,1.5], jointAxis = "0 1 0")
        
        if self.sensor_coins[1] == 1:
            pyrosim.Send_Cube(name="0", pos=[l_child/2,0,0] , size=[l_child,w_child,h_child], g_value = 1.0, b_value = 0.0)
        elif self.sensor_coins[1] == 0:
            pyrosim.Send_Cube(name="0", pos=[l_child/2,0,0] , size=[l_child,w_child,h_child], g_value = 0.0, b_value = 1.0)
        
        
        #Create additional links
        if self.length > 2:
            for linkNumber in range(0,self.length-2):
                l_parent = l_child
                w_parent = w_child
                h_parent = h_child
                l_child = random.uniform(0.1,3.0)
                w_child = random.uniform(0.1,3.0)
                h_child = random.uniform(0.1,3.0)
                
                if self.sensor_coins[linkNumber+2] == 1:
                    pyrosim.Send_Joint(name = str(linkNumber) + "_" + str(linkNumber+1) , parent= str(linkNumber) , child = str(linkNumber+1) , type = "revolute", position = [l_parent,0,0], jointAxis = "0 1 0")
                    pyrosim.Send_Cube(name=str(linkNumber+1), pos=[l_child/2,0,0] , size=[l_child,w_child,h_child], g_value = 1.0, b_value = 0.0)
                    
                else:
                    pyrosim.Send_Joint(name = str(linkNumber) + "_" + str(linkNumber+1) , parent= str(linkNumber) , child = str(linkNumber+1) , type = "revolute", position = [l_parent,0,0], jointAxis = "0 1 0")
                    pyrosim.Send_Cube(name=str(linkNumber+1), pos=[l_child/2,0,0] , size=[l_child,w_child,h_child], g_value = 0.0, b_value = 1.0)
        
        
        #End
        pyrosim.End()

        
    def Create_SnakeBrain(self):      
        #Start neural network
        pyrosim.Start_NeuralNetwork("brain" + str(self.myID) + ".nndf")
        
        
        #Count sensors and motors
        number_sensors = np.count_nonzero(self.sensor_coins)
        number_motors = np.count_nonzero(self.motor_coins)
        
        sensors = []
        motors = []
        
        for i, sensor in enumerate(self.sensor_coins):
            if sensor == 1:
                sensors.append(i)
        
        for j, motor in enumerate(self.motor_coins):
            if motor == 1:
                motors.append(j)
                
        print(sensors)
        print(motors)
        
        #Establish weights for synapses        
        self.weights = np.random.rand(number_sensors,number_motors)
        self.weights = 2*self.weights - 1
        
        
        #Send sensors
        for linkNumber in range(0,self.length):
            if self.sensor_coins[linkNumber] == 1:
                if linkNumber == 0:
                    pyrosim.Send_Sensor_Neuron(name = linkNumber, linkName = "Head")
                else:
                    pyrosim.Send_Sensor_Neuron(name = linkNumber, linkName = str(linkNumber-1))
        
        #Send motors
        for jointNumber in range(0,self.length-1):
            if self.motor_coins[jointNumber] == 1:
                if jointNumber == 0:
                    pyrosim.Send_Motor_Neuron(name = jointNumber + number_sensors, jointName = "Head_0")
                if jointNumber > 0:
                    pyrosim.Send_Motor_Neuron(name = jointNumber + number_sensors, jointName = str(jointNumber-1) + "_" + str(jointNumber))


        #Send synapses
        for currentRow, source in enumerate(sensors):
            for currentColumn, target in enumerate(motors):
                pyrosim.Send_Synapse(sourceNeuronName = source, targetNeuronName = target+number_sensors, weight = self.weights[currentRow][currentColumn])  

    
        #End
        pyrosim.End()

     
    def Start_Simulation(self, directOrGUI):
        self.Create_World()
        self.Create_SnakeBody()
        self.Create_SnakeBrain()
        
        os.system("python3 simulate.py " + str(directOrGUI) + " " + str(self.myID) + ' 2>&1')
        