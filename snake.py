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
        pyrosim.Start_URDF("body.urdf")
        pyrosim.Send_Cube(name="Head", pos=[0,0,0.5] , size=[1,1,1]) 
        
        #Head
        pyrosim.Send_Joint(name = "Head_0" , parent= "Head" , child = "0" , type = "revolute", position = [0.5,0,0.5], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name="0", pos=[1,0,0] , size=[2,1.5,1])
    
        pyrosim.Send_Joint(name = "0_1" , parent= "0" , child = "1" , type = "revolute", position = [2,0,0], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name="1", pos=[0.5,0,0] , size=[1,1,1])
        
        pyrosim.Send_Joint(name = "1_2" , parent= "1" , child = "2" , type = "revolute", position = [1,0,0], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name="2", pos=[1,0,0] , size=[2,0.5,0.5])
    
        
        pyrosim.End()
        
        
    def Create_SnakeBrain(self):
        pyrosim.Start_NeuralNetwork("brain" + str(self.myID) + ".nndf")
    
        pyrosim.Send_Sensor_Neuron(name = 0, linkName = "Head")
        pyrosim.Send_Sensor_Neuron(name = 1, linkName = "0")
        pyrosim.Send_Sensor_Neuron(name = 2, linkName = "1")
        pyrosim.Send_Sensor_Neuron(name = 3, linkName = "2")


        pyrosim.Send_Motor_Neuron(name = 4, jointName = "Head_0")
        pyrosim.Send_Motor_Neuron(name = 5, jointName = "0_1")
        pyrosim.Send_Motor_Neuron(name = 6, jointName = "1_2")

    
        for currentRow in range(4):
            for currentColumn in range(3):
                pyrosim.Send_Synapse(sourceNeuronName = currentRow, targetNeuronName = currentColumn+3, weight = self.weights[currentRow][currentColumn])

        pyrosim.End()




    def Create_SnakeBody1(self):
        #Define length of snake
        self.length = np.random.randint(2,10)
        print("Number of links: " + str(self.length))
        
        #Start simulation by creating snake's head and first link
        l_parent = random.uniform(0.1,3.0)
        w_parent = random.uniform(0.1,3.0)
        h_parent = random.uniform(0.0,3.0)
        l_child = random.uniform(0.1,3.0)
        w_child = random.uniform(0.1,3.0)
        h_child = random.uniform(0.1,3.0)

        pyrosim.Start_URDF("body.urdf")

        pyrosim.Send_Cube(name="Head", pos=[0,0,1.5] , size=[l_parent,w_parent,h_parent])
        pyrosim.Send_Joint(name = "Head_0" , parent= "Head" , child = "0" , type = "revolute", position = [l_parent/2,0,1.5], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name="0", pos=[l_child/2,0,0] , size=[l_child,w_child,h_child])
        
        #Create additional links
        if self.length > 2:
            for linkNumber in range(0,self.length-2):
                l_parent = l_child
                w_parent = w_child
                h_parent = h_child
                l_child = random.uniform(0.1,3.0)
                w_child = random.uniform(0.1,3.0)
                h_child = random.uniform(0.1,3.0)
                
                pyrosim.Send_Joint(name = str(linkNumber) + "_" + str(linkNumber+1) , parent= str(linkNumber) , child = str(linkNumber+1) , type = "revolute", position = [l_parent,0,0], jointAxis = "0 1 0")
                pyrosim.Send_Cube(name=str(linkNumber+1), pos=[l_child/2,0,0] , size=[l_child,w_child,h_child])
        
        pyrosim.End()

        
    def Create_SnakeBrain1(self):      
        pyrosim.Start_NeuralNetwork("brain" + str(self.myID) + ".nndf")

        sensor_coins = np.random.randint(0,2, size = self.length)
        motor_coins = np.random.randint(0,2, size = self.length - 1)
        
        number_sensors = np.count_nonzero(sensor_coins)
        number_motors = np.count_nonzero(motor_coins)
        
        self.weights = np.random.rand(number_sensors,number_motors)
        self.weights = 2*self.weights - 1
                
        
        #Create snake body
    #     for linkNumber in range(1,self.length):
            
    #         new_name = str(linkNumber)
    #         new_pos = []
    #         new_l = random.uniform(0.1,4.0)
    #         new_w = random.uniform(0.1,4.0)
    #         new_h = random.uniform(0.0,4.0)
            
    #         joint_name = name + "_" + new_name
            
    #         #Send sensors/motors in snake's body
    #         for i,sensor_coin in enumerate(sensor_coins):
    #         #Possibly add sensors
    #             if sensor_coin == 1:
                    
    #                 pyrosim.Send_Sensor_Neuron(name = linkNumber-1, linkName = name)

    #             #Possibly add motors
    #                 if motor_coins[i] == 1:
    #                     pyrosim.Send_Motor_Neuron(name = name+number_sensors, jointName = joint_name)
                    
        

        
    #         #Send synapses
    #         for currentRow in range(c.numSensorNeurons):
    #             for currentColumn in range(c.numMotorNeurons):
    #                 pyrosim.Send_Synapse(sourceNeuronName = currentRow, targetNeuronName = currentColumn+c.numSensorNeurons, weight = self.weights[currentRow][currentColumn])  
            
        
        pyrosim.End()





        
    def Start_Simulation(self, directOrGUI):
        self.Create_World()
        self.Create_SnakeBody1()
        self.Create_SnakeBrain1()
        
        os.system("python3 simulate.py " + str(directOrGUI) + " " + str(self.myID) + " 2&>1 &")
        