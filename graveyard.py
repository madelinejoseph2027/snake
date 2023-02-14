#Imports
import numpy as np
import pybullet as p
import pyrosim.pyrosim as pyrosim
import constants as c
import random
import os
import time


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