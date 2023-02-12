#Imports
import pybullet as p
import time
import pybullet_data
import pyrosim.pyrosim as pyrosim
#import numpy as np
#import random
import constants as c
from world import WORLD
from robot import ROBOT
from sensor import SENSOR
from motor import MOTOR

class SIMULATION:
    def __init__(self, directOrGUI, solutionID):
            self.directOrGUI = directOrGUI
        
            if self.directOrGUI == "DIRECT":
                p.connect(p.DIRECT)

            else:
                p.connect(p.GUI)
                
            self.world = WORLD()
            self.robot = ROBOT(solutionID)

    def Run(self):
        for i in range(c.iterations):
            p.stepSimulation()
            
            self.robot.Sense(i)
            self.robot.Think()
            self.robot.Act(i)
            
            if self.directOrGUI == "GUI":
                time.sleep(c.sleep)
                        
    def __del__(self):
        #SENSOR.Save_Values()
        #MOTOR.Save_Values()
        p.disconnect()
        
    def Get_Fitness(self):
        self.robot.Get_Fitness()
        