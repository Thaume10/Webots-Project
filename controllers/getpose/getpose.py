from controller import Robot
import math
import numpy as np
import time
import matplotlib.pyplot as plt
import os




# Global Variables for Robot Parameters
WHEEL_RADIUS = 0.15  # needs to be picked from wheel model
MAX_SPEED_ANGULAR = 2 # angular Velocity
MAX_SPEED_LINEAR = WHEEL_RADIUS * MAX_SPEED_ANGULAR  # linear velocity
DISTANCE_BETWEEN_WHEELS = 0.1
TIME_STEP = 64


def initGps(TIME_STEP, robot):
    gps = []
    gpsNames = ['gps1', 'gps2']
    value = [[0, 0, 0], [0, 0, 0]]

    for i in range(len(gpsNames)):
        gps.append(robot.getDevice(gpsNames[i]))
        gps[i].enable(TIME_STEP)
    return gps, value
    



#############################################################################################

def getGpsData(gps, value):

    # Put solution to Q2b here
    for i in range(len(gps)):
        value[i]=gps[i].getValues()
        
    #◘print("Getting GPS Data")
    return value


def getRobotPose(gps_value):
    pose = [0,0,-90]
    
    
    pose[2] = pose[2]+np.arctan2(gps_value[0][1] - gps_value[1][1], gps_value[0][0] - gps_value[1][0]) * (180/np.pi)
    pose[2] = ((pose[2] + 180) % 360) - 180
    angle= pose[2]*np.pi/(180)
    pose[1] =((gps_value[1][1])+0.2*np.cos(angle))
    pose[0] =((gps_value[1][0])-0.2*np.sin(angle))
    return pose
    
def fetchgps(gps,gps_value):
    gps_value = getGpsData(gps, gps_value)
    robot_pose = getRobotPose(gps_value)
    #print(gps_value)
    #print(robot_pose)
    return robot_pose
    
#############################################################################################





#############################################################################################



#############################################################################################

    
if __name__ == "__main__":
    # create the Robot instance.
    robot = Robot()    
    gps, gps_value = initGps(TIME_STEP, robot)


    while robot.step(TIME_STEP) != -1:
        gpsv=fetchgps(gps,gps_value)
        print(gpsv)
        
        

        