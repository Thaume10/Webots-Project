from controller import Robot
import math
import numpy as np
import time
import WebotsNavAlgorithms as nav
import matplotlib.pyplot as plt
import os
import AStar as ast




# Global Variables for Robot Parameters
WHEEL_RADIUS = 0.15  # needs to be picked from wheel model
MAX_SPEED_ANGULAR = 2 # angular Velocity
MAX_SPEED_LINEAR = WHEEL_RADIUS * MAX_SPEED_ANGULAR  # linear velocity
DISTANCE_BETWEEN_WHEELS = 0.1
TIME_STEP = 64

def initWheels(robot):
    wheels = []
    wheelsNames = ['wheel1', 'wheel2']
    for i in range(len(wheelsNames)):
        wheels.append(robot.getDevice(wheelsNames[i]))
        wheels[i].setPosition(float('inf'))
        wheels[i].setVelocity(0.0)
    return wheels

def initGps(TIME_STEP, robot):
    gps = []
    gpsNames = ['gps1', 'gps2']
    value = [[0, 0, 0], [0, 0, 0]]

    for i in range(len(gpsNames)):
        gps.append(robot.getDevice(gpsNames[i]))
        gps[i].enable(TIME_STEP)
    return gps, value
    

def move(velocityL, velocityR, wheels):
    wheels[0].setVelocity(velocityL)
    wheels[1].setVelocity(velocityR)

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

def findangle(objectif):
    robot_pose=fetchgps(gps,gps_value)
    return np.round(np.arctan2(objectif[1]-robot_pose[1],objectif[0]-robot_pose[0])*(180/np.pi), 2)
    
def angle_difference(a1, a2):
    return ((a1 - a2) % 360 + 540) % 360 - 180

def turntotarget(angle,wheels, robot):
    pose=fetchgps(gps,gps_value)
    #print(angle)
    diff_angle_1 = angle_difference(pose[2], angle)
    diff_angle_2 = angle_difference(pose[2] + 360, angle)
    diff_angle_3 = angle_difference(pose[2] - 360, angle)

    # Choisir la différence d'angle la plus petite en valeur absolue
    diff_angle = min(diff_angle_1, diff_angle_2, diff_angle_3, key=abs)

    if diff_angle > 0:
        # Tourner dans le sens horaire
        vL = MAX_SPEED_ANGULAR  
        vR = -MAX_SPEED_ANGULAR
    else:
        vL = -MAX_SPEED_ANGULAR 
        vR = MAX_SPEED_ANGULAR

      

    while abs(diff_angle)>2:
        robot.step(TIME_STEP)        
        move(vL, vR, wheels)
        pose=fetchgps(gps,gps_value)
        diff_angle_1 = angle_difference(pose[2], angle)
        diff_angle_2 = angle_difference(pose[2] + 360, angle)
        diff_angle_3 = angle_difference(pose[2] - 360, angle)

        diff_angle = min(diff_angle_1, diff_angle_2, diff_angle_3, key=abs)

    vL = 0
    vR = 0

    move(vL, vR, wheels)


    
def navigation(wheels,robot,objectif,map):
    pose=fetchgps(gps,gps_value)
    list_point,solution=ast.aStar(pose[0],pose[1],objectif[0],objectif[1],map)
    #print(list_point)
    vL = MAX_SPEED_ANGULAR  
    vR = MAX_SPEED_ANGULAR
    tol=0.2
    if solution:
        for p in list_point:
            print(p)
            if(p==list_point[-1]):
                tol=0.02
            while (abs(p[0]-pose[0])>tol)or(abs(p[1]-pose[1])>tol) :
                robot.step(TIME_STEP)
                angle=findangle(p)
                turntotarget(angle,wheels,robot)
                move(vL, vR, wheels)
                pose=fetchgps(gps,gps_value)
        vL = 0
        vR = 0
        move(vL, vR, wheels)



#############################################################################################



#############################################################################################

    
if __name__ == "__main__":
    # create the Robot instance.
    robot = Robot()    
    wheels = initWheels(robot)  # Initialize wheels here
    gps, gps_value = initGps(TIME_STEP, robot)
    goal=[7.25,0.5]

    map=nav.readOccupancyGrid("./SampleWorld.csv")
    map=nav.addBufferToOccupancyGrid(map,4)
    nav.printOccupancyGrid(map)
    while robot.step(TIME_STEP) != -1:
        navigation (wheels,robot,goal,map)
        break

        
        

        