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
    



def initdsensor(robot):
    ps = []
    psNames = [
    'd0', 'd1', 'd2', 'd3',
    'd4', 'd5', 'd6', 'd7','d8','d9','d10'
    ]

    for i in range(11):
        ps.append(robot.getDevice(psNames[i]))
        ps[i].enable(TIME_STEP)
    return ps
     

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

def turntotarget(angle,wheels, robot,ps,objectif):
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

      
    close=(abs(objectif[0]-pose[0])<0.5 and  abs(objectif[1]-pose[1])<0.5)

    while abs(diff_angle)>2:
        robot.step(TIME_STEP)
        psValues = []
        
        for i in range(11):
            psValues.append(ps[i].getValue())
        right_obstacle =  psValues[0] < 700.0 or psValues[1] < 700.0  or psValues[9] < 500.0
        left_obstacle = psValues[6] < 700.0 or psValues[7] < 700.0 or psValues[8] < 500.0
        if((left_obstacle or right_obstacle) and close==False):
             break
        move(vL, vR, wheels)
        pose=fetchgps(gps,gps_value)
        diff_angle_1 = angle_difference(pose[2], angle)
        diff_angle_2 = angle_difference(pose[2] + 360, angle)
        diff_angle_3 = angle_difference(pose[2] - 360, angle)

        diff_angle = min(diff_angle_1, diff_angle_2, diff_angle_3, key=abs)

    vL = 0
    vR = 0

    move(vL, vR, wheels)


    
def navigation(wheels,ps,robot,objectif,map):
    pose=fetchgps(gps,gps_value)
    list_point,solution=ast.aStar(pose[0],pose[1],objectif[0],objectif[1],map)
    print(list_point)
    vL = MAX_SPEED_ANGULAR  
    vR = MAX_SPEED_ANGULAR
    tol=0.75
    turnaround=0

    if solution:
        for p in list_point:
            print(p)
            if(p==list_point[-1]):
                tol=0.05
            while (abs(p[0]-pose[0])>tol)or(abs(p[1]-pose[1])>tol) :
                robot.step(TIME_STEP)
                angle=findangle(p)
                psValues = []
                for i in range(11):
                    psValues.append(ps[i].getValue())

                right_obstacle =  psValues[0] < 700.0 or psValues[1] < 700.0  or psValues[9] < 500.0
                left_obstacle = psValues[6] < 700.0 or psValues[7] < 700.0 or psValues[8] < 500.0
                arounf_left =  psValues[2] < 700.0 and right_obstacle==False
                arounf_right =  psValues[5] < 700.0 and left_obstacle==False
                print(psValues)
                print(p)
                close=(abs(objectif[0]-pose[0])<0.5 and  abs(objectif[1]-pose[1])<0.5)

                if left_obstacle and close==False :
                    print("turn right")
                    for i in range(7):
                        robot.step(TIME_STEP)
                        move(MAX_SPEED_ANGULAR,-MAX_SPEED_ANGULAR,wheels)
                    vL = MAX_SPEED_ANGULAR  
                    vR = -MAX_SPEED_ANGULAR
                    turnaround=0
                elif right_obstacle and close==False:
                    print("turn left")
                    for i in range(7):
                        robot.step(TIME_STEP)
                        move(-MAX_SPEED_ANGULAR,MAX_SPEED_ANGULAR,wheels)
                    vL = -MAX_SPEED_ANGULAR  
                    vR = MAX_SPEED_ANGULAR
                    turnaround=0
                elif ((arounf_left  ) or (arounf_right )) and (close==False) and turnaround<20 :
                    print("turn around")
                    turnaround=turnaround+1
                    vL = MAX_SPEED_ANGULAR  
                    vR = MAX_SPEED_ANGULAR
                else:
                    print("gogo")
                    turntotarget(angle,wheels,robot,ps,objectif)
                    turnaround=0



                
                #print(p)
                #print(pose)
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
    ps=initdsensor(robot)
    goal=[5.75,4.25]
    map=nav.readOccupancyGrid("./SampleWorld.csv")
    nav.printOccupancyGrid(map)
    map=nav.addBufferToOccupancyGrid(map,3)
    nav.printOccupancyGrid(map)
    while robot.step(TIME_STEP) != -1:
        navigation (wheels,ps,robot,goal,map)
        break

        
        

        