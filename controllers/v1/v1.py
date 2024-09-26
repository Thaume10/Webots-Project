from controller import Robot
import math
import numpy as np
import time

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
    
def initDisplay(robot,OG,pose): #some few changes
    
        disp=robot.getDevice('map')
        disp.setColor(0xFF0000)
        MiddleOriginX=200/2#(disp.getWidth)/2
        MiddleOriginY=200/2#(disp.getHeight)/2
        #print("teste")
        OG = np.array(OG)/0.1
        OG = OG.tolist()

        for i in range(len(OG[1])):
                disp.drawPixel(OG[0][i]+MiddleOriginX,(-1)*OG[1][i]+MiddleOriginY)
                
def initLidar(timestep,robot):
    
        lidar_front=robot.getDevice('lidar1')
        lidar_front.enable(timestep)
        lidar_front.enablePointCloud()
        
        #lidar information front
        horizontalResolution_front=lidar_front.getHorizontalResolution()
        fieldOfView_front=lidar_front.getFov()
        #print(fieldOfView_front)
        angle_value_front=np.divide(fieldOfView_front, horizontalResolution_front-1)
        
        #obtain angle vector front
        lidar_angles_front=[0]
        increaseangle=0
        positiveangles=[]
        negativeangles=[]
        for i in range(int((horizontalResolution_front-1)/2)):
                increaseangle=increaseangle+angle_value_front
                positiveangles.append(increaseangle)
        positiveangles=np.flip(positiveangles)
        positiveangles=positiveangles.tolist()
        #positiveangles = [ round(elem, 2) for elem in positiveangles ]   #round elements
        #print(positiveangles)

        negativeangles=-1*np.array(positiveangles)
        negativeangles=np.flip(negativeangles)
        negativeangles=negativeangles.tolist()
        #print(negativeangles)
        lidar_angles_front=positiveangles+lidar_angles_front+negativeangles
        
        #check values
        #print(angle_value)
        #print(lidar_angles)
        #print(len(lidar_angles))
        
        lidar_back = robot.getDevice('lidar2')
        lidar_back.enable(timestep)
        lidar_back.enablePointCloud()
        
        #lidar information front
        horizontalResolution_back = lidar_back.getHorizontalResolution()
        fieldOfView_back = lidar_back.getFov()
        #print(fieldOfView_back)
        angle_value_back = np.divide(fieldOfView_back, horizontalResolution_back-1)
        
        #obtain angle vector front
        lidar_angles_back=[0]
        increaseangle=0
        positiveangles=[]
        negativeangles=[]
        for i in range(int((horizontalResolution_back-1)/2)):
                increaseangle=increaseangle+angle_value_back
                positiveangles.append(increaseangle)
        positiveangles=np.flip(positiveangles)
        positiveangles=positiveangles.tolist()
        #positiveangles = [ round(elem, 2) for elem in positiveangles ]   #round elements
        #print(positiveangles)

        negativeangles=-1*np.array(positiveangles)
        negativeangles=np.flip(negativeangles)
        negativeangles=negativeangles.tolist()
        #print(negativeangles)
        lidar_angles_back=positiveangles+lidar_angles_back+negativeangles

        return lidar_front, lidar_angles_front, lidar_back, lidar_angles_back



def move(velocityL, velocityR, wheels):
    wheels[0].setVelocity(velocityL)
    wheels[1].setVelocity(velocityR)

#############################################################################################


def distanceForward(distance, wheels, robot):
    t_current = robot.getTime()  # starting time of loop
    t_duration = distance / MAX_SPEED_LINEAR
    t_end=t_current+t_duration
    vL = MAX_SPEED_ANGULAR
    vR = MAX_SPEED_ANGULAR
    
    while t_current < t_end:
        #print(t_current)
        robot.step(TIME_STEP)
        move(vL, vR, wheels)
        fetchgps(gps,gps_value)
        t_current = robot.getTime()  # Update current time

    vL = 0
    vR = 0

    move(vL, vR, wheels)
    
def stop(wheels, robot):
    vL = 0
    vR = 0
    for i in range(10):
        robot.step(TIME_STEP)
        print("stop")
        move(vL, vR, wheels)



def angleturnleft(angleR, wheels, robot):

    t_current = robot.getTime()  # starting time of rotation
    t_90=2*np.pi*DISTANCE_BETWEEN_WHEELS/MAX_SPEED_ANGULAR
    t_rotation=t_90*angleR/90
    t_end = t_current + t_rotation
    
    # Now set the angular velocities of the wheels
    vL = -MAX_SPEED_ANGULAR  # negative angular velocity for left wheel
    vR = MAX_SPEED_ANGULAR  

    while t_current < t_end:
        robot.step(TIME_STEP)
        move(vL, vR, wheels)
        fetchgps(gps,gps_value)
        t_current = robot.getTime()  # Update current time

    vL = 0
    vR = 0

    move(vL, vR, wheels)

def angleturnright(angleR, wheels, robot):

    t_current = robot.getTime()  # starting time of rotation
    t_90=2*np.pi*DISTANCE_BETWEEN_WHEELS/MAX_SPEED_ANGULAR
    t_rotation=t_90*angleR/90
    t_end = t_current + t_rotation
    
    # Now set the angular velocities of the wheels
    vL = MAX_SPEED_ANGULAR  # negative angular velocity for left wheel
    vR = -MAX_SPEED_ANGULAR  

    while t_current < t_end:
        robot.step(TIME_STEP)
        move(vL, vR, wheels)
        fetchgps(gps,gps_value)
        t_current = robot.getTime()  # Update current time

    vL = 0
    vR = 0

    move(vL, vR, wheels)

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

def findangle(objectif):
    robot_pose=fetchgps(gps,gps_value)
    dx=objectif[0]-robot_pose[0]
    dy=objectif[1]-robot_pose[1]
    angle=math.atan2(dy,dx)
    angle_degres = math.degrees(angle)

    #print(angle_degres)
    return angle_degres
    
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

      

    while abs(diff_angle)>1.5:
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
    
def movetotarget(objectif,wheels, robot):
    pose=fetchgps(gps,gps_value)

    vL = MAX_SPEED_ANGULAR  
    vR = MAX_SPEED_ANGULAR
    #print(abs(objectif[0]-pose[0]))
    while (abs(objectif[0]-pose[0])>0.05)or(abs(objectif[1]-pose[1])>0.05) :
        robot.step(TIME_STEP)
        angle=findangle(objectif)
        turntotarget(angle,wheels,robot)
        
        move(vL, vR, wheels)
        pose=fetchgps(gps,gps_value)


    vL = 0
    vR = 0

    move(vL, vR, wheels)

#############################################################################################
    
def getOG(lidar_front,lidar_angles_front, lidar_back, lidar_angles_back,pose): #code aufräumen und funktionen integrieren mit dem Skalieren wie im loop gemacht. Bzw kann man doppelte werte rauslöschen?

    lidar_distances_front=lidar_front.getRangeImage()
    #print(lidar_distances_front)

    #filter inf front
    filtered_lidar_distances_front=[]
    filtered_lidar_angles_front=[]
    for i in range(len(lidar_distances_front)):

        if lidar_distances_front[i]==np.inf:
                pass
        elif lidar_distances_front[i]!= np.inf:
                filtered_lidar_distances_front.append(lidar_distances_front[i])
                filtered_lidar_angles_front.append(lidar_angles_front[i])
    
    lidar_distances_back=lidar_back.getRangeImage()

    #filter inf front
    filtered_lidar_distances_back=[]
    filtered_lidar_angles_back=[]
    for i in range(len(lidar_distances_back)):

        if lidar_distances_back[i]==np.inf:
                pass
        elif lidar_distances_back[i]!= np.inf:
                filtered_lidar_distances_back.append(lidar_distances_back[i])
                filtered_lidar_angles_back.append(lidar_angles_back[i])



    MappingData=[[],[],[],[]] #[[x-values], [y-values]]
    cellSize=1

    #process information    
    angle= pose[2]*np.pi/(180)

    for x in range(len(filtered_lidar_angles_front)):
        #Front readings
        MappingData[0].append(((pose[0]+0.55*np.cos(angle)+filtered_lidar_distances_front[x]*np.cos(filtered_lidar_angles_front[x]+angle))/cellSize)) #shows only the values that are read in the moment #the pose of the robot is included in this vector. Why?
        MappingData[1].append((pose[1]+0.55*np.sin(angle)+filtered_lidar_distances_front[x]*np.sin(filtered_lidar_angles_front[x]+angle))/cellSize) #shows only the values that are read in the moment #the pose of the robot is included in this vector. Why?

        MappingData[0]=[ round(elem, 6) for elem in MappingData[0] ]
        MappingData[1]=[ round(elem, 6) for elem in MappingData[1] ]

    angle= -pose[2]*np.pi/(180)

    for y in range(len(filtered_lidar_angles_back)):
        #Back readings
        MappingData[2].append(((pose[0]-0.45*np.cos(angle)-filtered_lidar_distances_back[y]*np.cos(filtered_lidar_angles_back[y]+angle))/cellSize)) #shows only the values that are read in the moment #the pose of the robot is included in this vector. Why?
        MappingData[3].append((pose[1]+0.45*np.sin(angle)+filtered_lidar_distances_back[y]*np.sin(filtered_lidar_angles_back[y]+angle))/cellSize) #shows only the values that are read in the moment #the pose of the robot is included in this vector. Why?

        MappingData[2]=[ round(elem, 6) for elem in MappingData[2] ]
        MappingData[3]=[ round(elem, 6) for elem in MappingData[3] ]

    MappingData=[MappingData[0]+ MappingData[2], MappingData[1] + MappingData[3]]

        #Deletes Dub
    MappingData=np.array(MappingData)
    MappingData=np.round(MappingData,1)
    MappingData=MappingData.tolist()


    return MappingData                 

#############################################################################################

    
if __name__ == "__main__":
    # create the Robot instance.
    robot = Robot()
    # get the time step of the current world.
    objectif =[2,4.5]
    
    wheels = initWheels(robot)  # Initialize wheels here
    gps, gps_value = initGps(TIME_STEP, robot)
    lidar_front, lidar_angles_front, lidar_back, lidar_angles_back=initLidar(TIME_STEP, robot)
    #print(len(lidar_angles_front))
    OGGLOBAL=[[],[]]
    a=10
    while robot.step(TIME_STEP) != -1:
        if a==0:
            a=10
        #movetotarget(objectif,wheels,robot)
        #break
        if a==10:
            pose=fetchgps(gps,gps_value)
            OG=getOG(lidar_front,lidar_angles_front, lidar_back, lidar_angles_back, pose) #austauschen OG(lidar)
            initDisplay(robot, OG, pose)
            set_coords_1 = {tuple(c) for c in zip(OGGLOBAL[0], OGGLOBAL[1])}
            set_coords_2 = {tuple(c) for c in zip(OG[0], OG[1])}
            merged_coords = set_coords_1 | set_coords_2
            OGGLOBAL = [[x for x, _ in merged_coords], [y for _, y in merged_coords]]

        a=a-1
        
        move(-2,2,wheels)
        #time.sleep(2)
        
    #

    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    #while robot.step(TIME_STEP) != -1:
        #distanceForward(1, wheels, robot)  # Pass wheels here
        #stop(wheels, robot)
        #printgps(gps,gps_value)
        #angleturnleft(90, wheels, robot)  # Pass wheels here
        #stop(wheels, robot)
        