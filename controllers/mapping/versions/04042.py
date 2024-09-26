from controller import Robot
import math
import numpy as np
import time
import WebotsNavAlgorithms as nav
import matplotlib.pyplot as plt



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

def initdsensor(robot):
    ps = []
    psNames = [
    'd0', 'd1', 'd2', 'd3',
    'd4', 'd5', 'd6', 'd7','d8','d9'
    ]

    for i in range(10):
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

def turntotarget(angle,wheels, robot,ps,close):
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
        psValues = []
        
        for i in range(10):
            psValues.append(ps[i].getValue())
        if((psValues[0] < 1000.0 or psValues[1] < 1000.0  or psValues[9] < 500.0 or psValues[6] < 1000.0 or psValues[7] < 1000.0 or psValues[8] < 500.0 )and close==False):
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
    
def mapping(wheels,ps, robot,discovery_map):
    pose=fetchgps(gps,gps_value)
    vL = MAX_SPEED_ANGULAR  
    vR = MAX_SPEED_ANGULAR
    #print(abs(objectif[0]-pose[0]))
    a=19
    turnaround=0
    while (100*(discovery_map.sum())/((MapHeight)*(MapWidth))) <= 99 :
        if a==0:
             a=20
        if a==20:
            MappingData=getOG(lidar_front,lidar_angles_front, lidar_back, lidar_angles_back, pose, occupancy_map) 
            initDisplay(robot, MappingData,pose)
            discoverymap(pose)
            print(discovery_map)
            print(100*((discovery_map.sum())/(MapHeight*MapWidth)),'%')
            print(objectif)
        a=a-1
        objectif=MappingPoint(discovery_map)
        robot.step(TIME_STEP)
        psValues = []
        angle=findangle(objectif)
        delta=pose[2]-angle
        for i in range(10):
            psValues.append(ps[i].getValue())

        right_obstacle =  psValues[0] < 1000.0 or psValues[1] < 900.0  or psValues[9] < 400.0
        left_obstacle = psValues[6] < 900.0 or psValues[7] < 1000.0 or psValues[8] < 400.0
        arounf_left =  psValues[2] < 1000.0 or psValues[9] < 1000.0
        arounf_right =  psValues[5] < 1000.0 or psValues[8] < 1000.0
        #print(psValues)
        close=(abs(objectif[0]-pose[0])<0.2 and  abs(objectif[1]-pose[1])<0.2)
        if psValues[0] < 1000.0 and 1000.0 and (close==False):
            #print("turn iii")
            for i in range(5):
                robot.step(TIME_STEP)
                move(-MAX_SPEED_ANGULAR,MAX_SPEED_ANGULAR,wheels)
                turnaround=0
        elif left_obstacle and (close==False):
            #print("turn right")
            turnaround=0
            move(MAX_SPEED_ANGULAR,-MAX_SPEED_ANGULAR,wheels)
        elif right_obstacle and (close==False):
            #print("turn left")
            turnaround=0
            move(-MAX_SPEED_ANGULAR,MAX_SPEED_ANGULAR,wheels)
        elif ((arounf_left  ) or (arounf_right )) and (close==False) and turnaround<20:
             #print("contourne")
             turnaround=turnaround+1
             move(vL, vR, wheels)
        else:
            #print("go cible")
            #print(objectif)
            turnaround=0
            turntotarget(angle,wheels,robot,ps,close)
            move(vL, vR, wheels)
        pose=fetchgps(gps,gps_value)
        
        
        


    vL = 0
    vR = 0

    move(vL, vR, wheels)
    print('Mapping finsihed. To continue close Graph.........')
    nav.exportOccupancyGridAsCsv(occupancy_map)

#############################################################################################
    
def getOG(lidar_front,lidar_angles_front, lidar_back, lidar_angles_back,pose,occupancy_map): #code aufräumen und funktionen integrieren mit dem Skalieren wie im loop gemacht. Bzw kann man doppelte werte rauslöschen?

    #Get reading values from front lidar
    lidar_distances_front=lidar_front.getRangeImage()

    #filter out inf and delete values for inf (front)
    filtered_lidar_distances_front=[]
    filtered_lidar_angles_front=[]
    empty_lidar_distances_front=[]
    empty_lidar_angles_front=[]

    for i in range(len(lidar_distances_front)):

        if lidar_distances_front[i]==np.inf:
                empty_lidar_distances_front.append(1)
                empty_lidar_angles_front.append(lidar_angles_front[i])
        elif lidar_distances_front[i]!= np.inf:
                filtered_lidar_distances_front.append(lidar_distances_front[i])
                filtered_lidar_angles_front.append(lidar_angles_front[i])
    
    #Get reading values from back lidar
    lidar_distances_back=lidar_back.getRangeImage()

    #filter out inf and delete values for inf (back)
    filtered_lidar_distances_back=[]
    filtered_lidar_angles_back=[]
    empty_lidar_distances_back=[]
    empty_lidar_angles_back=[]

    for i in range(len(lidar_distances_back)):

        if lidar_distances_back[i]==np.inf:
                empty_lidar_distances_back.append(1)
                empty_lidar_angles_back.append(lidar_angles_front[i])
        elif lidar_distances_back[i]!= np.inf:
                filtered_lidar_distances_back.append(lidar_distances_back[i])
                filtered_lidar_angles_back.append(lidar_angles_back[i])



    MappingData=[[],[],[],[]] #[[x-values], [y-values]]

    #process information    
    angle= pose[2]*np.pi/(180)

    for x in range(len(filtered_lidar_angles_front)):
        #Front readings
        MappingData[0].append(((pose[0]+0.45*np.cos(angle)+filtered_lidar_distances_front[x]*np.cos(filtered_lidar_angles_front[x]+angle)))) #shows only the values that are read in the moment #the pose of the robot is included in this vector. Why?
        MappingData[1].append((pose[1]+0.45*np.sin(angle)+filtered_lidar_distances_front[x]*np.sin(filtered_lidar_angles_front[x]+angle))) #shows only the values that are read in the moment #the pose of the robot is included in this vector. Why?

        MappingData[0]=[ round(elem, 6) for elem in MappingData[0] ]
        MappingData[1]=[ round(elem, 6) for elem in MappingData[1] ]

    angle= -pose[2]*np.pi/(180)

    for y in range(len(filtered_lidar_angles_back)):
        #Back readings
        MappingData[2].append(((pose[0]-0.45*np.cos(angle)-filtered_lidar_distances_back[y]*np.cos(filtered_lidar_angles_back[y]+angle)))) #shows only the values that are read in the moment #the pose of the robot is included in this vector. Why?
        MappingData[3].append((pose[1]+0.45*np.sin(angle)+filtered_lidar_distances_back[y]*np.sin(filtered_lidar_angles_back[y]+angle))) #shows only the values that are read in the moment #the pose of the robot is included in this vector. Why?

        MappingData[2]=[ round(elem, 6) for elem in MappingData[2] ]
        MappingData[3]=[ round(elem, 6) for elem in MappingData[3] ]

    MappingData=[MappingData[0]+ MappingData[2], MappingData[1] + MappingData[3]]

        #Deletes Dub
    MappingData=np.array(MappingData)
    MappingData=np.round(MappingData,6)
    MappingData=MappingData.tolist()

    for i in range(len(MappingData[0])):
        
        # Convert robot coordinates to map coordinates (assuming robot at origin)
        map_x = int(MappingData[0][i]/cellSize)
        map_y = int((MapHeight/cellSize)-(MappingData[1][i]/cellSize))

        # Check if within map boundaries
        if 0 <= map_x < occupancy_map.shape[1] and 0 <= map_y < occupancy_map.shape[0]:
                #Mark cell as occupied
                occupancy_map[map_y, map_x] = 1

    
    #Define a variable for the Transformation
    emptyMappingData=[[],[],[],[]]   #[[x-values-front], [y-values-front], [x-values-back], [y-values-back]]

    #process information and get occupied points in World coordinates

    for x in range(len(empty_lidar_angles_front)):

        #Front readings
        emptyMappingData[0].append((((pose[0]+0.45*np.cos(np.radians(pose[2])))+empty_lidar_distances_front[x]*np.cos(empty_lidar_angles_front[x]+np.radians(pose[2]))))) #shows only the values that are read in the moment #the pose of the robot is included in this vector.
        emptyMappingData[1].append(((pose[1]+0.45*np.sin(np.radians(pose[2])))+empty_lidar_distances_front[x]*np.sin(empty_lidar_angles_front[x]+np.radians(pose[2])))) #shows only the values that are read in the moment #the pose of the robot is included in this vector.

        emptyMappingData[0]=[ round(elem, 6) for elem in emptyMappingData[0] ]
        emptyMappingData[1]=[ round(elem, 6) for elem in emptyMappingData[1] ]

    for y in range(len(empty_lidar_angles_back)):
        #Back readings
        emptyMappingData[2].append(((((pose[0]-0.45*np.cos(np.radians(-pose[2]))))-empty_lidar_distances_back[y]*np.cos(empty_lidar_angles_back[y]+np.radians(-pose[2]))))) #shows only the values that are read in the moment #the pose of the robot is included in this vector.
        emptyMappingData[3].append(((pose[1]+0.45*np.sin(np.radians(-pose[2])))+empty_lidar_distances_back[y]*np.sin(empty_lidar_angles_back[y]+np.radians(-pose[2])))) #shows only the values that are read in the moment #the pose of the robot is included in this vector.


        emptyMappingData[2]=[ round(elem, 6) for elem in emptyMappingData[2] ]
        emptyMappingData[3]=[ round(elem, 6) for elem in emptyMappingData[3] ]

    #Combine front and Back readings so Mapping Data [[x-values], [y-values]]
    emptyMappingData=[emptyMappingData[0]+ emptyMappingData[2], emptyMappingData[1] + emptyMappingData[3]]

    #Round the Values
    emptyMappingData=np.array(emptyMappingData)
    emptyMappingData=np.round(emptyMappingData,6)
    emptyMappingData=emptyMappingData.tolist()




    #lowres OC=discovery map
    for i in range(len(MappingData[0])):
        map_x= int(MappingData[0][i])
        map_y= int(MapHeight-MappingData[1][i])
        
        if 0 <= map_x < discovery_map.shape[1] and 0 <= map_y < discovery_map.shape[0]:
                #Mark cell as occupied
                discovery_map[map_y, map_x] = 1

    for i in range(len(emptyMappingData[0])):
        map_x= int(emptyMappingData[0][i])
        map_y= int(MapHeight-emptyMappingData[1][i])
        
        if 0 <= map_x < discovery_map.shape[1] and 0 <= map_y < discovery_map.shape[0]:
                #Mark cell as occupied
                discovery_map[map_y, map_x] = 1    



    return MappingData         


def discoverymap(pose):
        #update discovery map
        pose=np.array(pose)
        pose=np.round(pose,0)
        pose=pose.astype(int)
        if ((MapHeight-pose[1])<len(discovery_map) and (pose[0]-1)<len(discovery_map[0])):
            discovery_map[MapHeight-pose[1]][pose[0]-1] = 1
    


def MappingPoint(discovery_map):
    closest_distance = 10000
    MappingPointX, MappingPointY = 0, 0
    pose=fetchgps(gps,gps_value)

    target_x = round(pose[0],0)
    target_y = round(pose[1],0)

    
    for y in range(len(discovery_map)):
        for x in range(len(discovery_map[y])):

            if  discovery_map[y][x] == 0:
                distance = (x + 1 - target_x)* (x + 1 - target_x)+ (len(discovery_map) - y  - target_y)*(len(discovery_map) - y  - target_y)

                if distance < closest_distance:
                    closest_distance = distance
                    MappingPointX = x + 1
                    MappingPointY = len(discovery_map) - y  
    
    return MappingPointX, MappingPointY

#############################################################################################

    
if __name__ == "__main__":
    # create the Robot instance.
    robot = Robot()    
    wheels = initWheels(robot)  # Initialize wheels here
    gps, gps_value = initGps(TIME_STEP, robot)
    lidar_front, lidar_angles_front, lidar_back, lidar_angles_back=initLidar(TIME_STEP, robot)
    psValues=initdsensor(robot)
    MapHeight=8        #needs to have the same size as the world file
    MapWidth=8         #needs to have the same size as the world file
    cellSize=0.1
    occupancy_map = np.zeros((int(MapHeight/cellSize), int(MapWidth/cellSize)))
    lowres_occupancy_map=np.zeros((MapHeight,MapWidth))
    discovery_map=np.zeros((MapHeight,MapWidth))

    objectif =[7,1]

    a=19
    while robot.step(TIME_STEP) != -1:
        mapping(wheels,psValues,robot,discovery_map)
        break

        
        

        