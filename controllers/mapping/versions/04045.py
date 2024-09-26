from controller import Robot
import math
import numpy as np
import time
import WebotsNavAlgorithms as nav
import matplotlib.pyplot as plt
import os


def exportOccupancyGridAsCsvSafely(occupancy_grid, filename="occupancy_grid.csv"):
    """! Takes an existing occupancy grid and exports the information within it as a csv file.
    If a file with the same name already exists, it will be deleted before exporting.
    @param occupancy_grid **list**: A 2D occupancy grid in the form of a 2D python list, i.e. [[1, 1, 1, 1], [1, 0, 0, 1], [1, 0, 0, 1], [1, 1, 1, 1]]
    @param filename **str**: The name of the file to export the occupancy grid to
    @returns **NULL**: This function doesn't return anything
    """
    if os.path.isfile(filename):
        os.remove(filename)

    with open(filename, "w") as f:
        for row in occupancy_grid:
            for column in row:
                f.write(str(column))
                f.write(",")
            f.write("\n")



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

def calculate_percentage_not_equal_to_minus_one(array_2d):
    not_equal_to_minus_one = np.count_nonzero(array_2d != -1)
    total_elements = array_2d.size
    percentage = (not_equal_to_minus_one / total_elements) * 100
    return percentage
    
def mapping(wheels,ps, robot,occupancy_map):
    pose=fetchgps(gps,gps_value)
    vL = MAX_SPEED_ANGULAR  
    vR = MAX_SPEED_ANGULAR
    #print(abs(objectif[0]-pose[0]))
    a=20
    turnaround=0
    X=int(pose[0] * 10)
    Y=int(len(occupancy_map) - pose[1] * 10)
    objectif=[4,4]
    while calculate_percentage_not_equal_to_minus_one(occupancy_map) <100 :
        if a==0:
             a=20
        if a==20:
            MappingData=getOG(lidar_front,lidar_angles_front, lidar_back, lidar_angles_back, pose, occupancy_map) 
            initDisplay(robot, MappingData,pose)
            discoverymap(pose)
            print(occupancy_map)
            print(calculate_percentage_not_equal_to_minus_one(occupancy_map),'%')
            if(occupancy_map[Y][X]!=-1):
                objectif[0],objectif[1],X,Y=MappingPoint(occupancy_map)
            print(objectif)
        a=a-1
        #objectif=MappingPoint(occupancy_map)
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
        print(psValues)
        close=(abs(objectif[0]-pose[0])<0.2 and  abs(objectif[1]-pose[1])<0.2)
        if psValues[0] < 1000.0 and 1000.0 and (close==False):
            print("turn iii")
            for i in range(20):
                robot.step(TIME_STEP)
                move(-MAX_SPEED_ANGULAR,MAX_SPEED_ANGULAR,wheels)
                turnaround=0
        elif left_obstacle and (close==False):
            print("turn right")
            turnaround=0
            move(MAX_SPEED_ANGULAR,-MAX_SPEED_ANGULAR,wheels)
        elif right_obstacle and (close==False):
            print("turn left")
            turnaround=0
            move(-MAX_SPEED_ANGULAR,MAX_SPEED_ANGULAR,wheels)
        elif ((arounf_left  ) or (arounf_right )) and (close==False) and turnaround<20:
             print("contourne")
             turnaround=turnaround+1
             move(vL, vR, wheels)
        else:
            print("go cible")
            print(objectif)
            turnaround=0
            turntotarget(angle,wheels,robot,ps,close)
            move(vL, vR, wheels)
        pose=fetchgps(gps,gps_value)
        
        
        


    vL = 0
    vR = 0

    move(vL, vR, wheels)
    print('Mapping finsihed. To continue close Graph.........')
    exportOccupancyGridAsCsvSafely(occupancy_map)

#############################################################################################

def bresenham_with_threshold(x0, y0, x1, y1, threshold=0.5):
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)

    sx = -1 if x0 > x1 else 1
    sy = -1 if y0 > y1 else 1

    err = dx - dy

    points = []

    while True:
        # Vérifier si le point est suffisamment proche de la ligne idéale
        if x0!=x1 and abs(y1 - y0 - (x1 - x0) * (y0 - y1) / (x0 - x1)) <= threshold:
            points.append((x0, y0))

        if x0 == x1 and y0 == y1:
            break

        e2 = 2 * err

        if e2 > -dy:
            err -= dy
            x0 += sx

        if e2 < dx:
            err += dx
            y0 += sy

    return points


    
def getOG(lidar_front,lidar_angles_front, lidar_back, lidar_angles_back,pose,occupancy_map): #code aufräumen und funktionen integrieren mit dem Skalieren wie im loop gemacht. Bzw kann man doppelte werte rauslöschen?

    #Get reading values from front lidar
    lidar_distances_front=lidar_front.getRangeImage()

    #filter out inf and delete values for inf (front)
    filtered_lidar_distances_front=[]
    filtered_lidar_angles_front=[]


    for i in range(len(lidar_distances_front)):

        if lidar_distances_front[i]!= np.inf:
                filtered_lidar_distances_front.append(lidar_distances_front[i])
                filtered_lidar_angles_front.append(lidar_angles_front[i])
    
    #Get reading values from back lidar
    lidar_distances_back=lidar_back.getRangeImage()

    #filter out inf and delete values for inf (back)
    filtered_lidar_distances_back=[]
    filtered_lidar_angles_back=[]


    for i in range(len(lidar_distances_back)):
        if lidar_distances_back[i]!= np.inf:
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

    center_x = int(pose[0] * 10)
    center_y = int(len(occupancy_map) - pose[1] * 10)
    for i in range(len(MappingData[0])):
        
        # Convert robot coordinates to map coordinates (assuming robot at origin)
        map_x = int(MappingData[0][i]/cellSize)
        map_y = int((MapHeight/cellSize)-(MappingData[1][i]/cellSize))

        # Check if within map boundaries
        if 0 <= map_x < occupancy_map.shape[1] and 0 <= map_y < occupancy_map.shape[0]:
                line_points = bresenham_with_threshold(int(center_x), int(center_y), map_x, map_y)
                for point in line_points:
                    if 0 <= point[0] < occupancy_map.shape[1] and 0 <= point[1] < occupancy_map.shape[0] and occupancy_map[point[1], point[0]] ==-1  :
                        occupancy_map[point[1], point[0]] = 0
                #Mark cell as occupied
                occupancy_map[map_y, map_x] = 1
    return MappingData         



def discoverymap(pose):
    center_x = int(pose[0] * 10)
    center_y = int(len(occupancy_map) - pose[1] * 10)

    # Half side length of the square
    half_side_length = 7  # Half of 15, since the square is 15x15

    # Iterate over the square area
    for y in range(max(0, center_y - half_side_length), min(len(occupancy_map), center_y + half_side_length + 1)):
        for x in range(max(0, center_x - half_side_length), min(len(occupancy_map[0]), center_x + half_side_length + 1)):
            if occupancy_map[y][x] == -1:
                 occupancy_map[y][x] = 0

    


def MappingPoint(occupancy_map):
    closest_distance = 10000
    MappingPointX, MappingPointY,xfin,yfin = 0, 0,0,0
    pose=fetchgps(gps,gps_value)

    target_x = round(pose[0],2)
    target_y = round(pose[1],2)
    

    
    for y in range(len(occupancy_map)):
        for x in range(len(occupancy_map[y])):

            if  occupancy_map[y][x] == -1:
                distance = math.sqrt(((x+1)*0.1  - target_x) ** 2 + ( len(occupancy_map)*cellSize - y*0.1 - target_y) ** 2)

                if distance < closest_distance:
                    closest_distance = distance
                    MappingPointX = (x+1)*0.1 
                    MappingPointY = len(occupancy_map)*cellSize - y*0.1
                    xfin=x
                    yfin=y
    
    return MappingPointX, MappingPointY,xfin,yfin

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
    occupancy_map = np.full((int(MapHeight/cellSize), int(MapWidth/cellSize)), -1)



    while robot.step(TIME_STEP) != -1:
        mapping(wheels,psValues,robot,occupancy_map)
        break

        
        

        