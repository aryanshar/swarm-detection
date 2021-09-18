 
import airsim
import os
import sys
import pprint
import tempfile
import math
import time
import argparse
import numpy as np
import matplotlib.pyplot as pyplot
from mpl_toolkits import mplot3d
import mpl_toolkits.mplot3d.art3d as art3d
from scipy.spatial.transform import Rotation
import copy
#import qpsolvers
from scipy.optimize import minimize
import scipy.optimize as opt

# l2 norm of a vector
from numpy import array
from numpy.linalg import norm
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import json
import time



if __name__ == "__main__":

    const_Drones  = 1
    nDrones = 4
    tot_Drones = 5 
    # connect to the AirSim simulator
    client = airsim.MultirotorClient()
    client.confirmConnection()
    #Arm the drones and take off
    f=[]
    for i in range(0,tot_Drones):
        id = i+1
        client.enableApiControl(True, "Drone"+str(id))
        flagArm = client.armDisarm(True, "Drone"+str(id))
        print("Drone "+str(id)+" armed = {}".format(flagArm))
        f.append(client.moveToPositionAsync(0, 0, -50, 2, 15, vehicle_name="Drone"+str(id)))
        print("Drone "+str(id)+" took off = {}".format(flagArm))

    for i in range(0, tot_Drones):
        f[i].join()

    # airsim.wait_key('Press any key to takeoff')
    # f1 = client.takeoffAsync(vehicle_name="drone1")
    # f2 = client.takeoffAsync(vehicle_name="drone2")
    # f1.join()
    # f2.join()

    #Find the positions of the objects of interest
    object_rad = 6
    object_pose = np.zeros((7,nDrones))
    for i in range(0,nDrones):
        #pose= client.simGetObjectPose(object_names[i])
        object_pose[:,i]=array([0, 0, -50, 0, 0, 0, 0]) #hardcode the hover posituion of drones

    print('object_pose = {}'.format(object_pose))

    #Offsets of the drones from the player start location (local origin for the entire system)
    offset = np.zeros((3,nDrones))
    with open('/home/aryan/Documents/Unreal Projects/AirSimNH/LinuxNoEditor/offset.txt') as f:
        lc = 0;
        for line in f:
            data = line.split()
            offset[:,lc] = data
            lc = lc + 1

    print('offset ={}'.format(offset))
    f.close()

    


    drone_pos = np.zeros((3,nDrones))

    for i in range(0,nDrones):
        state0 = client.getMultirotorState(vehicle_name="Drone"+str(i+1))
        drone_pos[:,i] = [state0.kinematics_estimated.position.x_val, state0.kinematics_estimated.position.y_val, state0.kinematics_estimated.position.z_val]
        print("state, x = {}, y={}".format(state0.kinematics_estimated.position.x_val, state0.kinematics_estimated.position.y_val))

    print('drone_pos = {}'.format(drone_pos))


    #Find the positions to which the drones should move and capture images     

    ind=array([[-1,-1,1,1],[1,-1,-1,1]])
    dXm=20
    dYm=20
    dZm=-30               #Actual height of the cuboid

    NZp=5
    NYp=14
    NXp=14
    nPoints = NXp*NYp*NZp
    nextXPos = np.zeros((nDrones,nPoints))
    nextYPos = np.zeros((nDrones,nPoints))
    nextZPos = np.zeros((nDrones,nPoints))
    Yp0 = np.zeros((1,NYp))
    Zp0 = np.zeros((1,NZp))
    dx0 = float(dXm)/max(1,NXp-1)
    dy0 = float(dYm)/max(1,NYp-1)
    dz0 = float(dZm)/max(1,NXp-1)
    for i in range(0,nDrones):
        count=0
        dx=dx0*ind[0,i]
        dy=dy0*ind[1,i];
        dz=dz0;
        #xp0=dXm*ind[0,i]+object_pose[0,i];#generaTING ABOUT ORIGIN = remove obj_pose
        #yp0=dYm*ind[1,i]+object_pose[1,i];#same
        xp0=dXm*ind[0,i];#generaTING ABOUT ORIGIN = remove obj_pose
        yp0=dYm*ind[1,i];#same
        for iz in range(0,NZp):
            if iz==0:
                Zp0[0,iz]=-35 # -h_min
            else:
                Zp0[0,iz]=Zp0[0,iz-1]+dz
                yp0=Yp0[0,NYp-1]

            dy=-dy
            for iy in range(0, NYp):
                if iy==0:
                    Yp0[0,iy]=yp0
                else:
                    Yp0[0,iy]=Yp0[0,iy-1]+dy

                dx=-dx
                for ix in range(0, NXp):
                    if count==0:
                        nextXPos[i,count]=xp0
                    else:
                        if ix==0:
                            nextXPos[i,count]=nextXPos[i,count-1];
                        else:
                            nextXPos[i,count]=nextXPos[i,count-1]+dx

                    nextYPos[i,count]=Yp0[0,iy]
                    nextZPos[i,count]=Zp0[0,iz]
                    count=count+1


    print('nextXPosition = {}'.format(nextXPos))
    print('nextYPosition = {}'.format(nextYPos))
    print('nextZPosition = {}'.format(nextZPos))


    #airsim.wait_key('Press any key to move vehicles')
    frame_count = np.zeros((1,nDrones), int);
    #take the drones closer to their initial reference position
    ti = 0
    f=[]
    for i in range(0,nDrones):
            id = i+1
            if norm([object_pose[0,i]-nextXPos[i,ti], object_pose[1,i]-nextYPos[i,ti], object_pose[2,i]-nextZPos[i,ti]])<object_rad:
                continue
            yaw = math.atan2(object_pose[1,i]-nextYPos[i,ti], object_pose[0,i]-nextXPos[i,ti]);
            f.append(client.moveToPositionAsync(nextXPos[i,ti] - offset[0,i], nextYPos[i,ti] - offset[1,i], nextZPos[i,ti] - offset[2,i], 2, 15, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, (yaw)*float(180)/math.pi), vehicle_name="Drone"+str(id)))
            
    for i in range(0, nDrones):
        f[i].join()

    dXPos = np.zeros((nDrones,nPoints))
    dYPos = np.zeros((nDrones,nPoints))
    dZPos = np.zeros((nDrones,nPoints))
    flagPointSkipped = np.zeros((nDrones,nPoints))

    #Collecting the images
    folder_name_left =  '/home/aryan/Documents/Unreal Projects/AirSimNH/LinuxNoEditor/Dataset/nDrones_1/left_cam/'
    folder_name_right = '/home/aryan/Documents/Unreal Projects/AirSimNH/LinuxNoEditor/Dataset/nDrones_1/right_cam/'
    folder_name_depth = '/home/aryan/Documents/Unreal Projects/AirSimNH/LinuxNoEditor/Dataset/nDrones_1/depth_cam/'

    for ti in range(0,nPoints):
        f=[]
        for i in range(0,nDrones):
            id = i+1
            if norm([object_pose[0,i]-nextXPos[i,ti], object_pose[1,i]-nextYPos[i,ti], object_pose[2,i]-nextZPos[i,ti]])<object_rad:
                flagPointSkipped[i,ti] =1
                continue

            yaw = math.atan2(object_pose[1,i]-nextYPos[i,ti], object_pose[0,i]-nextXPos[i,ti]);
            f.append(client.moveToPositionAsync(nextXPos[i,ti] - offset[0,i], nextYPos[i,ti] - offset[1,i], nextZPos[i,ti] - offset[2,i], 2, 8, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, (yaw)*float(180)/math.pi), vehicle_name="Drone"+str(id)))
            #camera_autopitching as the drone moves in position
            #print('drone = {}'.format(id))
            responses_left = client.simGetImages([airsim.ImageRequest("front_left", airsim.ImageType.Scene)], vehicle_name="Drone"+str(id))  #scene vision image in uncompressed RGB array
            responses_right = client.simGetImages([airsim.ImageRequest("front_right", airsim.ImageType.Scene)], vehicle_name="Drone"+str(id))  #scene vision image in uncompressed RGB array
            responses_depth = client.simGetImages([airsim.ImageRequest("front_center", airsim.ImageType.DepthPerspective,True, False)], vehicle_name="Drone"+str(id))  #scene vision image in uncompressed RGB array

            response_left =responses_left[0]
            response_right =responses_right[0]
            response_depth =responses_depth[0]
            frame_count[0,i] = frame_count[0,i] + 1;
            filename_left = os.path.join(folder_name_left, "Drone_"+str(id)+"_frame_"+str(frame_count[0,i]))
            filename_right = os.path.join(folder_name_right, "Drone_"+str(id)+"_frame_"+str(frame_count[0,i]))
            filename_depth = os.path.join(folder_name_depth, "Drone_"+str(id)+"_frame_"+str(frame_count[0,i]))

            #convert depth image to grayscale
            #depth = np.array(response_depth.image_data_float, dtype=np.float32)
            #depth = depth.reshape(response_depth.height, response_depth.width)
            #depth = np.array(depth * 255, dtype=np.uint8)
            #png format
            #print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
            airsim.write_file(os.path.normpath(filename_left + '.png'), response_left.image_data_uint8)
            airsim.write_file(os.path.normpath(filename_right + '.png'), response_right.image_data_uint8)
            airsim.write_pfm(os.path.normpath(filename_depth + '.pfm'), airsim.get_pfm_array(response_depth))
            state0 = client.getMultirotorState(vehicle_name="Drone"+str(id))
            #print("state, x = {}, y={}".format(state0.kinematics_estimated.position.x_val, state0.kinematics_estimated.position.y_val))
            dXPos[i,ti] = state0.kinematics_estimated.position.x_val - object_pose[0,i] + offset[0,i]
            dYPos[i,ti] = state0.kinematics_estimated.position.y_val - object_pose[1,i] + offset[1,i]
            dZPos[i,ti] = state0.kinematics_estimated.position.z_val- object_pose[2,i] + offset[2,i]
        
        for i in range(0, nDrones):
            f[i].join()

  

    #airsim.wait_key('Press any key to reset to original state')
    #Disarm the drones and close the text files
    for i in range(0,tot_Drones):
        flagArm = client.armDisarm(False, "Drone"+str(i+1))
        text_file[i].close()

    client.reset()

    for i in range(0,tot_Drones):
        flagArm = client.enableApiControl(False, "Drone"+str(i+1))
