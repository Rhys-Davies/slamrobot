# Main robot script. 
# This runs the "rescue program" and starts the server, camera, etc.
# Landmark threshold file named lnd.npy must be in same folder as this 
# script or hardcoded default threshold values will be used. 
#
# On startup server blocks program until client connects. (Don't ask...)
#
# TODO: Oh boy...
#       
#       * Completely rewrite.
#       * Use a state machine.
#       * Networking that doesn't break when the client drops out.
#       * Allowing operation without a client connected.
#       * Allow resetting of run via client gui
#


# Author: Lachlan Gordon
# Networking, file loading, and landmark checking hacked in by Rhys Davies.

import numpy as np
import matplotlib #????
import matplotlib.pyplot as plt #????
import math as m
import cv2
import csv
import time

import penguinPi as ppi
import findtransform as tf
import networking as net

from streamPiCam import streamVision    
from drive import motionMotor  # drive python file
from landmarkDetector import lndmrkDetector
from slam import ekfSlam
#from gui import Gui

#Set Variables
print('####### WELCOME TO RESCUE SYSTEM #######')
print(' ')
print('#### BOOTING UP ####')
print(' ')
print('#####  ######')
print(' ')
print('YOUR RESCUE IS IMPORTANT')
print(' ')
print('TO US PLEASE STAY ON THE')
print(' ')
print('######## LINE ###########')
odometry = np.array([[0],[0]])


#Initialise modules
# Initilize
init_turn = 39
turn_angle_amount = 10
distance_amount = 1.3
distance_steps = 0.1

LND = lndmrkDetector()
ekf = ekfSlam()
drive = motionMotor()
vs = streamVision()
#gu = Gui()
center_distance = 0



# Initialise hardcoded defaults

dilateK = np.ones((3,3),np.uint8) # Dilation Kernel
erodeK = np.ones((1,1),np.uint8) # Erosion Kernel
lTh = np.array([[0, 0, 0],[10, 255, 255],[0, 0, 0],[179, 255, 255],[0, 0, 0],[179, 255, 255],[170, 0, 0],[179, 255, 255]]) # Landmark thresholds
distressRobot = []



print('Starting')
server = net.server(port=8081)
print('Waiting')
server.wait_and_connect()
print('Connected!')

def finishedRun():
    client.begin_session(IP)
    print('Setup Done')
    client.client_send(str(listofPlots))
    ppi.close()
   
def checkFiles():

        try:
            lTh = np.load('lnd.npy')
            #LND.setThresh(lTh)
            print('Found landmark threshold file, using it!')
        except:
            print('No updated landmark thresholds found, using defaults!')
            #LND.setThresh(lTh)
            lTh = np.array([[0, 0, 0],[10, 255, 255],[0, 0, 0],[179, 255, 255],[0, 0, 0],[179, 255, 255],[170, 0, 0],[179, 255, 255]]) # Landmark thresholds
        return lTh

def loadRun():
    textMap = []

    try:

        with open('map.txt') as csvfile:
            readCSV = csv.reader(csvfile, delimiter=',')
            for row in readCSV:
                textMap.append(row)

        distress = np.asarray(textMap[0])
        lnd1 = np.asarray(textMap[1])
        lnd2 = np.asarray(textMap[2])
                
        return distress, lnd1, lnd2

    except:
            print('Error Loading Map File!')
            return -1 -1 -1

def sendAll(mu,sig,ident):
    send_mu = mu.tolist()
    send_sig = sig.tolist()
    # WHY DID I DO THIS?!?!
    # Could have just combined into one message and split client-side.
    server.send_msg('mu' + ';' + str(send_mu))
    server.send_msg('sig' + ';' + str(send_sig))
    server.send_msg('ident' + ';' + str(ident))


def sendTransform(point,trans):
    
    send_point = point.tolist()
    server.send_msg('point' + ';' + str(send_point))
    server.send_msg('trans' + ';' + str(trans))
        
def findLandmarkIndex(ident,ident_list):
    # Must return -1 if an invalid landmark was given
    # Takes the ID of the landmark from the sensor and
    # Compares it to the list of known landmarks.
    # Returns the known X and Y co-ords of the landmark.

    n1 = -1
    n2 = -1

    # For-else loops are cool.
    for i in range(0,len(ident_list)):
        if ident[0] == ident_list[i]: # If the ID's match
            n1 = i #The index of the landmark


    for i in range(0,len(ident_list)):
        if ident[1] == ident_list[i]:
            n2 = i

    return n1, n2
    
def wraptopi(x):
    return m.atan2(m.sin(x), m.cos(x))   
    
    
#Load all relevant files
lTh = checkFiles()
distress, lnd1, lnd2 = loadRun()

#Setup the detector
LND.setMorphKernels(erodeK,dilateK,1,2)
LND.setThresh(lTh)

#Start vision
vs.start()
#print('Camera stream started')
time.sleep(1)
#print('Opencv ready')


current_x_pos = 0
current_y_pos = 0
current_theta = 0

# Drive functions
landmark_list = []
odometry = np.array([[0],[0]])

frame = vs.read()
LND.updateFrame(frame)
observed = LND.findLandmarks()
print('Found Landmarks',observed)

odo = np.array([0,0])

mu_t, sigma_t, landmark_list = ekf.updateSlam(observed, odo)
#gu.plotAll(mu_t,sigma_t,landmark_list,None)
sendAll(mu_t,sigma_t,landmark_list)

   
#turn 45 Degrees and Move 20 CM
#print('Turning 45 Degrees')
ot = drive.turn(m.radians(init_turn))
odo = np.array([0,ot])
frame = vs.read()
LND.updateFrame(frame)
observed = LND.findLandmarks()
mu_t, sigma_t, landmark_list = ekf.updateSlam(observed, odo)
#gu.plotAll(mu_t,sigma_t,landmark_list,None)
sendAll(mu_t,sigma_t,landmark_list)
time.sleep(0.2)

turn_amount = 0
odometry_distance = 0
odometry_turn = 0


while center_distance < distance_amount:
    #print('Driving Fowards ')
    od = drive.translate(distance_steps)
    odo = np.array([od,0])   

    frame = vs.read()
    LND.updateFrame(frame)
    observed = LND.findLandmarks()
    print('Found Landmarks',observed)

    mu_t, sigma_t, landmark_list = ekf.updateSlam(observed, odo)
    #gu.plotAll(mu_t,sigma_t,landmark_list,None)
    sendAll(mu_t,sigma_t,landmark_list)
    
    center_distance = center_distance + od
    time.sleep(0.1)
  
run = True 
while run:
    # print('odometry turn', odometry_turn)
    # print('odometry Dist', odometry_distance)
    odometry = np.array([odometry_distance,odometry_turn])
    #print('odometry', odometry)

    #Movement step, return odometry
    frame = vs.read()
    LND.updateFrame(frame)
    observed = LND.findLandmarks()
    print('Found Landmarks',observed)
    temp = LND.getboundingBox()
    red = LND.getBitmasked('lndmrkr')
    green = LND.getBitmasked('lndmrkg')
    blue = LND.getBitmasked('lndmrkb')
   
    ###### DRAW DATA TO SCREEN.    
    #cv2.imshow('Display window', temp)
    # cv2.imshow('RED',red)
    # cv2.imshow('GREEN',green)
    # cv2.imshow('BLUE',blue)
    #cv2.waitKey(500)
    
    #Update EKF and plot data
    #print('Odometry', odometry)
    mu_t, sigma_t, landmark_list = ekf.updateSlam(observed, odometry)
    print('landmark list',landmark_list)
    #print('MU_T', mu_t)
    #gu.plotAll(mu_t,sigma_t,landmark_list,None)
    sendAll(mu_t,sigma_t,landmark_list)
    time.sleep(0.1)
    
    print('hunting for Markers') 
    if turn_amount >= 720:
        #Stop Turning Drive Fowards
        print('We are no Longer Turning')
        run = False
       
    else: 
        print('Turning 30 Degrees')
        odometry_turn = drive.turn(m.radians(turn_angle_amount))
        turn_amount = turn_amount + turn_angle_amount
        #time.sleep(0.3)
        print('Turning 30 Degrees')

print('THANK YOU FOR WAITING')
print(' ')
print('YOU ARE ABOUT TO BE RESCUED')

#if mu_t.shape[0] == 13: #All 5 landmarks found
n1,n2 = findLandmarkIndex([int(lnd1[0]),int(lnd2[0])],landmark_list)

drive_to = False

print('lnd1',lnd1)
print('lnd2',lnd2)
print('index of 1',n1)
print('index of 2',n2)


if n1 is not -1 and n2 is not -1:
    a1 = 3 + (2*n1)
    a2 = 3 + (2*n2)
    m1 = mu_t[a1:a1+2]
    m2 = mu_t[a2:a2+2]
    t_theta,t_AxB,t_AyB = tf.findTransform(m1,m2,lnd1[1:3],lnd2[1:3])
    print('t_theta',t_theta)
    print('t_AxB',t_AxB)
    print('t_AyB',t_AyB)
    distressRobot = tf.transformPoint(distress[1:3],[t_theta,t_AxB,t_AyB])
    print('Transformed Distress Signal',distressRobot)
    sendTransform(distressRobot,[t_theta,t_AxB,t_AyB])
    drive_to = True
else:
    print('CRITICAL ERROR IN RESCUE MODULE')
    print('PATIENT IS DEAD')
           
while drive_to:  
    #X & Y COR 
    current_x_pos = float(mu_t[0])
    current_y_pos = float(mu_t[1])
    current_theta = float(mu_t[2])
    rescue_goal_x = float(distressRobot[0]) 
    rescue_goal_y = float(distressRobot[1])

    x_to_goal = rescue_goal_x - current_x_pos
    y_to_goal = rescue_goal_y - current_y_pos

    #calculate thetaD
    
    
    
    
    theta_d = m.atan2(y_to_goal,x_to_goal)
    # if rescue_goal_x < current_x_pos:
        # if current_theta > 0 and theta_d < 0:
            # theta_d = theta_d - 2 * m.pi
        # elif current_theta < 0 and theta_d > 0:
            # theta_d = theta_d + 2 * m.pi

    theta_to_goal = wraptopi(theta_d - current_theta) #+ m.pi
    #theta_to_goal = theta_to_goal * (2 * m.pi)
    print('theta_d: ',theta_d)
    print('current Theta: ', current_theta)
    print('theta goal: ', theta_to_goal)
 
    distance_to_goal = (m.sqrt(x_to_goal**2 + y_to_goal**2))
    if distance_to_goal < 0:
        distance_to_goal = distance_to_goal * -1
        
    print('distance to goal: ', distance_to_goal)
    
    ## TURN TO GOAL
    odometry_turn = drive.turn(theta_to_goal)
    odometry = np.array([[0],[odometry_turn]])
    
    frame = vs.read()
    LND.updateFrame(frame)
    observed = LND.findLandmarks()
    
    mu_t, sigma_t, landmark_list = ekf.updateSlam(observed, odometry)
    
    print('Turn Odom: ',odometry_turn)
    
    #if distance is a negative value.
    if distance_to_goal < 0:
        distance_to_goal = distance_to_goal * -1

    # #if less then 20cm stop    
    # if distance_to_goal >=0 and distance_to_goal<= 0.21:
        # print('Target Reached')
        # print('YOU HAVE BEEN RESCUED')
        # drive_to = False
        # break
        
    if distance_to_goal >= 0 and distance_to_goal <= 0.25:
        odometry_distance = drive.translate(0.150) 
        odometry = np.array([[odometry_distance],[0]])        
        mu_t, sigma_t, landmark_list = ekf.updateSlam(None, odometry)
        sendAll(mu_t,sigma_t,landmark_list)
        
        print('Target Reached')
        print('YOU HAVE BEEN RESCUED')
        drive_to = False
        break
        
    #Drive
    if distance_to_goal >= 0.15:     
        print('Distance is', str(distance_to_goal), ' cm to go')                
        #drive.translate(-0.1)
        odometry_distance = drive.translate(0.15)
        print('Dist Odom: ',odometry_distance)
        distance_to_goal = distance_to_goal-150

    
    odometry = np.array([[odometry_distance],[0]])
    print('odometry', odometry)

    #Movement step, return odometry
    frame = vs.read()
    LND.updateFrame(frame)
    observed = LND.findLandmarks()
    print('Found Landmarks',observed)
    temp = LND.getboundingBox()
    red = LND.getBitmasked('lndmrkr')
    green = LND.getBitmasked('lndmrkg')
    blue = LND.getBitmasked('lndmrkb')
   
    ###### DRAW DATA TO SCREEN.    
    #cv2.imshow('Display window', temp)
    # cv2.imshow('RED',red)
    # cv2.imshow('GREEN',green)
    # cv2.imshow('BLUE',blue)
    #cv2.waitKey(500)
    
    #Update EKF and plot data
    #print('Odometry', odometry)
    mu_t, sigma_t, landmark_list = ekf.updateSlam(observed, odometry)
    print('landmark list',landmark_list)
    #print('MU_T', mu_t)
    #gu.plotAll(mu_t,sigma_t,landmark_list,None)
    sendAll(mu_t,sigma_t,landmark_list)
    time.sleep(0.1)
    




  

