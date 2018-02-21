# The CV Processing module 2
# This module detects the landmarks identified by three coloured stripes.
#
# Author: Rhys Davies
#

import numpy as np
import math as m
import cv2
from operator import itemgetter
import proto as proto

## Landmark Detection Class ##

class lndmrkDetector():

    def __init__(self):

        self.image = None
        self.hsvImage = None
        self.thl = np.array([[0, 0, 0], [179, 255, 255], [0, 0, 0], [179, 255, 255], [0, 0, 0], [179, 255, 255], [0, 0, 0], [179, 255, 255]]) # Landmark thresholds
        self.lrbit = None # Landmark red bitmask
        self.lgbit = None # Landmark green bitmask
        self.lbbit = None # Landmark blue bitmask
        self.ekernel = np.ones((1, 1), np.uint8) # Erosion kernel
        self.dkernel = np.ones((1, 1), np.uint8) # Dilation kernel
        self.epasses = 1 # Erosion passes
        self.dpasses = 1 # Dilation passes
        self.landmarkList = None # Tuple of (color,centroidx,centroidy,contour) for each landmark blob
        self.boundBox = []
        self.boundBoxCol = []
        self.possible_landmarks = [45,57,27,29,38]
        print('Landmark Detector Initialized')


    # Takes a raw bgr frame and loads it into this opencv object.
    # Use this one for reading frames from the PiCamera.
    # PiCamera must be running in raw bgr array mode.
    def updateFrame(self, bgrArray):
        if bgrArray == None:
            print('No frame to update')
            return None
        else:
            self.image = bgrArray.copy()
            #cv2.imshow('test',self.image)
            #cv2.waitKey(500)
            self.hsvImage = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
            #print('Import operation complete')
            self.obsBoundBox = None
            self.landMarkBoundBox = None

    def processImage(self):

        lrbit1 = cv2.inRange(self.hsvImage, self.thl[0, :], self.thl[1, :])
        lrbit2 = cv2.inRange(self.hsvImage, self.thl[2, :], self.thl[3, :])

        self.lrbit = lrbit1+lrbit2
        self.lrbit = cv2.erode(self.lrbit, self.ekernel, iterations=self.epasses)
        self.lrbit = cv2.dilate(self.lrbit, self.dkernel, iterations=self.dpasses)

        self.lgbit = cv2.inRange(self.hsvImage, self.thl[4, :], self.thl[5, :])
        self.lgbit = cv2.erode(self.lgbit, self.ekernel, iterations=self.epasses)
        self.lgbit = cv2.dilate(self.lgbit, self.dkernel, iterations=self.dpasses)

        self.lbbit = cv2.inRange(self.hsvImage, self.thl[6, :], self.thl[7, :])
        self.lbbit = cv2.erode(self.lbbit, self.ekernel, iterations=self.epasses)
        self.lbbit = cv2.dilate(self.lbbit, self.dkernel, iterations=self.dpasses)

    # Finds the approx distance and angle to each
    # detected landmark in the scene.
    # Takes arguments: X centroid of one of the stripes, total height of the striped
    # area in pixels
    def findLandmarkLoc(self,cx,totheight):
        dist = (3.04*123*proto.CAM_V)/(totheight*3.67)
        if cx > proto.CAM_H/2:
            angle = ((proto.HFOV/2)/(proto.CAM_H/2)) * (cx-(proto.CAM_H/2))
        elif cx < proto.CAM_H/2:
            angle = (((proto.HFOV/2)/(proto.CAM_H/2)) * cx) - (proto.HFOV/2)
        elif cx == proto.CAM_H:
            angle = 0
        else:
            angle = 0
            print('Angle error, cx =',cx)
        
        # Convert dist to m
        dist = dist/1000

        if angle > 31.1:
            angle = 31.1
            print('Angle positive capped')
        if angle < -31.1:
            print('Angle negative capped')
            angle = -31.1
        
        # Conver angle to rad     
        angle = m.radians(angle)

        return dist, angle


    # Finds all red, green and blue squares
    # Needs some serious re-writing, not very memory efficient.
    def findLandmarks(self):
        print('Detecting landmarks...')
        self.processImage()
    
        self.boundBox = []
        self.boundBoxCol = []

        # Find all contours in the r,g,b bitmasks
        _, cntr, _ = cv2.findContours(self.lrbit, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        _, cntg, _ = cv2.findContours(self.lgbit, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        _, cntb, _ = cv2.findContours(self.lbbit, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnt = cntr+cntg+cntb 

        # If < 3 contours, a landmark cannot possibly exist
        if len(cnt) < 3:
            return None
        
        # A matching list of each contours colour
        colorlist = [proto.RED]*len(cntr)+[proto.GREEN]*len(cntg)+[proto.BLUE]*len(cntb)
        areas = [cv2.contourArea(c) for c in cnt]

        # Remove anything with area = 0 (tiny contours whose areas are so small they round down to 0)
        cnt, colorlist, areas = zip(*((x, y, z) for x, y, z in zip(cnt, colorlist, areas) if z > 0))
        
        if len(cnt) < 3:
            print('Not enough blobs after filtering')
            return None

        # Generate X,Y and bounding boxes
        moments = [cv2.moments(c) for c in cnt]
        X = list([int(m['m10']/m['m00']) for m in moments])
        Y = list([int(m['m01']/m['m00']) for m in moments])
        boxes = list([cv2.boundingRect(c) for c in cnt])

        # Sort all by ascending X order.
        (X,Y,cnt,areas,boxes,colorlist) = zip(*sorted(zip(X,Y,cnt,areas,boxes,colorlist),key = lambda b:b[0], reverse=False))
        
        landmark = []
        landmarkID = []

        # Search through and find groups of three blobs that are close to each other.
        for i in range(0,len(X)-1):
            _, _, w, h = boxes[i]
            upper = X[i] + (w/3)
            lower = X[i] - (w/3)
            uppery = Y[i] + (h*2)
            lowery = Y[i] - (h*2)
            aupper = areas[i] + areas[i]*0.3
            alower = areas[i] - areas[i]*0.3
            #print('bounds: ' + str(upper) + ',' + str(lower) + ';' + str(uppery) + ',' + str(lowery))
            count = 0
            temp = []
            tempcol = []
            for j in range(i,len(X)):
                if (lower <= X[j] <= upper) and (lowery <= Y[j] <= uppery) and (alower <= areas[j] <= aupper):
                    #print('found:' + str(count))
                    temp.append(cnt[j])
                    tempcol.append(colorlist[j])
                    if count == 2:
                        #print('found 3')
                        i = j
                        landmark.append(temp)
                        landmarkID.append(tempcol)
                    count += 1
                else:
                    #print('Next')
                    pass
                
        print('Landmarks found: ' + str(len(landmark)))
        returnList = []

        for i in range(len(landmark)):
            lcnt = landmark[i]
            lMoments = [cv2.moments(c) for c in lcnt]
            lColorCode = landmarkID[i]
            lX = list([int(m['m10']/m['m00']) for m in lMoments])
            lY = list([int(m['m01']/m['m00']) for m in lMoments])
            
            # Sort the entries in landmark i by y value 
            (lX,lY,lMoments,lcnt,lColorCode) = zip(*sorted(zip(lX,lY,lMoments,lcnt,lColorCode),key = lambda b:b[1], reverse=False))

            if lY[2] == lY[0]:
                print('Error in landmark detector')
                return None
            totheight = lY[2] - lY[0]
            dist, ang = self.findLandmarkLoc(lX[1],totheight)
            #print('dist,ang: ' + str(dist) + ',' + str(ang))

            ang = ang*-1
            
            ids = int(lColorCode[0] + lColorCode[1] + lColorCode[2],2)
            #print('ID: ' + str(id))

            #returnList = np.append(returnList,[id,dist,ang],axis=0)
            #returnList.append([id,dist,ang])
            #print('Landmarks: ',returnList)
            
            if ids in self.possible_landmarks and dist < 1.8:
                returnList.append([ids,dist,ang])
            # else:
                # landmark[i].remove
                # landmarkID[i].remove
                
                
        if len(landmark) > 0:
            for i in landmark:
                for h in i:
                    points = cv2.boundingRect(h)
                    self.boundBox.append(points)

            for m in landmarkID:
                for n in m:
                    self.boundBoxCol.append(n)

            return np.asarray(returnList)
        else:
            return None

        #(ID, colorlist, cx, cy, cnt, areas) = zip(*sorted(zip(ID, colorlist, cx, cy, cnt, areas),key = lambda b: (b[2],b[3]), reverse=False))

        # Remove and contours with area 0


    ############# SETTER FUNCTIONS #############

    # Set the designated threshhold. newThresh must be a numpy
    # array the same size and dimension as the one being replaced.

    def setThresh(self,new):
        self.thl = new

    # Sets the kernel that will be used for dilation or erosion
    # var = 'e' for erosion
    # var = 'd' for dilation
    # Kernel must be a numpy array
    def setMorphKernels(self,kernele,kerneld,passese,passesd):
        self.dkernel = kerneld.copy()
        self.ekernel = kernele.copy()
        self.epasses = passese
        self.dpasses = passesd
        #print('Updated morph kernels in cv module')

    ############# GETTER FUNCTIONS ############
    
    # Gets and returns the requested bitmask
    def getBitmasked(self,var):

        if var == 'lndmrkr':
            #print('Returning landmark red bitmasked image')
            temp = cv2.bitwise_and(self.image,self.image,mask=self.lrbit)
            return temp
        elif var == 'lndmrkg':
            #print('Returning landmark green bitmasked image')
            temp = cv2.bitwise_and(self.image,self.image,mask=self.lgbit)
            return temp
        elif var == 'lndmrkb':
            #print('Returning landmark blue bitmasked image')
            temp = cv2.bitwise_and(self.image,self.image,mask=self.lbbit)
            return temp
        else:
            print('Invalid bitmasked request')
            return self.image

    def getboundingBox(self):
        image = self.image.copy()
        if self.boundBox == None:
            #print('No Obstacle Bounding Boxes')
            return image
        else:
            for i in range(len(self.boundBox)):
                x, y, w, h = self.boundBox[i]

                col = (255,255,0)

                if self.boundBoxCol[i] == '01':
                    cv2.rectangle(image, (x, y), (x+w, y+h), (255,0,0), 2)
                elif self.boundBoxCol[i] == '10':
                    cv2.rectangle(image, (x, y), (x+w, y+h), (0,255,0), 2)
                elif self.boundBoxCol[i] == '11':
                    cv2.rectangle(image, (x, y), (x+w, y+h), (0,0,255), 2)
                else: 
                    cv2.rectangle(image, (x, y), (x+w, y+h), (255,255,255), 2)
               

            return image
