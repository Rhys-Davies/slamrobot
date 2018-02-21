# Processing module 1
# This just looks for a single colour, 90mm square object.
#
# Author: Rhys Davies
#

import numpy as np
import math as m
import cv2
from operator import itemgetter
import proto as proto

## Object Detection Class ##

class objDetector():

    def __init__(self):

        self.image = None
        self.hsvImage = None
        self.objHeight = 90 # The height of obstacle cube in mm
        self.tho = np.array([[0, 0, 0], [179, 255, 255]]) # Obstacle thresholds
        self.obit = None # Obstacle bitmask
        self.ekernel = np.ones((1, 1), np.uint8) # Erosion kernel
        self.dkernel = np.ones((1, 1), np.uint8) # Dilation kernel
        self.epasses = 1 # Erosion passes
        self.dpasses = 1 # Dilation passes
        self.obsBoundBox = None # Co-ords in pixels for Obs bounding box. Is None if no goal found

        print('Object Detector Initialized')

    # Takes a raw bgr frame and loads it into this opencv object.
    # Use this one for reading frames from the PiCamera.
    # PiCamera must be running in raw bgr array mode.
    def updateFrame(self, bgrArray):
        if bgrArray == None:
            print('No frame to update')
            return None
        else:
            self.image = bgrArray.copy()
            self.hsvImage = cv2.cvtColor(self.image, cv2.COLOR_RGB2HSV)
            #self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
            #print('Import operation complete')
            self.obsBoundBox = None
            self.landMarkBoundBox = None

    def processImage(self):

        self.obit = cv2.inRange(self.hsvImage, self.tho[0, :], self.tho[1, :])
        self.obit = cv2.erode(self.obit, self.ekernel, iterations=self.epasses)
        self.obit = cv2.dilate(self.obit, self.dkernel, iterations=self.dpasses)


    # Looks for contours in a bitmask image.
    # This function will always return the largest
    # single contour present
    def findOneContour(self):
        _, contours, _ = cv2.findContours(self.obit, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        boundingBoxes = [cv2.boundingRect(c) for c in contours]
        if len(boundingBoxes) <= 0:
            print('found nothin, leave')
            return None
        picked = 0
        cnt = contours[0]
        M = cv2.moments(cnt)
        carea = int(M['m00'])
        for b in range(1, len(boundingBoxes)):
            cnt2 = contours[b]
            M2 = cv2.moments(cnt2)
            carea2 = int(M2['m00'])
            if carea2 > carea:
                M = M2
                carea = carea2
                picked = b
                cnt = cnt2

        #print('Blob ' + str(picked) + ' had the biggest area')
        return cnt


    # Finds the approx distance and angle to object.
    # Takes height in pixels and x,y of centroid in pixels

    def findObjLoc(self, cx, totheight):
        dist = (3.04*self.objHeight*proto.CAM_V)/(totheight*3.67)
        if cx > proto.CAM_H:
            angle = ((proto.HFOV/2)/(proto.CAM_H/2)) * (cx-proto.CAM_H)
        elif cx < proto.CAM_H:
            angle = (((proto.HFOV/2)/(proto.CAM_H/2)) * cx) - (proto.HFOV/2)
        elif cx == proto.CAM_H:
            angle = 0

        return dist, angle

    def findObj(self):
        
        self.processImage()
        cnt = self.findOneContour()
        if cnt == None:
            print('Found no obstacles... Leaving')
            self.obsBoundBox = None
            return None
        self.obsBoundBox = cv2.boundingRect(cnt)
        M = cv2.moments(cnt)
        x,y,w,h = self.obsBoundBox
        cx = x+(w/2)
        dist, ang = self.findObjLoc(w,cx)
        #print('Returning dist ang to obs')
        print('Obj at ' + str(dist) + ', ' + str(ang))
        return dist, ang

    ############# SETTER FUNCTIONS #############

    def setThresh(self, new):
        self.tho = new


    def setMorphKernels(self, kernele, kerneld, passese, passesd):
        self.dkernel = kerneld.copy()
        self.ekernel = kernele.copy()
        self.epasses = passese
        self.dpasses = passesd
        print('Set morph in detector')

    ############# GETTER FUNCTIONS ############
    
    def getBitmasked(self):
        temp = cv2.bitwise_and(self.image, self.image, mask=self.obit)
        return temp

    def getboundingBox(self):
        image = self.image.copy()
        if self.obsBoundBox == None:
            #print('No Obstacle Bounding Boxes')
            return image
        else:
            x, y, w, h = self.obsBoundBox
            cv2.rectangle(image, (x, y), (x+w, y+h), (255, 0, 0), 2)
            return image