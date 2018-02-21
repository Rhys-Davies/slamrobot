import numpy as np
import math as m

# Functions to find the transform and use the transform
# to find a point.
# 
# Author: Rhys Davies

#http://apmonitor.com/che263/index.php/Main/PythonSolveEquations

def findTransform(m1,m2,m1t,m2t):

    # m1 = marker 1 in robots reference frame
    # m2 = marker 2 in robots reference frame
    # m1t = marker 1 in distress reference frame
    # m2t = marker 2 in distress reference frame

    Ax1 = float(m1[0])
    Ay1 = float(m1[1])
    Ax2 = float(m2[0])
    Ay2 = float(m2[1])

    Bx1 = float(m1t[0])
    By1 = float(m1t[1])
    Bx2 = float(m2t[0])
    By2 = float(m2t[1])
    
    a = np.array([[-By1,Bx1,1,0],
                  [ Bx1,By1,0,1],
                  [-By2,Bx2,1,0],
                  [ Bx2,By2,0,1]])

    b = np.array([Ax1,Ay1,Ax2,Ay2])

    x = np.linalg.solve(a,b) #x = [sinth,costh,axb,ayb]

    theta = m.atan2(x[0],x[1])

    AxB = x[2]
    AyB = x[3]

    return theta, AxB, AyB

def transformPoint(pnt,trns):
    
    a = np.array([[m.cos(trns[0]),-m.sin(trns[0]),float(trns[1])],
                  [m.sin(trns[0]),m.cos(trns[0]),float(trns[2])],
                  [0.0,0.0,1.0]])

    b = np.array([[float(pnt[0])],
                  [float(pnt[1])],
                  [1.0]])

    return np.dot(a,b)

if __name__ == "__main__":
    
    # m1, m2, m1t, m2t
    theta, AxB, AyB = findTransform([0.18,1.38],[1.06,0.125],[1.5486,-1.0253],[1.8137,0.48437])
    print('theta',theta)
    print('AxB',AxB)
    print('AyB',AyB)
    transform = transformPoint([1.2374,-0.17678],[theta,AxB,AyB])
    print('transformed point',transform[0:2])
