import numpy as np
import math as m
import time
from numpy.linalg import inv

# obs is an array [ID, range, bearing]
# odo is an array [delta_d, delta_theta]
# REMEMBER, BEACONS ARE ORDERED IN THE ORDER THEY WERE SPOTTED
# USE THIS TO APPROPRIATELY INDEX MU AND SIGMA

# Using .dot() to perform matrix multiplication as the @ operator was
# not added to python 3 until 3.5 (Rasbian Python version is 3.4).

# EKF Psudocode: Lachlan Gordon
# Author: Rhys Davies

class ekfSlam():
    
    def __init__(self):
        self.mu_t = np.array([[0.2],[0.2],[0.0]])
        self.sigma_t = np.zeros([3,3])
        self.landmark_list = [] #List of ID's in the order they are observed


        sigr = 0.05 # Translation error
        sigbeta = m.radians(2) #Rotation error
        sigd = 0.1 # Sensor distance error
        sigtheta = m.radians(3) # Sensor angle error

        self.R = np.array([[sigr**2, 0], 
                           [0, sigbeta**2]])
        
        self.Q = np.array([[sigd**2, 0],
                           [0, sigtheta**2]])
    
    def updateSlam(self,obs,odo):
      
        #print('odo',odo)
        delta_d = float(odo[0])
        delta_theta = float(odo[1])
        mu_bar = self.mu_t
        sigma_bar = self.sigma_t

        #print('old mu_bar',mu_bar)
        
        new_state = self.update_state(self.mu_t[0:3], delta_d, delta_theta)

        mu_bar[0:3] = np.array(new_state) 

        #print('mu_bar',mu_bar)

        l = int((len(mu_bar) - 3)/2)

        Jxr = self.getJx(delta_d,delta_theta)
        Jur = self.getJu(delta_theta)

        if l == 0:
            Jx = Jxr
            Ju = Jur
        else:
            
            Jx1 = np.append(Jxr, np.zeros([3,2*l]), axis=1)
            Jx2 = np.append(np.zeros([2*l,3]), np.eye(2*l), axis=1)
            Jx = np.append(Jx1,Jx2,axis=0)

            Ju = np.append(Jur,np.zeros([2*l,2]), axis=0)

        JxT = np.transpose(Jx)
        JuT = np.transpose(Ju)

        sigma_bar = (np.dot(Jx,self.sigma_t).dot(JxT)) + (np.dot(Ju,self.R).dot(JuT))

        # Update
        # Will need to modify this to handle vision input. Check if landmark exists and return its ID in the list
        # otherwise add it to list. 
        # ID in list will be its index, as beacons will be ordered in mu_t in the order they are detected.
        if obs is not None:
            for row in obs: #for a in range...

                res = self.matchLandmark(int(row[0])) # Should return unseen == true if not seen
                # all indexing for obs will need to be adjusted
                
                l = int((len(mu_bar) - 3)/2)
                if res == -1: #If landmark is not ID'd
                    new = self.initLnd(mu_bar[0:3],row[1:3])
                    mu_bar = np.append(mu_bar,new,axis=0)
                    Lz, LzT = self.getL(mu_bar[0:3],row[1:3])
                    a = len(sigma_bar)

                    b = np.dot(Lz,self.Q).dot(LzT)

                    sigma_bar = np.append(np.append(sigma_bar,np.zeros([a,2]),axis=1),np.append(np.zeros([2,a]),b,axis=1),axis=0)
                    self.landmark_list.append(int(row[0])) #Add new landmark ID to list
                    n = len(self.landmark_list)-1 #set n to the newly added landmark
                else:
                    n = res #set n to position of landmark in list 
                    
                a = 3 + (2*n)

                zti = self.getZ(mu_bar[0:3], mu_bar[a:a+2])

                s = np.array([[row[1]],[row[2]]]) - zti

                s[1] = self.wraptopi(float(s[1]))

                Gt, GtT = self.getG(mu_bar[0:3],mu_bar[a:a+2],zti[0],n,l)


                kt = np.dot(sigma_bar,GtT).dot(inv((np.dot(Gt,sigma_bar).dot(GtT)) + self.Q))

                mu_bar = mu_bar + np.dot(kt,s)

                mu_bar[2] = self.wraptopi(mu_bar[2])

                sigma_bar = np.dot(np.eye(len(sigma_bar)) - np.dot(kt,Gt),sigma_bar)

        self.mu_t = mu_bar
        self.sigma_t = sigma_bar

        return self.mu_t, self.sigma_t, self.landmark_list


    def initLnd(self,bot,z):
        x = float(bot[0])
        y = float(bot[1])
        th = float(bot[2])
        b = float(z[1])
        r = float(z[0])

        theta = self.wraptopi(th+b)
        lnew = np.array([[x + r * m.cos(theta) ],
                         [y + r * m.sin(theta)]])

        return lnew

    

    def getG(self,bot,lnd,r,n,totl):
        # n = landmark number (location in mu)
        # totl = total number of landmarks in map
        # r = range from robot estimated to landmark

         if r is not 0:

            xl = float(lnd[0])
            yl = float(lnd[1])
            yr = float(bot[1])
            xr = float(bot[0])
            r = float(r)

            G1 = np.array([[-1*(xl-xr)/r,     -1*(yl-yr)/r,   0],
                           [(yl-yr)/r**2,  -1*(xl-xr)/r**2,  -1]])

            G2 = np.array([[(xl-xr)/r, (yl-yr)/r],
                           [-1*(yl-yr)/r**2, (xl-xr)/r**2]])

            if 2*n == 0 and totl < 2:
                G = np.append(G1,G2,axis=1)
                #G = np.append(G,np.zeros([2,(totl-n)*2]),axis=1)
            elif (totl-n)*2 == 0:
                G = np.append(G1,np.zeros([2,2*n]),axis=1)
                G = np.append(G,G2,axis=1)
            else:
                G = np.append(G1,np.zeros([2,2*n]),axis=1)
                G = np.append(G,G2,axis=1)
                G = np.append(G,np.zeros([2,(totl-1-n)*2]),axis=1)

            GT = np.transpose(G)
        
            return  G, GT 


    def getZ(self,bot,lnd):
     
        y = lnd[1] - bot[1]
        x = lnd[0] - bot[0]

        r = m.sqrt((bot[0] - lnd[0])**2 + (bot[1] - lnd[1])**2)
        B = self.wraptopi(m.atan2(y,x) - bot[2])

        return np.array([[r], [B]])
        
    # Update States         
    def update_state(self,state,deltad,delta_theta): 

        x = float(state[0])
        y = float(state[1])
        th = float(state[2])

        a = x + (deltad * m.cos(th))
        b = y + (deltad * m.sin(th))
        c = self.wraptopi(th+delta_theta)

        return np.array([[a],
                         [b],
                         [c]]) #

    #TODO Check and fix this
    def matchLandmark(self,idLnd):  
        # Must return -1 if an invalid landmark was given
        # Takes the ID of the landmark from the sensor and
        # Compares it to the list of known landmarks.
        # Returns the known X and Y co-ords of the landmark.

        # For-else loops are cool.
        for i in range(0,len(self.landmark_list)):
            if idLnd == self.landmark_list[i]: # If the ID's match
                return i #The index of the landmark
        else:
            return -1



    def wraptopi(self,x):
        return m.atan2(m.sin(x), m.cos(x))


    def getL(self,mu,z):

        th = float(mu[2])
        b = float(z[1])
        r = float(z[0])

        theta = self.wraptopi(th+b)

        Lz = np.array([[m.cos(theta), -r*m.sin(theta)],
                       [m.sin(theta),  r*m.cos(theta)]])

        LzT = np.transpose(Lz)

        return Lz, LzT

    def getJu(self,theta):
                
        Ju = np.array([[m.cos(theta), 0],
                       [m.sin(theta), 0],
                       [0, 1]])
        
        return Ju

    def getJx(self,delta_d,theta):

        Jx = np.array([[1, 0, ((-1*delta_d)*m.sin(theta))],
                [0, 1, delta_d* m.cos(theta)],
                [0, 0, 1]])

        return Jx
