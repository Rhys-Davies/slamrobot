
import slam as sl
import gui
import math as m
import numpy as np
import sys
import time
import random as r




import modules.networking as net


#gu = modules.gui.Gui()
ekf = sl.ekfSlam()   
## Variables ##

m = [] # Sim landmark locations
sensor = [] # Sim sensor readings
X = [] # Sim real robot pos
odom = [] # Sim odometry
timestep = 0.2


def getTrue(X,k):
    return np.array([[X[k,0]],[X[k,1]]])


def getSimObs(sensor,num_z,k):

    num = r.randint(0,5)

    idx = k * num_z
    z = sensor[idx:idx+num_z,:]

    #Add stuff here to simulate returning ID
    temp = []
    for i in range(0,z.shape[0]):
        temp.append([i,float(z[i,0]),float(z[i,1])])

    z = np.asarray(temp)
    
    np.random.shuffle(z)
    
    temp = []
    if num == 0:
        return None
    else:
        num = 1
        for i in range(0,num):
            print(i)
            temp.append([float(z[i,0]),float(z[i,1]),float(z[i,2])])
        z = np.asarray(temp)
    
    #print(z)
    
    return z

def getOdo(odom,k):
            
    delta_d   = odom[k,0]
    delta_t  = odom[k,1]
    return np.array([delta_d, delta_t])

def loadSimData():
        
    m = np.genfromtxt('slammap.txt',delimiter=',')
    sensor = np.genfromtxt('slamsensor.txt',delimiter=',')
    X = np.genfromtxt('slamtrue.txt',delimiter=',')
    odom = np.genfromtxt('slamodom.txt',delimiter=',')
    num_z = len(m)

    return m, sensor, X, odom, num_z

m, sensor, X, odom, num_z = loadSimData()



print('Starting')
server = net.server()
print('Waiting')
server.wait_and_connect()
print('Connected!')


for k in range(0,50):


    real = getTrue(X,k)
    z = getSimObs(sensor,num_z,k)
    odo = getOdo(odom,k)
    mu_t, sigma_t, ident = ekf.updateSlam(z,odo)
    point = [1,1]
    #gu.plotAll(mu_t,sigma_t,ident,None)
    send_mu = mu_t.tolist()
    send_sig = sigma_t.tolist()
    server.send_msg('mu' + ';' + str(send_mu))
    server.send_msg('sig' + ';' + str(send_sig))
    server.send_msg('ident' + ';' + str(ident))
    server.send_msg('point' + ';' + str(point))
    time.sleep(0.2)

input("Press Enter to exit...")
server.send_msg('done' + ';' + 'done')