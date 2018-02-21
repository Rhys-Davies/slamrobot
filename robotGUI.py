# This is the program that connects to the Robot and displays map data.
# Change the IP address in  client.begin_session('IP here') to match that
# of the robot.
#
# TODO:
#      * IP address and ports as arguments when launching the program.
#      * Alternatively, add a gui element to enter port and IP address.
#      * Allow reseting of this gui and robot mainscript from this gui to    #        get around relaunching everything after each run.
#      * Make networking a bit more efficient... its operation is best
#        described as spewing packets onto the network...
#
# Author: Rhys Davies


from gui import Gui
import socket
import json
import math as m
import numpy as np
import networking as net 
import argparse

client = net.client(port=8081)
client.begin_session('172.19.232.129')
print('Connected')

gu = Gui()

mu_t = None
sigma_t = None
ident = None
point = None
trans = None

while True:

    data = client.recv_msg()
    if data == None:
        print('No data recieved')
    else:
        data = data.decode("utf-8").split(';')

    if data[0] == 'mu':
        #print('Tried to decode mu_t')
        mu_t = np.asarray(json.loads(data[1]))
    elif data[0] == 'sig':
        sigma_t = np.asarray(json.loads(data[1]))
    elif data[0] == 'ident':
        ident = json.loads(data[1])
    elif data[0] == 'point':
        point = json.loads(data[1])
        print('Transformed Point:',point)
    elif data[0] == 'done':
        print('done')
    elif data[0] == 'trans':
        trans = json.loads(data[1])
        print('Transform:',trans)
        a = np.array([[m.cos(trans[0]),-m.sin(trans[0]),trans[1]],
                [m.sin(trans[0]),m.cos(trans[0]),trans[2]],
                [0,0,1]])
        print('Transform Matrix',a)
    else:
        break

    if mu_t is not None and sigma_t is not None and ident is not None:
        gu.plotAll(mu_t,sigma_t,ident,point)
        mu_t = None
        sigma_t = None
        ident = None
    
input("Press Enter to exit...")
