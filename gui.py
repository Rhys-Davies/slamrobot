# GUI module used in robotGUI.

# Author: Rhys Davies


import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import math as m
import numpy as np


class Gui(): 
    def __init__(self):

        plt.ion()
        self.fig = plt.figure(figsize=(10,8))
        self.ax = self.fig.add_subplot(121)
        self.bx = self.fig.add_subplot(122)
        self.fig.tight_layout()

    def plotDaBot(self,mu_t,sigma_t):
        
        coneLength = 0.1
        halfFOV = m.radians(90)
   
        LX = mu_t[0] + coneLength*m.cos(mu_t[2])
        LY = mu_t[1] + coneLength*m.sin(mu_t[2])

        RX = mu_t[0] + coneLength*m.cos(mu_t[2]+halfFOV)
        RY = mu_t[1] + coneLength*m.sin(mu_t[2]+halfFOV)

        #self.chart.clear()
        self.data1 = self.ax.plot([mu_t[0],LX],[mu_t[1],LY],c='r')
        self.data2 = self.ax.plot([mu_t[0],RX],[mu_t[1],RY],c='b')

        self.data3 = self.bx.plot([mu_t[0],LX],[mu_t[1],LY],c='r')
        self.data4 = self.bx.plot([mu_t[0],RX],[mu_t[1],RY],c='b')

        sigma_2 = sigma_t[0:2,0:2]
        point = mu_t[0:2]

        w, h, theta = self.gen_cov_ellipse(sigma_2,point)
        ell = Ellipse(xy=[float(point[0]),float(point[1])],width=w, height=h,
                        angle=theta, edgecolor='k', lw = 1, facecolor='none')
        self.ax.add_patch(ell)

        ell2 = Ellipse(xy=[float(point[0]),float(point[1])],width=w, height=h,
                        angle=theta, edgecolor='k', lw = 1, facecolor='none')

        self.bx.add_patch(ell2)


    def plotAll(self,mu_t,sigma_t,ids,point):
        self.bx.clear()

        self.plotDaBot(mu_t,sigma_t)
        self.plotLandmarks(mu_t,sigma_t,ids)
        if point is not None:
            self.plotPoint(point)

        self.bx.set_ylim(-0.5,2.5)
        self.bx.set_xlim(-0.5,2.5)
        self.ax.set_xlim(-0.5,2.5)
        self.ax.set_ylim(-0.5,2.5)
        self.fig.canvas.draw()

    def plotPoint(self,point):
        self.ax.scatter(point[0],point[1],c='r')
        self.bx.scatter(point[0],point[1],c='r')

    def plotLandmarks(self,mu_t,sigma_t,ids):
        
        # n = number landmark is in list
        col = ['c','m','y','b','g']
        l = int((len(mu_t) - 3)/2)

        scatter1 = []
        scatter2 = []
        
        if l is not 0:        
            for i in range(0,l):
                if i > 5:
                    col = 'r'
                a = 3 + (2*i)
                lnd = mu_t[a:a+2,0]
                sigma = sigma_t[a:a+2,a:a+2]
                scatter1.append(self.ax.plot(lnd[0], lnd[1],c=col[i],marker='*'))
                scatter2.append(self.bx.plot(lnd[0], lnd[1],c=col[i],marker='*'))

                self.bx.annotate(str(ids[i]), xy=(float(lnd[0]),float(lnd[1])),size=11)

                w, h, theta = self.gen_cov_ellipse(sigma,lnd)
                ell = Ellipse(xy=[float(lnd[0]),float(lnd[1])],width=w, height=h,
                              angle=theta, edgecolor=col[i], lw = 1, facecolor='none')
                self.ax.add_patch(ell)
                ell2 = Ellipse(xy=[float(lnd[0]),float(lnd[1])],width=w, height=h,
                              angle=theta, edgecolor=col[i], lw = 1, facecolor='none')
                self.bx.add_patch(ell2)


    def gen_cov_ellipse(self, cov, pos, nstd=3):

    # Cov = the 2x2 error ellipse
    # Pos = the X,Y of the ellipse
    # nstd = sigma bounds, default 3

        def eigsorted(cov):
            vals, vecs = np.linalg.eigh(cov)
            order = vals.argsort()[::-1]
            return vals[order], vecs[:,order]

        vals, vecs = eigsorted(cov)
        theta = np.degrees(np.arctan2(*vecs[:,0][::-1]))
        w, h = 2 * nstd * np.sqrt(vals)

        return w, h, theta

