# Interfaces a GUI with both Cv modules to allow for thresholding and testing
# without using the robot.
#
# TODO: Connect to robot via network.
#           - Retrievel of images directly from pi cam.
#           - Uploading/Saving thresholds directly to robot.
#           - Fix image colourspace being displayed in BGR instead of RGB
#
# Threshold Tool - v3
# Rhys Davies
#

import tkinter as tk
from tkinter.filedialog import askopenfilename, asksaveasfilename
import cv2
import numpy as np
import objectDetector as oD
import landmarkDetector as lD
from PIL import Image, ImageTk

class thGui(tk.Frame):

    image = None
    imageBoxed = None
    imageBitmask = None
 
    def __init__(self, master):

        self.fn = None

        self.root = master
        tk.Frame.__init__(self, master)
        self.obj = oD.objDetector()
        self.lnd = lD.lndmrkDetector()

        self.state = 'bitmask'

        # Initialize the menu bars
        self.menubar = tk.Menu(self.root)
        self.filemenu = tk.Menu(self.menubar, tearoff=0)
        self.filemenu.add_separator()
        self.filemenu.add_command(label="Open Local File", command=self.openFile)
        self.filemenu.add_command(label="Exit", command=exit)
        self.menubar.add_cascade(label="File", menu=self.filemenu)

        self.exportmenu = tk.Menu(self.menubar, tearoff=0)
        self.exportmenu.add_separator()
        self.exportmenu.add_command(label="Obj", command=lambda: self.exportTh('obj'))
        self.exportmenu.add_command(label="Landmark", command=lambda: self.exportTh('lndmrk'))
        self.menubar.add_cascade(label="Export Thresholds", menu=self.exportmenu)
        self.root.config(menu=self.menubar)

        self.importmenu = tk.Menu(self.menubar, tearoff=0)
        self.importmenu.add_separator()
        self.importmenu.add_command(label="Obj", command=lambda: self.importTh('obj'))
        self.importmenu.add_command(label="Landmark", command=lambda: self.importTh('lndmrk'))
        self.menubar.add_cascade(label="Import Thresholds", menu=self.importmenu)
        self.root.config(menu=self.menubar)


        self.frameTh = tk.Frame(self.root,width=440, height=640)
        self.thVar = tk.StringVar()
        self.o1 = tk.OptionMenu(self.frameTh, self.thVar, "obj", "lndmrkr", "lndmrkg", "lndmrkb",command = self.updateConfig)
        self.thVar.set('obj')
        self.o1.place(x=0, y=10)
        self.lh = tk.Scale(self.frameTh, label='H-Low',from_=0, to=179, length=200,showvalue=2,orient=tk.HORIZONTAL, command=self.updateThreshold)
        self.lh.place(x=0, y=50)
        self.hh = tk.Scale(self.frameTh,label='H-High', from_=0, to=179, length=200,orient=tk.HORIZONTAL, command=self.updateThreshold)
        self.hh.place(x=220, y=50)
        #self.hh.set(179)
        self.lhwrap = tk.Scale(self.frameTh,label='H-Low Wrap', from_=0, to=179, length=200,orient=tk.HORIZONTAL, command=self.updateThreshold)
        self.lhwrap.place(x=0,y=120)
        self.hhwrap = tk.Scale(self.frameTh,label='H-High Wrap', from_=0, to=179, length=200,orient=tk.HORIZONTAL, command=self.updateThreshold)
        self.hhwrap.place(x=220,y=120)
        #self.hhwrap.set(179)
        self.ls = tk.Scale(self.frameTh, label='S-Low',from_=0, to=255, length=200,orient=tk.HORIZONTAL, command=self.updateThreshold)
        self.ls.place(x=0,y=190)
        self.hs = tk.Scale(self.frameTh, label="S-High", from_=0, to=255, length=200,orient=tk.HORIZONTAL, command=self.updateThreshold)
        self.hs.place(x=220,y=190)
        #self.hs.set(255)
        self.lv = tk.Scale(self.frameTh, label="V-Low", from_=0, to=255, length=200, orient=tk.HORIZONTAL, command=self.updateThreshold)
        self.lv.place(x=0,y=260)
        self.hv = tk.Scale(self.frameTh, label="V-High", from_=0, to=255, length=200, orient=tk.HORIZONTAL, command=self.updateThreshold)
        self.hv.place(x=220,y=260)
        #self.hv.set(255)
        self.erode = tk.Scale(self.frameTh, label="Erode", from_=1, to=100, length=200, orient=tk.HORIZONTAL, command=self.updateMorph)
        self.erode.set(5)
        self.erode.place(x=0,y=330)
        self.erodePasses = tk.Scale(self.frameTh, label="Erode Passes", from_=1, to=10, length=200, orient=tk.HORIZONTAL, command=self.updateMorph)
        self.erodePasses.place(x=220,y=330)
        self.dilate = tk.Scale(self.frameTh, label="Dilate", from_=1, to=100, length=200, orient=tk.HORIZONTAL, command=self.updateMorph)
        self.dilate.set(12)
        self.dilate.place(x=0,y=400)
        self.dilatePasses = tk.Scale(self.frameTh, label="Dilate Passes", from_=1, to=10, length=200, orient=tk.HORIZONTAL, command=self.updateMorph)
        self.dilatePasses.place(x=220,y=400)
        self.switchFrame = tk.Button(self.frameTh, text="Switch View", command=self.imSwitch)
        self.switchFrame.place(x=10, y = 500)

        self.frameIm = tk.Frame(self.root,width=640, height=640)
        self.imgBoxed = tk.Label(self.frameIm, text="RGB", image=None)
        self.imgBoxed.grid(row=0, column=0)
        self.imgMasked = tk.Label(self.frameIm, text="Masked", image=None)
        self.imgMasked.grid(row=0, column=0)

        self.frameIm.grid(column=0,row=0)
        self.frameTh.grid(column=1,row=0)

        self.oTh = np.array([[20, 20, 20], [179, 255, 255]], dtype=np.uint8)
    # lTh = [[red1 lower],[red1 upper],[red2 lower],[red2 upper],[green lower],[green upper],[blue lower],[blue upper]]
        self.lTh = np.array([[170, 21, 22],[179, 255, 255],[23, 24, 25],[10, 255, 255],[26, 27, 28],[179, 255, 255],[29, 30, 31],[179, 255, 255]], dtype=np.uint8)
        
        self.loadOnStartup()
        
        self.dilateK = None
        self.erodeK = None
        self.morphPasses = [1,1]

        self.updateConfig('meh')

    def imSwitch(self):
        if self.state == 'bitmask':
            self.state = 'boxed'
            self.imgBoxed.tkraise()
        elif self.state == 'boxed':
            self.state = 'bitmask'
            self.imgMasked.tkraise()
        else:
            print('Frame switching error')

    def updateIm(self):
 
        self.imageBoxed = Image.fromarray(self.imageBoxed)
        self.imageBoxed = ImageTk.PhotoImage(self.imageBoxed)
        self.imgBoxed.configure(image=self.imageBoxed)
        self.imgBoxed.image = self.imageBoxed

        self.imageBitmask = Image.fromarray(self.imageBitmask)
        self.imageBitmask = ImageTk.PhotoImage(self.imageBitmask)
        self.imgMasked.configure(image=self.imageBitmask)
        self.imgMasked.image = self.imageBitmask


    def loadOnStartup(self):
        try:
            self.lTh = np.load('lnd.npy')
            self.updateConfig('meh')
            self.updateCV()
            print('Found landmark threshold file, using it!')
        except:
            print('No updated landmark thresholds found, using defaults!')

        try:
            self.oTh = np.load('obj.npy')
            self.updateConfig('meh')
            self.updateCV()
            print('Found object threshold file, using it!')
        except:
            print('No updated object thresholds found, using defaults!')

    def updateThreshold(self,var):

        mode = self.thVar.get()

        lh = self.lh.get()
        ls = self.ls.get()
        lv = self.lv.get()
        
        hh = self.hh.get()
        hs = self.hs.get()
        hv = self.hv.get()

        hhw = self.hhwrap.get()
        lhw = self.lhwrap.get()

        if mode == 'obj':
            self.oTh = np.array([[lh,ls,lv],[hh,hs,hv]], dtype=np.uint8) 
            #print('Updated Obs threshold')
            self.obj.setThresh(self.oTh)
        elif mode == 'lndmrkr':
            self.lTh[0,:] = np.array([lh,ls,lv], dtype=np.uint8)
            self.lTh[1,:] = np.array([hh,hs,hv], dtype=np.uint8)
            self.lTh[2,:] = np.array([lhw,ls,lv], dtype=np.uint8)
            self.lTh[3,:] = np.array([hhw,hs,hv], dtype=np.uint8) 
            #print('Updated landmark threshold')
            self.lnd.setThresh(self.lTh)
        elif mode == 'lndmrkg':
            self.lTh[4,:] = np.array([lh,ls,lv], dtype=np.uint8)
            self.lTh[5,:] = np.array([hh,hs,hv], dtype=np.uint8)  
            #print('Update landmark g threshold')
            self.lnd.setThresh(self.lTh)
        elif mode == 'lndmrkb':
            self.lTh[6,:] = np.array([lh,ls,lv], dtype=np.uint8)
            self.lTh[7,:] = np.array([hh,hs,hv], dtype=np.uint8)
            #print('Update landmark b threshold')
            self.lnd.setThresh(self.lTh)
        else:
            print('Oopsie-daisy')

        self.updateCV()

    def updateMorph(self,var):

        if self.fn == None:
            return 0

        d = self.dilate.get()
        e = self.erode.get()
        f = self.erodePasses.get()
        g = self.dilatePasses.get()

        self.dilateK = np.ones((d,d),np.uint8)
        self.erodeK = np.ones((e,e),np.uint8)

        self.morphPasses[0] = f
        self.morphPasses[1] = g

        self.lnd.setMorphKernels(self.erodeK,self.dilateK,self.morphPasses[0],self.morphPasses[1])
        self.obj.setMorphKernels(self.erodeK,self.dilateK,self.morphPasses[0],self.morphPasses[1])
        print('updated Morph')
        self.updateCV()

    def openFile(self):
        self.fn = askopenfilename()

        if self.fn:
            self.image = cv2.imread(self.fn)
            self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
            #self.image = cv2.flip(self.image,-1)
            self.lnd.updateFrame(self.image)
            self.obj.updateFrame(self.image)
            self.updateCV()
            print('Image opened successfully')
        else:
            print('Failed to open')


    def exportTh(self,var):
        fn = asksaveasfilename()
        if fn == None:
            print('save cancelled')
            return 0
        if var == 'obj':
            np.save(fn, self.oTh)
        elif var == 'lndmrk':
            np.save(fn, self.lTh)
        else:
            print('save error')
            return(0)

    def importTh(self,var):
        fn = askopenfilename()
        if fn == None:
            print('save cancelled')
            return 0
        if var == 'obj':
            self.oTh = np.load(fn)
            self.updateConfig('meh')
            self.updateCV()
        elif var == 'lndmrk':
            self.lTh = np.load(fn)
            self.updateConfig('meh')
            self.updateCV()
        else:
            print('Load error')
            return(0)

    def updateConfig(self,meh):

        var = self.thVar.get()

        if var == 'obj':
            self.lh.set(self.oTh[0,0])
            self.ls.set(self.oTh[0,1])
            self.lv.set(self.oTh[0,2])
            self.hh.set(self.oTh[1,0])
            self.hs.set(self.oTh[1,1])
            self.hv.set(self.oTh[1,2])
            #print('Switched to Obstacle Mode')
        elif var == 'lndmrkr':
            self.lh.set(self.lTh[0,0])
            self.ls.set(self.lTh[0,1])
            self.lv.set(self.lTh[0,2])
            self.hh.set(self.lTh[1,0])
            self.hs.set(self.lTh[1,1])
            self.hv.set(self.lTh[1,2])
            self.lhwrap.set(self.lTh[2,0])
            self.hhwrap.set(self.lTh[3,0])
            #print('Switched to Landmark Red mode')
        elif var == 'lndmrkg':
            self.lh.set(self.lTh[4,0])
            self.ls.set(self.lTh[4,1])
            self.lv.set(self.lTh[4,2])
            self.hh.set(self.lTh[5,0])
            self.hs.set(self.lTh[5,1])
            self.hv.set(self.lTh[5,2])
            #print('Switched to Landmark Green mode')
        elif var == 'lndmrkb':
            self.lh.set(self.lTh[6,0])
            self.ls.set(self.lTh[6,1])
            self.lv.set(self.lTh[6,2])
            self.hh.set(self.lTh[7,0])
            self.hs.set(self.lTh[7,1])
            self.hv.set(self.lTh[7,2])
            #print('Switched to Landmark Blue mode')
        else:
            print('Whoops var')
            return(0)

    def updateCV(self):

        if self.fn == None:
            #print('No image selected yet')
            return 0

        alist = self.obj.findObj()
        blist = self.lnd.findLandmarks()

        print('Landmark Detector Found:')
        print(blist)
        print('Object Detector Found:')
        print(alist)

        var = self.thVar.get()

        if var == 'obj': 
            self.imageBoxed = self.obj.getboundingBox()
            self.imageBitmask = self.obj.getBitmasked()
        else:
            self.imageBoxed = self.lnd.getboundingBox()
            self.imageBitmask = self.lnd.getBitmasked(var)

        self.updateIm()
        



if __name__ == "__main__":

    root = tk.Tk()
    #root.resizable(width=False, height=False)
    app = thGui(root)
    root.mainloop()
