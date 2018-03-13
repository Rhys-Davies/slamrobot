from threading import Thread
from picamera import PiCamera
from picamera.array import PiRGBArray
import numpy as np
import proto
import time

# Threaded camera frame grabber
# Heavily inspired by pyimagesearch's tutorial
#
# Astute readers will notice a design flaw that will (and did) cause 
# the CV module to retrieve corrupt images. 
#
# Hint: Race condition.
#
# TODO: Add a mutex lock to self.frame
#
# Author: Rhys Davies
#

class streamVision():

    IM_WIDTH = proto.CAM_H
    IM_HEIGHT = proto.CAM_V 

    def __init__(self, resolution=(IM_WIDTH, IM_HEIGHT), framerate=20):
        self.camera = PiCamera() 
        self.camera.resolution = (self.IM_WIDTH, self.IM_HEIGHT)

        #White balance adjustment!
        self.camera.awb_mode = 'off' #Options: 'off', 'sunlight', 'cloudy', 'shade', 'tungsten', 'incandescent',
                                 # 'flash', 'horizon', 'auto', 'fluorescent'
        self.camera.awb_gains = (1.3,2.5) #Need to set this if awb_mode = off
        self.camera.vflip = False
        self.camera.hflip = False
        #self.camera.rotation = 180
        # self.camera.framerate = 50 #Not needed... most likely
        self.rawCap = PiRGBArray(self.camera, size=resolution)
        self.stream = self.camera.capture_continuous(self.rawCap,
                    format="bgr", use_video_port=True)

        self.frame = None
        self.stopped = False

    def start(self):
        Thread(target=self.update, args=()).start()
        return self 
        
    def take_img(self):    
        number_of_images = 30
        for idx in range(number_of_images):
            self.camera.capture('image%s.jpg' %idx)
            print('Image Captured')
            time.sleep(1)
            
    def update(self):
        for f in self.stream:
            self.frame = f.array
            self.rawCap.truncate(0)

            if self.stopped:
                self.stream.close()
                self.rawCap.close()
                self.camera.close()
                return

    def read(self):
        print('Frame Requested')
        return self.frame

    def stop(self):
        self.stopped = True

if __name__ == "__main__":
    vs = streamVision()
    time.sleep(2.0)

    vs.take_img()    
    pass
