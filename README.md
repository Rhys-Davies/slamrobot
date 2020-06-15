# A Robot! 

A video of it in action can be found here: https://www.youtube.com/watch?v=Nohl0ReMUO8

This was a university project to program a mobile (differential drive) robot to perform
autonomous mapping and localisation in an arena containing 5 distinct landmarks. The robot
provided to us was based on a raspberryPi 2, and a raspberryPi Camera was provided for sensing.

# Operation

Perception is simple colour segmentation, with the blobs filtered and sorted into potential valid 
landmarks which are then sanity checked. The distance and angle to any valid landmarks is then calculated.
Image processing is done in the HSV colourspace with OpenCV.

Mapping and localisation is accomplished using EKF-SLAM.

The entire project was implemented in Python 3.4.

# Dependencies

These are the dependencies required for all python code in this repository.

* OpenCV 3.2.0 or above ("pip install opencv-python && opencv-contrib-python" will install OpenCV 3.4.x)
* Pyhon 3.4
  * TKinter
  * Numpy
  * Pillow
  * PIL-ImageTk (On linux this will have to be installed via apt. A full install of Anaconda contains this package already)

# Authors:

Lachlan Gordon (https://github.com/lachlangor) and myself, as noted in each file.

