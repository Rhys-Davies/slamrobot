# A Robot! 

A video of it in action can be found here: https://www.youtube.com/watch?v=Nohl0ReMUO8

This was a univeristy project to program a mobile (differential drive) robot to perform
autonomous mapping and localisation in an arena containing 5 distinct landmarks. The robot
provided to us was based on a raspberryPi 2, and a raspberryPi Camera was provided for perception.

# Operation

Perception is simple colour segmentation, with the blobs filtered and sorted into potential valid 
landmarks which are then sanity checked. The distance and angle to any valid landmarks is then calculated.
Image processing is done in the HSV colourspace.

Mapping and localisation is accomplished using EKF-SLAM.

# Dependencies

These are the dependencies required for all python code in this repository.

* OpenCV 3.2.0 (Compiled with Python 3 bindings)
* Pyhon 3.4
  * TKinter
  * Numpy
  * Pillow
  * PIL-ImageTk

# Authors:
Lachlan Gordon (https://github.com/lachlangor) and myself, as noted in each file.

