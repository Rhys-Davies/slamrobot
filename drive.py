## Motor Controller
## Author: Lachlan Gordon



import penguinPi as ppi
import math as m
import time


#drive 
distance = 0.1
angle = m.radians(-45)


class motionMotor():
	
	## initilize	
    def __init__(self):
        self.mA = ppi.Motor(ppi.AD_MOTOR_A)
        self.mB = ppi.Motor(ppi.AD_MOTOR_B)
        ppi.init()

### Turn ###
    def turn(self,radangle):
        #radangle = radangle * -1
        # Declare Variables
        self.turn_speed = 0
        self.turn_speed_count = 0
        self.turn_speed_count2 = 0
        self.time = 1
        
        self.rightPrev = self.mA.get_ticks()
        self.leftPrev = self.mB.get_ticks()
        check = True
        self.direction = 1
        #radangle = radangle * -1

        if radangle > 0:
            self.mA.set_power(34)
            self.mB.set_power(30)
            self.direction = 1
            #print('anti-clockwise')
        elif radangle < 0:
            self.mA.set_power(-34)
            self.mB.set_power(-30)
            self.direction = -1
            #print('clockwise')
            
        print(str(self.direction))
		
        self.ticks = m.floor(radangle*0.072*1763) * self.direction
        self.count = (self.mA.get_ticks() - self.rightPrev) * self.direction
        self.count2 = (self.mB.get_ticks() - self.leftPrev) * self.direction
        self.disance_travelled_start_L = self.count2
        self.disance_travelled_start_R = self.count
        
        print(str(self.ticks))
		
        while check:
            self.tic = time.clock()
            
            self.count = (self.mA.get_ticks() - self.rightPrev) * self.direction
            self.count2 = (self.mB.get_ticks() - self.leftPrev) * self.direction
            
            # get loop ticks
            self.turn_speed_count = self.count - self.turn_speed_count
            self.turn_speed_count2 = self.count2 - self.turn_speed_count2
            
            # Ticks to mm conversion, then to Meters
            self.turn_speed_distanceR = (self.turn_speed_count * 0.567)/ 1000
            self.turn_speed_distanceL = (self.turn_speed_count2 * 0.567)/ 1000
            
            # Speed = Distance/Time
            self.turn_speedR = (self.turn_speed_distanceR/self.time)/1000
            self.turn_speedL = (self.turn_speed_distanceL/self.time)/1000
            #self.count = (self.mA.get_ticks() - self.rightPrev) * self.direction
            #self.count2 = (self.mB.get_ticks() - self.leftPrev) * self.direction
                       
            #print("R: " + str(self.count) + "," + "L: " + str(self.count2))
            #print("Speed of Left: " + str(self.turn_speedL) + " m/s")
            #print("Speed of Right: " + str(self.turn_speedR) + " sec")
            #print("Time: " + str(self.time) + " m/s")
            
           
            
            if self.count > self.ticks:
                self.mA.set_power(0)
                self.mB.set_power(0)
                check = False
                self.disance_travelled_end = self.count
                self.disance_travelled_start = self.disance_travelled_start_R
                #print("Right Triggered: " + str(self.count))
                
            elif self.count2 > self.ticks:
                self.mA.set_power(0)
                self.mB.set_power(0)
                check = False
                self.disance_travelled_end = self.count2
                #print("Left Triggered: " + str(self.count2))
                self.disance_travelled_start = self.disance_travelled_start_L
               
            time.sleep(0.005)
            
            self.toc = time.clock()
            self.time = self.toc - self.tic
            #print(self.direction)
        # To Calculate Angle Moved
        self.disance_travelled_ticks = (self.disance_travelled_end - self.disance_travelled_start)
        self.disance_travelled_rad = ((self.disance_travelled_ticks*125)/15867) * self.direction
        
        print("Distance Traveled Rads: " + str(self.disance_travelled_rad))
        print("Distance Traveled in Deg: " + str(m.degrees(self.disance_travelled_rad)))
        
        return self.disance_travelled_rad
            
            
### Fowards ###
    def translate(self,dist):
        #dist = dist * -1
        self.rightPrev = self.mA.get_ticks()
        self.leftPrev = self.mB.get_ticks()
        self.direction = 1
        self.speed = 0
        self.speed_count = 0
        self.speed_count2 = 0
        self.time = 1
        check = True
        dist = dist * -1

        if dist > 0:
            self.mA.set_power(34) #LEFT  
            self.mB.set_power(-30)  #RIGHT 
            self.direction = 1
        elif dist < 0:
            self.mA.set_power(-34)
            self.mB.set_power(30)
            self.direction = -1

        self.ticks = m.floor(dist * 1763)* self.direction
        self.disance_travelled_start = (self.mA.get_ticks() + self.mB.get_ticks())/2
        
        #print(str(self.ticks))
		
		
        while check:
            
            
            self.tic = time.clock()
            
            self.count = (self.mA.get_ticks() - self.rightPrev) * self.direction
            self.count2 = (self.mB.get_ticks() - self.leftPrev) * self.direction
            
            # get loop ticks
            self.speed_count = self.count - self.speed_count
            self.speed_count2 = self.count2 - self.speed_count2
            
            # Ticks to mm conversion, then to Meters
            self.speed_distanceR = (self.speed_count * 0.567)/ 1000
            self.speed_distanceL = (self.speed_count2 * 0.567)/ 1000
            
            # Speed = Distance/Time
            self.speedR = (self.speed_distanceR/self.time)/1000
            self.speedL = (self.speed_distanceL/self.time)/1000
            
            #Print Results
            #print("R: " + str(self.count) + "," + "L: " + str(self.count2))
            
            #print("Speed of Left: " + str(self.speedL) + " m/s")
            #print("Speed of Right: " + str(self.speedR) + " m/s")
            #print("Time: " + str(self.time) + " sec")
            if self.count > self.ticks or self.count2 > self.ticks:
                self.mA.set_power(0)
                self.mB.set_power(0)
                check = False

            time.sleep(0.005)
            self.toc = time.clock()
            self.time = self.toc - self.tic

        self.disance_travelled_end = (self.mA.get_ticks() + self.mB.get_ticks())/2
        self.disance_travelled_ticks =self.disance_travelled_end - self.disance_travelled_start
        self.disance_travelled = self.disance_travelled_ticks * (0.567/1000)
        if self.disance_travelled < 0:
            self.disance_travelled = self.disance_travelled * -1
        #print("Distance Traveled: " + str(self.disance_travelled))

            
        return self.disance_travelled
		
if __name__ == '__main__':
    #pass
    drive = motionMotor()
    #drive.translate(distance)
    drive.turn(angle)
        #drive.translate(distance)
