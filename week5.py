#!/usr/bin/env python3
#
#   motordemo.py
#
#   This shows how to interface with the GPIO (general purpose I/O)
#   pins and how to drive the PWM for the motors.  Please use as an
#   example, but change to suit the weekly goals.
#
# Imports
import pigpio
import sys
import time
import math
# Define the motor pins.
MTR1_LEGA = 7   #right
MTR1_LEGB = 8
MTR2_LEGA = 5   #left
MTR2_LEGB = 6
IR_LEFT = 14 #left IR sensor
IR_CENTER = 15 #center IR sensor
IR_RIGHT = 18 #right IR sensor

class Motor:

    def __init__(self):
        # Initialize the connection to the pigpio daemon (GPIO interface).
        self.io = pigpio.pi()
        if not self.io.connected:
            print("Unable to connection to pigpio daemon!")
            sys.exit(0)
            
        # Set up the four pins as output (commanding the motors).
        self.io.set_mode(MTR1_LEGA, pigpio.OUTPUT)
        self.io.set_mode(MTR1_LEGB, pigpio.OUTPUT)
        self.io.set_mode(MTR2_LEGA, pigpio.OUTPUT)
        self.io.set_mode(MTR2_LEGB, pigpio.OUTPUT)
        
        self.io.set_mode(IR_LEFT, pigpio.INPUT)
        self.io.set_mode(IR_CENTER, pigpio.INPUT)
        self.io.set_mode(IR_RIGHT, pigpio.INPUT)

        # Prepare the PWM.  The range gives the maximum value for 100%
        # duty cycle, using integer commands (1 up to max).
        self.io.set_PWM_range(MTR1_LEGA, 255)
        self.io.set_PWM_range(MTR1_LEGB, 255)
        self.io.set_PWM_range(MTR2_LEGA, 255)
        self.io.set_PWM_range(MTR2_LEGB, 255)
        
        # Set the PWM frequency to 1000Hz.  You could try 500Hz or 2000Hz
        # to see whether there is a difference?
        self.io.set_PWM_frequency(MTR1_LEGA, 1000)
        self.io.set_PWM_frequency(MTR1_LEGB, 1000)
        self.io.set_PWM_frequency(MTR2_LEGA, 1000)
        self.io.set_PWM_frequency(MTR2_LEGB, 1000)
        
        # Clear all pins, just in case.
        self.io.set_PWM_dutycycle(MTR1_LEGA, 0)
        self.io.set_PWM_dutycycle(MTR1_LEGB, 0)
        self.io.set_PWM_dutycycle(MTR2_LEGA, 0)
        self.io.set_PWM_dutycycle(MTR2_LEGB, 0)
        print("GPIO ready...")


    def shutdown(self):
        # Clear all pins, just in case.
        print("Turning off")
        self.io.set_PWM_dutycycle(MTR1_LEGA, 0)
        self.io.set_PWM_dutycycle(MTR1_LEGB, 0)
        self.io.set_PWM_dutycycle(MTR2_LEGA, 0)
        self.io.set_PWM_dutycycle(MTR2_LEGB, 0)

        self.io.stop()

    def set(self, leftdutycycle, rightdutycycle):
        
        if leftdutycycle < 0.0:
            self.io.set_PWM_dutycycle(MTR1_LEGA, 0)
            self.io.set_PWM_dutycycle(MTR1_LEGB, int(-1*leftdutycycle*255))
        else:
            self.io.set_PWM_dutycycle(MTR1_LEGA, int(leftdutycycle*255))
            self.io.set_PWM_dutycycle(MTR1_LEGB, 0)

        if rightdutycycle < 0.0:
            self.io.set_PWM_dutycycle(MTR2_LEGA, 0)
            self.io.set_PWM_dutycycle(MTR2_LEGB, int(-1*rightdutycycle*255))
        else:
            self.io.set_PWM_dutycycle(MTR2_LEGA, int(rightdutycycle*255))
            self.io.set_PWM_dutycycle(MTR2_LEGB, 0)
        
#         print("Left motor PWM: ", leftdutycycle*255)
#         print("Right motor PWM: ", rightdutycycle*255)
        
    def setlinear(self, speed):
        """
        Speed is given in meters per second
        """
        # slopeavg = 1/0.600465
        slope1 = 1/0.552776
        slope2 = 1/0.656167
        self.set(speed*slope1, speed*slope2)
        
    def setspin(self, speed):
        """
        Speed is given in degrees per second.
        """
        slope = 1/469.19
        self.set(speed*slope, -1*speed*slope)
    
    def setvel(self, linear, spin):
        if (spin == 0):
            self.setlinear(linear)
        else:
            d = 2*linear/spin       
            width = 0.1285875
            T = 2*math.pi/spin
            v_outer = math.pi*(d+width)/T
            v_inner = math.pi*(d-width)/T
            self.set(v_outer*1/0.552776, v_inner*1/0.656167)
    
    def spin(self, dir):
        """
        Speed is given in degrees per second.
        """
        # left: 1 or -3
        # backward: 2 or -2
        # right: 3 or -1
        # no turn: 0
        # positive spin left, negative spin right
        if (dir == 3): # turn the shorter way
            dir = -1
        if (dir == -3):
            dir = 1
#       dir_to_speed = {0: 0, 1: -245, 2: -310, 3: -385, -1: 245, -2: 310, -3: 385}
        dir_to_speed = {0: 0, 1: -285, 2: -310, 3: -385, -1: 285, -2: 310, -3: 385}
        slope = 1/469.19
        d = dir_to_speed[dir]
        self.set(d*slope, -1*d*slope)
        time.sleep(0.7)
        self.setvel(0,0)
        time.sleep(0.7)
        
    def linefollow(self):
        vel_nom = 0.4
        rad_small = 20*math.pi/180
        rad_large = 40*math.pi/180
        state = 'C'
        searching = True
        angularspeed = 0.9
        lost_counter = 0
        exitcond = False;
        try:
            while not exitcond:
                # Reading IR Sensors
                ir_left_state = self.io.read(IR_LEFT)
                ir_center_state = self.io.read(IR_CENTER)
                ir_right_state = self.io.read(IR_RIGHT)
                
#                 if (lost_counter >= 200): #detects if the robot is lost after set number of cycles
#                     searching = True
#                     lost_counter = 0
#                 
#                 if (searching):
#                     print("lost")
#                     if angularspeed > 0.0001: # must maintain an angular speed greater than 0 to keep circling
#                         angularspeed -= 0.0001
#                     self.setvel(0.3, angularspeed) 
#                     if (ir_left_state == 1 or ir_right_state == 1 or ir_center_state == 1):
#                         searching = False
#                     continue
#                 
#                 print("left: " , ir_left_state)
#                 print("center: " , ir_center_state)
#                 print("right: " , ir_right_state)
                
                # Setting motor states
                if (ir_left_state == 0 and ir_center_state == 1 and ir_right_state == 0): # centered
                    print("centered!")
                    self.setvel(vel_nom, 0)
                    lost_counter = 0
                    state = 'C'
                elif (ir_left_state == 0 and ir_center_state == 1 and ir_right_state == 1): # slight left
                    print("slight left!")
                    self.setvel(vel_nom, math.sin(rad_small)*vel_nom/0.125)
                    lost_counter = 0
                    state = 'L'
                elif (ir_left_state == 0 and ir_center_state == 0 and ir_right_state == 1): # more left
                    print("more left!")
                    self.setvel(vel_nom, math.sin(rad_large)*vel_nom/0.125)
                    lost_counter = 0
                    state = 'L'
                elif (ir_left_state == 1 and ir_center_state == 1 and ir_right_state == 0): # slight right
                    print("slight right!")
                    lost_counter = 0
                    state = 'R'
                    self.setvel(vel_nom, -1*math.sin(rad_small)*vel_nom/0.125)
                elif (ir_left_state == 1 and ir_center_state == 0 and ir_right_state == 0): # more right
                    print("more right!")
                    lost_counter = 0
                    state = 'R'
                    self.setvel(vel_nom, -1*math.sin(rad_large)*vel_nom/0.125)
                elif (ir_left_state == 0 and ir_center_state == 0 and ir_right_state == 0): # 3 cases
                    print("passed the end of street or pushed off street!")
                    self.setvel(0,0)
                    exitcond = True
                elif (ir_left_state == 1 and ir_center_state == 1 and ir_right_state == 1):
                    print("seeing intersection")
                    time.sleep(0.4)
                    self.setvel(0,0)
                    exitcond = True
                else: # special case 101 where you can do anything
                    print("Currently in a turning state. Continuing what I was doing.")
        except:
            self.setvel(0,0)
            
    def readSensors(self):
        left = self.io.read(IR_LEFT)
        center = self.io.read(IR_CENTER)
        right = self.io.read(IR_RIGHT)
        return [left, center, right]
        
    def checkIntersections(self):
        """
        Returns a list of Booleans if an intersection is found at this location.
        
        Returns False if an intersection is not found at this location and True if an intersection is found.
        List is in the order [forward, left, backward, right]
        """
        sensors = self.readSensors()
        intersections = []
        if (sensors[0] == 1 or sensors[1] == 1 or sensors[2] == 1):
            intersections.append(True)
        else:
            intersections.append(False)
        # Spin left
        self.spin(1)
        sensors = self.readSensors()
        if (sensors[0] == 1 or sensors[1] == 1 or sensors[2] == 1):
            intersections.append(True)
        else:
            intersections.append(False)
        # Straighten
        self.spin(-1)
        # Add backwards is True
        intersections.append(True)
        # Spin right
        self.spin(-1)
        sensors = self.readSensors()
        if (sensors[0] == 1 or sensors[1] == 1 or sensors[2] == 1):
            intersections.append(True)
        else:
            intersections.append(False)
        # Straighten
        self.spin(1)
        return intersections 
        
        
        

#
#   Main
#
if __name__ == "__main__":
    ############################################################
    
    motor = Motor()
    motor.linefollow()
    time.sleep(1)
    intersections = motor.checkIntersections()
    print(intersections)
    


    motor.shutdown()   