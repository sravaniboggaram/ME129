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
            
#     def spiral(self):
#         for pwmlevel in range(0.83, 0, -1*0.01):
#             #motor.setvel(pwmlevel*0.351099, 2*pwmlevel*0.351099/0.5)
#             motor.setvel(0.351099, pwmlevel*2*0.5*0.351099/0.5) # tests that path is spiral if w is constant
#             speed = pwmlevel*0.600465
#             time.sleep(0.1)

#
#   Main
#
if __name__ == "__main__":
    ############################################################
    
    motor = Motor()
    vel_nom = 0.4
    rad_small = 15*math.pi/180
    rad_large = 30*math.pi/180
    state = 'C'
    searching = True
    cw = 1 # 1 for clockwise, -1 for counter-clockwise
    
#     motor.setvel(vel_nom, -1*math.sin(rad)*vel_nom/0.125)
#     time.sleep(7)
    
    # Problem 3
#     try:
#         motor.setvel(vel_nom, 0)
#         while True:
#             # Reading IR Sensors
#             ir_left_state = motor.io.read(IR_LEFT)
#             ir_center_state = motor.io.read(IR_CENTER)
#             ir_right_state = motor.io.read(IR_RIGHT)
#             
#             print("left: " , ir_left_state)
#             print("center: " , ir_center_state)
#             print("right: " , ir_right_state)
#             
#             # Setting motor states
#             if (ir_right_state == 1) : # this includes 001 and 011
#                 print("Turning Left!")
#                 motor.setvel(vel_nom, math.sin(rad_small)*vel_nom/0.125)
#             elif (ir_left_state == 1): # this includes 100 and 110
#                 print("Turning Right!")
#                 motor.setvel(vel_nom, -1*math.sin(rad_small)*vel_nom/0.125)
#             elif (ir_center_state == 1):
#                 print("Going Straight!")
#                 motor.setvel(vel_nom, 0)
#             else:
#                 print("Turning Off!")
#                 motor.setvel(0,0)
#                 break
#                 
#     except:
#         motor.setvel(0,0)
        
    # Problem 4
#     try:
#         motor.setvel(vel_nom, 0)
#         while True:
#             # Reading IR Sensors
#             ir_left_state = motor.io.read(IR_LEFT)
#             ir_center_state = motor.io.read(IR_CENTER)
#             ir_right_state = motor.io.read(IR_RIGHT)
#             
#             print("left: " , ir_left_state)
#             print("center: " , ir_center_state)
#             print("right: " , ir_right_state)
#             
#             # Setting motor states
#             if (ir_left_state == 0 and ir_center_state == 1 and ir_right_state == 0): # centered
#                 print("centered!")
#                 motor.setvel(vel_nom, 0)
#                 # state = 'C'
#             elif (ir_left_state == 0 and ir_center_state == 1 and ir_right_state == 1): # slight left
#                 print("slight left!")
#                 motor.setvel(vel_nom, math.sin(rad_small)*vel_nom/0.125)
#                 state = 'L'
#             elif (ir_left_state == 0 and ir_center_state == 0 and ir_right_state == 1): # more left
#                 print("more left!")
#                 motor.setvel(vel_nom, math.sin(rad_large)*vel_nom/0.125)
#                 state = 'L'
#             elif (ir_left_state == 0 and ir_center_state == 0 and ir_right_state == 0): # 3 cases
#                 if (state == 'L'): # complete left
#                     print("complete left!")
#                     motor.setspin(300)
#                     state = 'L'
#                 elif (state == 'R'): # complete right
#                     print("complete right!")
#                     motor.setspin(-1*300)
#                     state = 'R'
#                 else: # past the end and centered
#                     print("past and centered!")
#                     motor.setvel(0, 0)
#                     # state = 'C'
#                     break
#             elif (ir_left_state == 1 and ir_center_state == 1 and ir_right_state == 0): # slight right
#                 print("slight right!")
#                 state = 'R'
#                 motor.setvel(vel_nom, -1*math.sin(rad_small)*vel_nom/0.125)
#             elif (ir_left_state == 1 and ir_center_state == 0 and ir_right_state == 0): # more right
#                 print("more right!")
#                 state = 'R'
#                 motor.setvel(vel_nom, -1*math.sin(rad_large)*vel_nom/0.125)
#             else: # 2 special cases 111 and 101 where you can do anything
#                 print("Currently in a turning state. Continuing what I was doing.")
#                 
#     except:
#         motor.setvel(0,0)
        
    # Problem 5 and 6
#     try:
#         motor.setvel(vel_nom, 0)
#         while True:
#             # Reading IR Sensors
#             ir_left_state = motor.io.read(IR_LEFT)
#             ir_center_state = motor.io.read(IR_CENTER)
#             ir_right_state = motor.io.read(IR_RIGHT)
#             
#             if (searching):
#                 motor.setvel(vel_nom, 0)
#                 if (ir_left_state == 1 or ir_right_state == 1 or ir_center_state == 1):
#                     searching = False
#                 continue
#             
#             print("left: " , ir_left_state)
#             print("center: " , ir_center_state)
#             print("right: " , ir_right_state)
#             
#             # Setting motor states
#             if (ir_left_state == 0 and ir_center_state == 1 and ir_right_state == 0): # centered
#                 print("centered!")
#                 motor.setvel(vel_nom, 0)
#                 # state = 'C'
#             elif (ir_left_state == 0 and ir_center_state == 1 and ir_right_state == 1): # slight left
#                 print("slight left!")
#                 motor.setvel(vel_nom, math.sin(rad_small)*vel_nom/0.125)
#                 state = 'L'
#             elif (ir_left_state == 0 and ir_center_state == 0 and ir_right_state == 1): # more left
#                 print("more left!")
#                 motor.setvel(vel_nom, math.sin(rad_large)*vel_nom/0.125)
#                 state = 'L'
#             elif (ir_left_state == 0 and ir_center_state == 0 and ir_right_state == 0): # 3 cases
#                 if (state == 'L'): # complete left
#                     print("complete left!")
#                     # motor.setvel(vel_nom, math.sin(rad_large)*vel_nom/0.125)
#                     motor.setspin(400)
#                     state = 'L'
#                 elif (state == 'R'): # complete right
#                     print("complete right!")
#                     # motor.setvel(vel_nom, -1*math.sin(rad_large)*vel_nom/0.125)
#                     motor.setspin(-1*400)
#                     state = 'R'
#                 else: # past the end and centered
#                     print("past and centered!")
#                     # motor.setvel(0, 0)
#                     motor.setspin(400)
#                     # state = 'C'
#                     break
#             elif (ir_left_state == 1 and ir_center_state == 1 and ir_right_state == 0): # slight right
#                 print("slight right!")
#                 state = 'R'
#                 motor.setvel(vel_nom, -1*math.sin(rad_small)*vel_nom/0.125)
#             elif (ir_left_state == 1 and ir_center_state == 0 and ir_right_state == 0): # more right
#                 print("more right!")
#                 state = 'R'
#                 motor.setvel(vel_nom, -1*math.sin(rad_large)*vel_nom/0.125)
#             else: # 2 special cases 111 and 101 where you can do anything
#                 print("Currently in a turning state. Continuing what I was doing.")
#                 # motor.setvel(vel_nom,0)
#                 
#     except:
#         motor.setvel(0,0)

# Problem 7
#     try:
#         angularspeed = 0.9
#         lost_counter = 0
#         # motor.setvel(vel_nom, 0)
#         while True:
#             # Reading IR Sensors
#             ir_left_state = motor.io.read(IR_LEFT)
#             ir_center_state = motor.io.read(IR_CENTER)
#             ir_right_state = motor.io.read(IR_RIGHT)
#             
#             if (lost_counter >= 200): #detects if the robot is lost after set number of cycles
#                 searching = True
#                 lost_counter = 0
#             
#             if (searching):
#                 print("lost")
#                 if angularspeed > 0.0001: # must maintain an angular speed greater than 0 to keep circling
#                     angularspeed -= 0.0001
#                 motor.setvel(0.3, angularspeed) 
#                 if (ir_left_state == 1 or ir_right_state == 1 or ir_center_state == 1):
#                     searching = False
#                 continue
#             
#             print("left: " , ir_left_state)
#             print("center: " , ir_center_state)
#             print("right: " , ir_right_state)
#             
#             # Setting motor states
#             if (ir_left_state == 0 and ir_center_state == 1 and ir_right_state == 0): # centered
#                 print("centered!")
#                 motor.setvel(vel_nom, 0)
#                 lost_counter = 0
#                 state = 'C'
#             elif (ir_left_state == 0 and ir_center_state == 1 and ir_right_state == 1): # slight left
#                 print("slight left!")
#                 motor.setvel(vel_nom, math.sin(rad_small)*vel_nom/0.125)
#                 lost_counter = 0
#                 state = 'L'
#             elif (ir_left_state == 0 and ir_center_state == 0 and ir_right_state == 1): # more left
#                 print("more left!")
#                 motor.setvel(vel_nom, math.sin(rad_large)*vel_nom/0.125)
#                 lost_counter = 0
#                 state = 'L'
#             elif (ir_left_state == 0 and ir_center_state == 0 and ir_right_state == 0): # 3 cases
#                 lost_counter += 1
#                 if (state == 'L'): # complete left
#                     print("complete left!")
#                     # motor.setvel(vel_nom, math.sin(rad_large)*vel_nom/0.125)
#                     motor.setspin(400)
#                     state = 'L'
#                 elif (state == 'R'): # complete right
#                     print("complete right!")
#                     # motor.setvel(vel_nom, -1*math.sin(rad_large)*vel_nom/0.125)
#                     motor.setspin(-1*400)
#                     state = 'R'
#                 else: # past the end and centered
#                     print("past and centered!")
#                     # motor.setvel(0, 0)
#                     motor.setspin(400)
#                     # state = 'C'
#             elif (ir_left_state == 1 and ir_center_state == 1 and ir_right_state == 0): # slight right
#                 print("slight right!")
#                 lost_counter = 0
#                 state = 'R'
#                 motor.setvel(vel_nom, -1*math.sin(rad_small)*vel_nom/0.125)
#             elif (ir_left_state == 1 and ir_center_state == 0 and ir_right_state == 0): # more right
#                 print("more right!")
#                 lost_counter = 0
#                 state = 'R'
#                 motor.setvel(vel_nom, -1*math.sin(rad_large)*vel_nom/0.125)
#             else: # 2 special cases 111 and 101 where you can do anything
#                 print("Currently in a turning state. Continuing what I was doing.")
#                 # motor.setvel(vel_nom,0)
#                 
#     except:
#         motor.setvel(0,0)
    
    # Problem Extra Credit
    try:
        angularspeed = 0.9
        lost_counter = 0
        CW = 1 # will turn clockwise
        # motor.setvel(vel_nom, 0)
        while True:
            # Reading IR Sensors
            ir_left_state = motor.io.read(IR_LEFT)
            ir_center_state = motor.io.read(IR_CENTER)
            ir_right_state = motor.io.read(IR_RIGHT)
            
            if (lost_counter >= 100): #detects if the robot is lost after set number of cycles
                searching = True
                lost_counter = 0
                CW = 1
            
            if (searching):
                print("lost")
                if angularspeed > 0.0001: # must maintain an angular speed greater than 0 to keep circling
                    angularspeed -= 0.0001
                motor.setvel(0.3, angularspeed) 
                if (ir_left_state == 1 or ir_right_state == 1 or ir_center_state == 1):
                    searching = False
                continue
            
            print("left: " , ir_left_state)
            print("center: " , ir_center_state)
            print("right: " , ir_right_state)
            
            # Setting motor states
            if (ir_left_state == 0 and ir_center_state == 1 and ir_right_state == 0): # centered
                print("centered!")
                motor.setvel(vel_nom, 0)
                lost_counter = 0
                state = 'C'
            elif (ir_left_state == 0 and ir_center_state == 1 and ir_right_state == 1): # slight left
                print("slight left!")
                motor.setvel(vel_nom, math.sin(rad_small)*vel_nom/0.125)
                lost_counter = 0
                state = 'L'
            elif (ir_left_state == 0 and ir_center_state == 0 and ir_right_state == 1): # more left
                print("more left!")
                motor.setvel(vel_nom, math.sin(rad_large)*vel_nom/0.125)
                lost_counter = 0
                state = 'L'
            elif (ir_left_state == 0 and ir_center_state == 0 and ir_right_state == 0): # 3 cases
                lost_counter += 1
                if (state == 'L'): # complete left
                    print("complete left!")
                    print("CW", CW)
                    CW = 1
                    # motor.setvel(vel_nom, math.sin(rad_large)*vel_nom/0.125)
                    motor.setspin(400*CW)
                    state = 'L'
                elif (state == 'R'): # complete right
                    print("complete right!")
                    print("CW", CW)
                    CW = -1
                    # motor.setvel(vel_nom, -1*math.sin(rad_large)*vel_nom/0.125)
                    motor.setspin(400*CW)
                    state = 'R'
                else: # past the end and centered
                    print("past and centered!")
                    motor.setspin(400*CW)
                    print("CW", CW)
                    #motor.setspin(400)
                    # state = 'C'
            elif (ir_left_state == 1 and ir_center_state == 1 and ir_right_state == 0): # slight right
                print("slight right!")
                lost_counter = 0
                state = 'R'
                motor.setvel(vel_nom, -1*math.sin(rad_small)*vel_nom/0.125)
            elif (ir_left_state == 1 and ir_center_state == 0 and ir_right_state == 0): # more right
                print("more right!")
                lost_counter = 0
                state = 'R'
                motor.setvel(vel_nom, -1*math.sin(rad_large)*vel_nom/0.125)
            else: # 2 special cases 111 and 101 where you can do anything
                print("Currently in a turning state. Continuing what I was doing.")
                # motor.setvel(vel_nom,0)
                
    except:
        motor.setvel(0,0)
    
    motor.shutdown()   