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
import random
# Define the motor pins.
MTR1_LEGA = 7   #right
MTR1_LEGB = 8
MTR2_LEGA = 5   #left
MTR2_LEGB = 6
IR_LEFT = 14 #left IR sensor
IR_CENTER = 15 #center IR sensor
IR_RIGHT = 18 #right IR sensor

# Global Constants:
# Headings
NORTH = 0
WEST = 1
SOUTH = 2
EAST = 3
HEADING = {NORTH:'North', WEST:'West', SOUTH:'South', EAST:'East', None: 'None'} # For printing

# Street status
UNKNOWN = 'Unknown'
NOSTREET = 'NoStreet'
UNEXPLORED = 'Unexplored'
CONNECTED = 'Connected'

# Global Variables:
intersections = [] # list of intersections
lastintersection = None # last intersection visited
long = 0 # current east/west coordinate
lat = -1 # current north/south coordinate
heading = NORTH # current heading

# Kevin's stuff
turning = False
turning2 = False
rightturntime = 0.6

# New longitude/latitude value after a step in the given heading.

def stop_on_line(gpio, level, tick):
    global turning
    turning = False
    
def stop_on_line2(gpio, level, tick):
    global turning2
    turning2 = False
        
    
def shift(long, lat, heading):
    if heading % 4 == NORTH:
        return (long, lat+1)
    elif heading % 4 == WEST:
        return (long-1, lat)
    elif heading %4 == SOUTH:
        return (long, lat-1)
    elif heading % 4 == EAST:
        return (long+1, lat)
    else:
        raise Exception("This can't be")
    
# Find the intersection
def intersection(long, lat):
    list = [i for i in intersections if i.long == long and i.lat == lat]
    if len(list) == 0:
        return None
    if len(list) > 1:
        raise Exception("Multiple intersections at (%2d, %2d)" % (long, lat))
    return list[0]

class Intersection:
    # Initialize - create new intersection at (long, lat)
    def __init__(self, long, lat):
        # save the parameters
        self.long = long
        self.lat = lat
        # status of streets at the intersection, in NWSE dirdctions.
        self.streets = [UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN]
        # direction to head from this intersection in planned move.
        self.headingToTarget = None
        
        # add this to the global lsit of intersections to make it searchable
        global intersections
        if intersection(long, lat) is not None:
            raise Exception("Duplicate intersection at (%2d,%2d)" % (long,lat))
        # intersections.append(self)
    
    # Print format.
    def __repr__(self):
        return ("(%2d, %2d) N:%s W:%s S:%s E:%s - head %s\n" %
                (self.long, self.lat, self.streets[0],
                self.streets[1], self.streets[2], self.streets[3],
                HEADING[self.headingToTarget]))
        

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
        
        cbrise = self.io.callback(IR_CENTER, pigpio.RISING_EDGE, stop_on_line)
        cbrise2 = self.io.callback(IR_CENTER, pigpio.RISING_EDGE, stop_on_line2)


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
    
    def turn_90(self):
        starttime = time.time()
        global turning
        global rightturntime
        slope = 1/469.19
        self.set(380*slope, -380*slope)
        turning = True
        while time.time()-starttime < rightturntime and turning == True:
            continue
        # heading = ??
        self.setvel(0,0)
        time.sleep(0.2)
        if not turning:
            return True
        else:
            return False
        
    
    def spintonextline(self,dir):
        global rightturntime
        global turning2
        slope = 1/469.19
#         self.set(470*slope, -470*slope) # overcome motor stickiness
#         time.sleep(0.1)

        if dir == 3:
            dir = -1
        elif dir == -3:
            dir = 1
        
        for i in range(abs(dir)):
            starttime = time.time()
            turning2 = True
            while turning2:
                if dir > 0:
                    self.set(-380*slope, 380*slope)
                elif dir < 0:
                    self.set(380*slope, -380*slope)
                ir_center_state = self.io.read(IR_CENTER)
                if (time.time() - starttime > rightturntime+0.05):
                    turning2 = False
            
        self.setvel(0,0)
        time.sleep(0.2)

#     def spincheck(self):
#         """
#         Speed is given in degrees per second.
#         Turning left is denoted by 1 or -3
#         Turning rihgt is denoted by 3 or -1
#         Turning backward is denoted by 2 or -2
#         No turn is denoted by a 0
#         A positive number corresponds to a left spin and a negative number corresponds to a right spin
#         Note that the shorter turning path is chosen, so a 3 corresponds to a -1 and a -3 corresponds to a 1
#         """
#         slope = 1/469.19
#         self.set(470*slope, -470*slope) # overcome motor stickiness
#         time.sleep(0.01)
#         
#         #start a timer
#         starttime = time.time()
#         time_full_rot = 0.58
#         time_360 = 0
#         counter = 0
#         intersectiontimes = []
#         last_center_reading = 0
#         while(time_360 < time_full_rot):
#             self.set(380*slope, -380*slope) # spin for 360 degrees
#             ir_left_state = self.io.read(IR_LEFT)
#             ir_center_state = self.io.read(IR_CENTER)
#             ir_right_state = self.io.read(IR_RIGHT)
#             if ir_center_state == 1:
#                 if last_center_reading == 0:
#                     counter += 1
#                     time_enter = time.time()
#                 last_center_reading = 1
#             elif ir_center_state == 0:
#                 if last_center_reading == 1:
#                     counter += 1
#                     time_exit = time.time()
#                     intersectiontimes.append(((time_exit + time_enter)/2) - starttime)
#                 last_center_reading = 0
#             time_360 = time.time() - starttime
#             
#         self.setvel(0,0)
#         time.sleep(0.5)
#         
#         # want [forward, left, backward, right]
#         # reads [forward, right, back, left]

#         print(intersectiontimes)
#         paths = [False, False, False, False]
#         for t in intersectiontimes:
#             inter_num = round(4*t/time_full_rot)
#             paths[inter_num] = True
#         print("paths_2 ", paths)
#         paths_reorder = [paths[0]]
#         paths_reorder.extend(paths[len(paths):0:-1])
#         return paths_reorder

    def spincheck(self):
        """
        Speed is given in degrees per second.
        Turning left is denoted by 1 or -3
        Turning rihgt is denoted by 3 or -1
        Turning backward is denoted by 2 or -2
        No turn is denoted by a 0
        A positive number corresponds to a left spin and a negative number corresponds to a right spin
        Note that the shorter turning path is chosen, so a 3 corresponds to a -1 and a -3 corresponds to a 1
        """
        # initial [right, back, left, forward]
        # reorder [forward, left, back, right]
        
        paths = []
        for i in range(4):
            pathexist = self.turn_90() # pathexist is a boolean representing whether that pathway exists
            paths.append(pathexist)     
        
#         slope = 1/469.19
#         self.set(470*slope, -470*slope) # overcome motor stickiness
#         time.sleep(0.01)
#         paths = []
#         
#         for i in range(4):
#             starttime = time.time()
#             self.set(380*slope, -380*slope)
#             counter = 0
#             last_center_reading = 1
#             time_90 = 0
#             time_rot = 0.565
#             while(time_90 < time_rot):
#                 ir_center_state = self.io.read(IR_CENTER)
#                 if ir_center_state == 1:
#                     if last_center_reading == 0:
#                         counter += 1
#                         paths.append(True)
#                         break
#                     last_center_reading = 1
#                 elif ir_center_state == 0:
#                     last_center_reading = 0
#                 time_90 = time.time() - starttime
#             self.setvel(0,0)
#             time.sleep(0.2)
#             if counter == 0:
#                 paths.append(False)
                
        
        # want [forward, left, backward, right]
        # reads [forward, right, back, left]
#         print(intersectiontimes)
#         paths = [False, False, False, False]
#         for t in intersectiontimes:
#             inter_num = round(4*t/time_full_rot)
#             paths[inter_num] = True\

        # Current reads in the order of [
        paths_reorder = [paths[3],paths[2], paths[1], paths[0]]
        # paths reorder = paths[::-1]
        return paths_reorder
                
        
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
                    self.setvel(vel_nom, 0)
                    lost_counter = 0
                    state = 'C'
                elif (ir_left_state == 0 and ir_center_state == 1 and ir_right_state == 1): # slight left
                    self.setvel(vel_nom, math.sin(rad_small)*vel_nom/0.125)
                    lost_counter = 0
                    state = 'L'
                elif (ir_left_state == 0 and ir_center_state == 0 and ir_right_state == 1): # more left
                    self.setvel(vel_nom, math.sin(rad_large)*vel_nom/0.125)
                    lost_counter = 0
                    state = 'L'
                elif (ir_left_state == 1 and ir_center_state == 1 and ir_right_state == 0): # slight right
                    lost_counter = 0
                    state = 'R'
                    self.setvel(vel_nom, -1*math.sin(rad_small)*vel_nom/0.125)
                elif (ir_left_state == 1 and ir_center_state == 0 and ir_right_state == 0): # more right
                    lost_counter = 0
                    state = 'R'
                    self.setvel(vel_nom, -1*math.sin(rad_large)*vel_nom/0.125)
                elif (ir_left_state == 0 and ir_center_state == 0 and ir_right_state == 0): # passed end or pushed off
                    self.setvel(0,0)
                    exitcond = True
                elif (ir_left_state == 1 and ir_center_state == 1 and ir_right_state == 1): # seeing intersection
                    time.sleep(0.53)
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
        

#
#   Main
#
if __name__ == "__main__":
    ############################################################
    
    motor = Motor()
    
    def convertabsolute(paths):
        # paths input is [Forward, Left, Backward, Right]
        # returns [NORTH, WEST, SOUTH, EAST]
        global heading
        convert ={True : UNEXPLORED, False: NOSTREET}
        if heading == NORTH:
            return [convert[paths[0]], convert[paths[1]], convert[paths[2]], convert[paths[3]]]
        elif heading == EAST:
            return [convert[paths[1]], convert[paths[2]], convert[paths[3]], convert[paths[0]]]
        elif heading == WEST:
            return [convert[paths[3]], convert[paths[0]], convert[paths[1]], convert[paths[2]]]
        elif heading == SOUTH:
            return [convert[paths[2]], convert[paths[3]], convert[paths[0]], convert[paths[1]]]
        
    def headback():
        global intersections
        global heading
        print("heading back")
        for i in range(len(intersections)-1, 0,-1):
            # print("heading: ", heading)
            motor.spintonextline((intersections[i].headingToTarget - heading)%4)
            heading = intersections[i].headingToTarget
            motor.linefollow()
        motor.spintonextline((SOUTH-heading)%4)
        heading = SOUTH
        motor.linefollow()
            
    def trackmap():
        motor.linefollow()
        global heading
        global long
        global lat
        global lastintersection
        (long, lat) = shift(long, lat, heading)
        print("Currently at (%2d, %2d)" % (long, lat))
        global intersections
        if intersection(long, lat) == None:
            inter = Intersection(long, lat)
            intersections.append(inter) # append
            paths = motor.spincheck()
            inter.streets = convertabsolute(paths)
        if lastintersection != None:
            inter = intersection(long,lat)
            lastintersection.streets[heading] = CONNECTED
            inter.streets[(heading+2)%4] = CONNECTED
            inter.headingToTarget = (heading+2)%4
            print("Current latitude: ", inter.lat, " and Current longitude: ", inter.long)
            print("headingToTarget: ", inter.headingToTarget)
            
#             k = 0
#             for i in range(0,len(inter.streets)):
#                 if inter.streets[i] == UNEXPLORED:
#                     k = i
#                     break
#             while inter.streets[k] == NOSTREET: # All streets are either NOSTREET or CONNECTED
#                 k = random.randint(0,len(inter.streets)-1) # Must be a CONNECTED street
#             motor.spintonextline((k-heading)%4)
#             heading = k
#             lastintersection = inter
        
        k = random.randint(0,len(intersection(long,lat).streets)-1)
        for i in range(0,len(intersection(long,lat).streets)):
            if intersection(long,lat).streets[i] == UNEXPLORED:
                k = i
                break
        while intersection(long,lat).streets[k] == NOSTREET: # All streets are either NOSTREET or CONNECTED
            k = random.randint(0,len(intersection(long,lat).streets)-1) # Must be a CONNECTED street
        motor.spintonextline((k-heading)%4)
        heading = k
        lastintersection = intersection(long,lat)
        print(intersection(long,lat).streets)

    try:
        allConnected = False
        while allConnected == False:
            allconnected = True
            for i in intersections:
                for s in i.streets:
                    if s == UNEXPLORED or s == UNKNOWN:
                        allconnected = False
                        print("This intersection still has unexplored or unknown:", i.long, i.lat)
                        print(i.streets.index(s))
            trackmap()
        headback()
        motor.shutdown()
    except KeyboardInterrupt:
        motor.setvel(0,0)
        print("KeyboardInterrupt")
        
