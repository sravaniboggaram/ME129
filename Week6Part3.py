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

ULTRA1_ECHO = 16
ULTRA2_ECHO = 20
ULTRA3_ECHO = 21
ULTRA1_TRIGGER = 13
ULTRA2_TRIGGER = 19
ULTRA3_TRIGGER = 26

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

turning = False
turning2 = False
rightturntime = 0.57

# Ultrasonic global variables
ultrastarttime = time.time()
left_object = False
front_object = False
right_object = False
stopdistance = 0.3

# New longitude/latitude value after a step in the given heading.

def stop_on_line(gpio, level, tick):
    global turning
    turning = False
    
def stop_on_line2(gpio, level, tick):
    global turning2
    turning2 = False
        
def rising_front(gpio, level, tick):
    global front_object
    global ultrastarttime
    global stopdistance
    dist = 343/2 * (time.time() - ultrastarttime)
    if dist < stopdistance:
        print("rising distance: ", dist)
        print("entered rising_front if statment")
        front_object = True
        
    # while front_object:
#         ultrastarttime = time.time()
#         time.sleep(0.1)

def falling_front(gpio, level, tick):
    global ultrastarttime
    global front_object
    global stopdistance
    dist = 343/2 * (time.time() - ultrastarttime)
    if dist > stopdistance:
        print("falling distance: ", dist)
        print("entered falling_front if statment")
        front_object = False
    
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

        self.io.set_mode(ULTRA1_TRIGGER, pigpio.OUTPUT)
        self.io.set_mode(ULTRA2_TRIGGER, pigpio.OUTPUT)
        self.io.set_mode(ULTRA3_TRIGGER, pigpio.OUTPUT)
        self.io.set_mode(ULTRA1_ECHO, pigpio.INPUT)
        self.io.set_mode(ULTRA2_ECHO, pigpio.INPUT)
        self.io.set_mode(ULTRA2_ECHO, pigpio.INPUT)
        
        # double check left pins
        # cbriseultraleft = self.io.callback(ULTRA1_ECHO, pigpio.RISING_EDGE, rising_left)
        # cbfallultraleft = self.io.callback(ULTRA1_ECHO, pigpio.FALLING_EDGE, falling_left)

        cbriseultrafront = self.io.callback(ULTRA2_ECHO, pigpio.RISING_EDGE, rising_front)
        cbfallultrafront = self.io.callback(ULTRA2_ECHO, pigpio.FALLING_EDGE, falling_front)

        # cbriseultraright = self.io.callback(ULTRA3_ECHO, pigpio.RISING_EDGE, rising_right)
        # cbfallultraright = self.io.callback(ULTRA3_ECHO, pigpio.FALLING_EDGE, falling_right)

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
#         self.set(speed*slope1, speed*slope2)
        self.set(speed*slope1, speed*slope1)
        
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
#              self.set(v_outer*1/0.552776, v_inner*1/0.656167)
            self.set(v_outer*1/0.552776, v_inner*1/0.552776)

    def turn_90(self):
        starttime = time.time()
        global turning
        global rightturntime
        slope = 1/469.19
        self.set(380*slope, -380*slope)
        turning = True
        while time.time()-starttime < rightturntime and turning == True:
            continue
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
        paths_reorder = [paths[3],paths[2], paths[1], paths[0]]
        return paths_reorder
                
    def stupidlinefollow(self):
        vel_nom = 0.4
        rad_small = 20*math.pi/180
        rad_large = 40*math.pi/180
        exitcond = True
        state = 'C'
        while exitcond:
            # Reading IR Sensors
            ir_left_state = self.io.read(IR_LEFT)
            ir_center_state = self.io.read(IR_CENTER)
            ir_right_state = self.io.read(IR_RIGHT)
            
            # Setting motor states
            if (ir_left_state == 0 and ir_center_state == 1 and ir_right_state == 0): # centered
                self.setvel(vel_nom, 0)
                if state == 'C':
                    exitcond = False
                state = 'C'
            elif (ir_left_state == 0 and ir_center_state == 1 and ir_right_state == 1): # slight left
                self.setvel(vel_nom, math.sin(rad_large)*vel_nom/0.125)
                state = 'L'
            elif (ir_left_state == 0 and ir_center_state == 0 and ir_right_state == 1): # more left
                self.setvel(vel_nom, math.sin(rad_large)*vel_nom/0.125)
                state = 'L'
            elif (ir_left_state == 0 and ir_center_state == 0 and ir_right_state == 0): # 3 cases
                if (state == 'L'): # complete left
                    self.setspin(400)
                    state = 'L'
                elif (state == 'R'): # complete right
                    self.setspin(-1*400)
                    state = 'R'
                else: # past the end and centered
                    self.setspin(400)
                    # state = 'C'
            elif (ir_left_state == 1 and ir_center_state == 1 and ir_right_state == 0): # slight right
                state = 'R'
                self.setvel(vel_nom, -1*math.sin(rad_large)*vel_nom/0.125)
            elif (ir_left_state == 1 and ir_center_state == 0 and ir_right_state == 0): # more right
                state = 'R'
                self.setvel(vel_nom, -1*math.sin(rad_large)*vel_nom/0.125)
            elif (ir_left_state == 1 and ir_center_state == 1 and ir_right_state == 1):
                time.sleep(0.01)
    
    
    def linefollow(self):
        global front_object
        global ultrastarttime
        vel_nom = 0.4
        rad_small = 20*math.pi/180
        rad_large = 40*math.pi/180
        state = 'C'
        searching = True
        angularspeed = 0.9
        lost_counter = 0
        exitcond = False
        lasttime = time.time()

        try:
            
#             while not exitcond:
#                 while time.time()-lasttime > 0.1:
#                     while not front_object:
                        # if statement of how much time has passed (100 ms)
                            # update lasttime
                        
                        # trigger 
            
            while not exitcond and not front_object:
                if time.time() - lasttime > 0.1:
                    # self.io.gpio_trigger(ULTRA1_TRIGGER, 10, 1)
                    self.io.gpio_trigger(ULTRA2_TRIGGER, 10, 1)
                    # self.io,gpio_trigger(ULTRA3_TRIGGER, 10, 1)
                    lasttime = time.time()
                    ultrastarttime = time.time()
                # Reading IR Sensors
                ir_left_state = self.io.read(IR_LEFT)
                ir_center_state = self.io.read(IR_CENTER)
                ir_right_state = self.io.read(IR_RIGHT)              
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
                    self.setlinear(vel_nom)
                    time.sleep(0.42)
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
        global long
        global lat
        print("heading back")
        for i in range(len(intersections)-1, 0,-1):
            motor.spintonextline((intersections[i].headingToTarget - heading)%4)
            heading = intersections[i].headingToTarget
            motor.linefollow()
#         motor.spintonextline((SOUTH-heading)%4)
#         heading = SOUTH
#         motor.linefollow()
        motor.spintonextline((NORTH - heading)%4)
        long = 0
        lat = 0
        heading = NORTH
            
    def heading_to_target():
        global heading
        global lat
        global long
        current_intersection = intersection(long,lat)
        while current_intersection.headingToTarget != None:
            motor.spintonextline((current_intersection.headingToTarget - heading)%4)
            heading = current_intersection.headingToTarget
            motor.linefollow()
            (long, lat) = shift(long, lat, heading)
            current_intersection = intersection(long,lat)
        motor.setvel(0,0)
        time.sleep(0.2)
        
    def cycle_deadends():
        print("enter deadend")
        global intersections
        global long
        global lat
        
        deadends = []
        
        for i in intersections:
            nostreets = 0
            connecteds = 0
            for street in i.streets:
                if street == NOSTREET:
                    nostreets += 1
                elif street == CONNECTED:
                    connecteds += 1
            if connecteds == 1 and nostreets == 3:
                deadends.append(i)
        
        for i in range(len(deadends)):
            djikstra(intersection(long,lat),deadends[i])
            heading_to_target()
            
    
    def trackmap():
        global heading
        global long
        global lat
        global lastintersection
        global intersections
        
        motor.linefollow()
        while front_object:
            motor.setvel(0,0)
            time.sleep(5)
            motor.linefollow()
        (long, lat) = shift(long, lat, heading)
        if intersection(long, lat) == None:
            print("checking intersection")
            inter = Intersection(long, lat)
            intersections.append(inter) # append
            paths = motor.spincheck()
            inter.streets = convertabsolute(paths)
            if lastintersection != None:
                inter.headingToTarget = (heading+2)%4
        if lastintersection != None:
            inter = intersection(long,lat)
#             if lastintersection.headingToTarget == heading and inter.headingToTarget == None:
#                 i = inter.streets.index(CONNECTED)
#                 if i == (heading+2)%4: # will be the case and cause a loop where two headingToTarget's point at each other
#                     i = inter.streets.index(CONNECTED, i+1, 3)
#                 inter.headingToTarget == i
            if inter.headingToTarget == None:
                inter.headingToTarget = (heading+2)%4
            lastintersection.streets[heading] = CONNECTED
            inter.streets[(heading+2)%4] = CONNECTED
        k = random.randint(0,len(intersection(long,lat).streets)-1)
        allc = True
        for i in range(0,len(intersection(long,lat).streets)):
            if intersection(long,lat).streets[i] == UNEXPLORED or intersection(long,lat).streets[i] == UNKNOWN:
                allc = False
                break
        if allc:
            # k = intersection(long,lat).headingToTarget
            k = random.randint(0,3)
            while intersection(long,lat).streets[k] == NOSTREET:
                k = random.randint(0,3)
        else:
            for i in range(0,len(intersection(long,lat).streets)):
                if intersection(long,lat).streets[i] == UNEXPLORED:
                    k = i
                    break
            while intersection(long,lat).streets[k] == NOSTREET: # All streets are either NOSTREET or CONNECTED
                k = random.randint(0,len(intersection(long,lat).streets)-1) # Must be a CONNECTED street
        # Labeling deadends
#         nostreets = 0
#         connecteds = 0
#         print("inter.streets: ", inter.streets)
#         for street in inter.streets:
#             if street == NOSTREET:
#                 nostreets += 1
#             elif street == CONNECTED:
#                 connecteds += 1
#         print("nostreets: ",nostreets, " connecteds: ", connecteds)
#         if connecteds == 1 and nostreets == 3:
#             deadends.append(inter)
        motor.spintonextline((k-heading)%4)
        heading = k
        lastintersection = intersection(long,lat)
            
    def djikstra(start, goal):
        global intersections
        to_be_processed = []
        print("entered dykstra")
        for inter in intersections:
            inter.headingToTarget = None
        to_be_processed.append(goal)
        temp_target = to_be_processed.pop(0)
        stop_cond = False
        while temp_target.lat != start.lat or temp_target.long != start.long:
            for i in range(0,len(temp_target.streets)):
                # getting neighbors
                if i == 0:
                    checking_intersection = intersection(temp_target.long,temp_target.lat+1)
                elif i == 1:
                    checking_intersection = intersection(temp_target.long-1,temp_target.lat)
                elif i == 2:
                    checking_intersection = intersection(temp_target.long,temp_target.lat-1)
                elif i == 3:
                    checking_intersection = intersection(temp_target.long+1,temp_target.lat)    
    
                if temp_target.streets[i] == CONNECTED and checking_intersection.headingToTarget == None:
                    checking_intersection.headingToTarget = (i+2)%4 # point toward temp_target
                    if checking_intersection.long == goal.long and checking_intersection.lat == goal.lat:
                        checking_intersection.headingToTarget = None
                    if checking_intersection.long == start.long and checking_intersection.lat == start.lat:
                        # checking_intersection.headingToTarget = None
                        to_be_processed.append(checking_intersection)
                        stop_cond = True
                        break
                    else:
                        to_be_processed.append(checking_intersection)
            if stop_cond:
                break
            if len(to_be_processed) != 0:
                temp_target = to_be_processed.pop(0)
                
    try:

        # Spiral to find the line to "kick off the session" -gunter
        searching = True
        angularspeed = 0.9
        
        while searching == True:
            ir_left_state = motor.io.read(IR_LEFT)
            ir_center_state = motor.io.read(IR_CENTER)
            ir_right_state = motor.io.read(IR_RIGHT)
            if (searching):
                if angularspeed > 0.0001: # must maintain an angular speed greater than 0 to keep circling
                    angularspeed -= 0.0000001
                motor.setvel(0.3, angularspeed) 
                if (ir_left_state == 1 or ir_right_state == 1 or ir_center_state == 1):
                    searching = False
                continue
        motor.stupidlinefollow()
         
        
        allConnected = False
        trackmap()
        while allConnected == False:
            allConnected = True
            for i in intersections:
                for s in i.streets:
                    if s == UNEXPLORED or s == UNKNOWN:
                        allConnected = False
            trackmap()
            print(intersections)
        
#         prompt user to enter spiral
#         goal_long = input("longitude: ")
#         goal_lat = input("latitude: ")
#         time.sleep(3)
#         djikstra(intersection(0,0),intersection(int(goal_long), int(goal_lat)))
#         heading_to_target()

        time.sleep(3)
        cycle_deadends()
        motor.shutdown()
    except KeyboardInterrupt:
        motor.setvel(0,0)
    