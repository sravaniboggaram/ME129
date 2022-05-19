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
import statistics
import threading
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
front_object = False

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

class Ultrasonic:
        
    def __init__(self, triggerpin, echopin):
        
        self.io = pigpio.pi()
        if not self.io.connected:
            print("Unable to connection to pigpio daemon!")
            sys.exit(0)
        
        self.triggerpin = triggerpin
        self.echopin = echopin
        self.start_time = 0
        self.object_present = False
        self.stop_distance = 0.3
        self.distance = 0
        self.stopflag = False
        
        self.io.set_mode(triggerpin, pigpio.OUTPUT)
        self.io.set_mode(echopin, pigpio.INPUT)
        
        cbrise = self.io.callback(echopin, pigpio.RISING_EDGE, self.rising)
        cbfall = self.io.callback(echopin, pigpio.FALLING_EDGE, self.falling)

    def rising(self, gpio, level, tick):
        self.start_time = tick
    
    def falling(self, gpio, level, tick):
        self.distance = 343/2 * (tick - self.start_time) * (10**-6)
        if self.distance > self.stop_distance:
            self.object_present = False
        if self.distance < self.stop_distance:
            self.object_present = True
    
    def trigger(self):
        self.io.write(self.triggerpin, 1)
        time.sleep(0.00001)
        self.io.write(self.triggerpin, 0)
        
    def stopcontinual(self):
        self.stopflag = True
        
    def runcontinual(self):
        self.stopflag = False
        while not self.stopflag:
            self.trigger()
            time.sleep(0.05 + 0.01*random.random())
        

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
#         self.set(speed*slope1, speed*slope2)
        self.set(speed*slope1, speed*slope1)
        
    def setspin(self, speed):
        """
        Speed is given in degrees per second.
        positive is clowckwise, negative is ccw
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
    
    def wall_follow(self, u):
#         PWM_left = max(0.5, min(0.9,0.7-u))
#         PWM_right = max(0.5, min(0.9, 0.7+u))
        PWM_left = max(0.5, min(0.9,0.7-u))
        PWM_right = max(0.5, min(0.9, 0.7+u))
        self.set(PWM_left, PWM_right)
        time.sleep(0.1)
    
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
    left_us = Ultrasonic(ULTRA1_TRIGGER, ULTRA1_ECHO)
    center_us = Ultrasonic(ULTRA2_TRIGGER, ULTRA2_ECHO)
    right_us = Ultrasonic(ULTRA3_TRIGGER, ULTRA3_ECHO)
#     stopflag = False
    
#     def stopcontinual():
#         stopflag = True
#         
#     def runcontinual(self):
#         print("entered runcontinual")
#         stopflag = False
#         while not stopflag:
#             center_us.trigger()
#             left_us.trigger()
#             right_us.trigger()
#             time.sleep(0.8 + 0.4*random.random())
            
        
    
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
            linefollow(motor, center_us)
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
            linefollow(motor, center_us)
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
    
    def ultraturn(motor, l, c, r):
        """
        motor, a motor object
        l, left ultrasonic object
        c, center ultrasonic object
        r, right ultrasonic object
        """
        
        vel_nom = 0.45
        rad_small = 20*math.pi/180
        rad_large = 40*math.pi/180
        try:
            while True:
                # Reading IR Sensors
                if (not l.object_present and c.object_present and not r.object_present): # 010
                    motor.setvel(-1*vel_nom, 0)
                elif (not l.object_present and c.object_present and r.object_present): # 011
                    motor.setspin(-300)
                elif (not l.object_present and not c.object_present and r.object_present): # 001
                    motor.setspin(-300)
                elif (l.object_present and c.object_present and not r.object_present): # 110
                    motor.setspin(300)
                elif (l.object_present and not c.object_present and not r.object_present): # 100
                    motor.setspin(300)
                elif (not l.object_present and not c.object_present and not r.object_present): # 000
                    motor.setvel(vel_nom, 0)
                elif (l.object_present and c.object_present and r.object_present): # 111
                    motor.setspin(300)
                else: # 101
                    motor.setvel(vel_nom, 0)
        except BaseException as ex:
                print("Ending due to exception: %s" % repr(ex))
                
    def linefollow(motor, ultra):
        vel_nom = 0.4
        rad_small = 20*math.pi/180
        rad_large = 40*math.pi/180
        state = 'C'
        exitcond = False
        try:
            while not exitcond:
                # Reading IR Sensors
                ir_left_state = motor.io.read(IR_LEFT)
                ir_center_state = motor.io.read(IR_CENTER)
                ir_right_state = motor.io.read(IR_RIGHT)
                print(ultra.distance)
                # Setting motor states
                print(ultra.object_present)
                if ultra.object_present:
                    motor.setvel(0,0)
                elif (ir_left_state == 0 and ir_center_state == 1 and ir_right_state == 0): # centered
                    motor.setvel(vel_nom, 0)
                    lost_counter = 0
                    state = 'C'
                elif (ir_left_state == 0 and ir_center_state == 1 and ir_right_state == 1): # slight left
                    motor.setvel(vel_nom, math.sin(rad_small)*vel_nom/0.125)
                    lost_counter = 0
                    state = 'L'
                elif (ir_left_state == 0 and ir_center_state == 0 and ir_right_state == 1): # more left
                    motor.setvel(vel_nom, math.sin(rad_large)*vel_nom/0.125)
                    lost_counter = 0
                    state = 'L'
                elif (ir_left_state == 1 and ir_center_state == 1 and ir_right_state == 0): # slight right
                    lost_counter = 0
                    state = 'R'
                    motor.setvel(vel_nom, -1*math.sin(rad_small)*vel_nom/0.125)
                elif (ir_left_state == 1 and ir_center_state == 0 and ir_right_state == 0): # more right
                    lost_counter = 0
                    state = 'R'
                    motor.setvel(vel_nom, -1*math.sin(rad_large)*vel_nom/0.125)
                elif (ir_left_state == 0 and ir_center_state == 0 and ir_right_state == 0): # passed end or pushed off
                    motor.setvel(0,0)
                    exitcond = True
                elif (ir_left_state == 1 and ir_center_state == 1 and ir_right_state == 1): # seeing intersection
                    motor.setlinear(vel_nom)
                    time.sleep(0.42)
                    motor.setvel(0,0)
                    exitcond = True
        except BaseException as ex:
                print("Ending due to exception: %s" % repr(ex)) 
    
    def trackmap():
        global heading
        global long
        global lat
        global lastintersection
        global intersections
        lasttime = time.time()
        global ultrastarttime
        
        linefollow(motor, center_us)
        (long, lat) = shift(long, lat, heading)
        if intersection(long, lat) == None:
            inter = Intersection(long, lat)
            intersections.append(inter) # append
            paths = motor.spincheck()
            inter.streets = convertabsolute(paths)
            if lastintersection != None:
                inter.headingToTarget = (heading+2)%4
        if lastintersection != None:
            inter = intersection(long,lat)
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
    
#     thread = threading.Thread(target=runcontinual)
    thread_left = threading.Thread(target=left_us.runcontinual)
    thread_center = threading.Thread(target=center_us.runcontinual)
    thread_right = threading.Thread(target=right_us.runcontinual)
    thread_left.start()
    thread_center.start()
    thread_right.start()
    k = 0.5
    # k = 3
    # k = 0.03
    
    try:
#         center_us.trigger()
#         while True:      
#             while right_us.distance < 1:
#                 while center_us.distance < 0.2:
#                     print("turning")
#                     while center_us.object_present:
#                         # ultraturn(motor, left_us, center_us, right_us)
#                         motor.setspin(-300)
#                     time.sleep(0.6)
#                 e = right_us.distance - right_us.stop_distance
#                 motor.wall_follow(-1*k*e)
#             while not right_us.object_present:
#                 ultraturn(motor, left_us, center_us, right_us)
                #motor.setvel(0.4,0.4/(right_us.stop_distance - 0.1))

        # herding, problem 6
        # ultraturn(motor, left_us, center_us, right_us)
        
        # wall follow problem 7
        # while right_us.distance < 1:
        while True:
            e = right_us.distance - right_us.stop_distance
            motor.wall_follow(-1*k*e)
        motor.setvel(0.4,0.4/right_us.stop_distance)
        
            

            #print("dist: ", [left_us.distance, center_us.distacnce, right_us.distance])
        
    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))
        
    except KeyboardInterrupt:
        motor.setvel(0,0)
        
#     stopcontinual()
    left_us.stopcontinual()
    center_us.stopcontinual()
    right_us.stopcontinual()
    thread_left.join()
    thread_center.join()
    thread_right.join()
     
    motor.shutdown()