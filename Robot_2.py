
# Import constants
from termios import TIOCGLCKTRMIOS
from constants_2 import *
from Motor import *
from Ultrasonic import *
from Intersection import *

# Establish global variables
intersections = [] # list of intersections
lastintersection = None # last intersection visited
long = 0 # current east/west coordinate
lat = -1 # current north/south coordinate
heading = NORTH # current heading
turned_around = False

counter = 0

# turning boolean establishes whether the robot is currently turning.
turning = False
turning_starttick = 1000000000000
over_line = False
# A constant to measure the time it takes to turn right approximately 90
# degrees
rightturntime = 0.6
# rightturntime = 1
# A boolean representing whether a front object is currently present, based on
# the ultrasonic sensor
front_object = False

# max power: 470*SLOPE

class Infrared:
    """
    A class to represent all infrared sensors.
    Attributes
    ----------
    io : pigpio.io
        use a passed in pigpio.io object to setup IR pins
    cbrise : callback
        a callback that is activated by rising_edge of center IR
    Methods
    -------
    stop_on_line():
        Establish a callback function that enables the robot to stop once it
        has determined to have seen a line.
    readSensors():
        Reads the input of all infrared sensors.
    """

    def __init__(self, io):
        """
        Constructs all the necessary attributes for the infrared object.
        Parameters
        ----------
            io : pigpio.io
                pigpio.io object that contains pins, interface with Pi
        """
        self.io = io
        
        # Set up IR pins as input
        self.io.set_mode(IR_LEFT, pigpio.INPUT)
        self.io.set_mode(IR_CENTER, pigpio.INPUT)
        self.io.set_mode(IR_RIGHT, pigpio.INPUT)

        # Initiate callback functions for the IR sensors.
        self.cbrise = self.io.callback(IR_CENTER, pigpio.RISING_EDGE, \
            self.stop_on_line)
        
        # self.cbfall = self.io.callback(IR_CENTER, pigpio.FALLING_EDGE, self.falling)
    
    def stop_on_line(self, gpio, level, tick):
        """
        Establish a callback that enables the robot to stop once it has
        determined to have seen a line.
        """
        # Set global turning to False. Motors will then respond.
        global counter
        global turning_starttick

        counter += 1
        # print(counter, " Rising edge in the center detected!") 

        global turning
        global over_line

        # over_line = True
        turning = False
        # turning_starttick = tick

    # def falling(self, gpio, level, tick):
    #     global over_line
    #     over_line = False
        

    def readSensors(self):
        """
        Input:
            none
        Output:
            tuple of readings in the order left, center, right
        
        Read the infrared sensors.
        """
        left = self.io.read(IR_LEFT)
        center = self.io.read(IR_CENTER)
        right = self.io.read(IR_RIGHT)
        return (left, center, right)

class Robot:
    """
    A class to represent the robot and associated functions.
    Attributes
    ----------
    motor : Motor
        the Motor object used to control the propulsion and movement of
        the robot
    infrared : Infrared
        the Infrared object used to control callback, reading, and
        functionality of the infrared sensors
    left_us : Ultrasonic
        the Ultrasonic sensor corresponding to the detection of objects
        to the left of the robot
    front_us : Ultrasonic
        the Ultrasonic sensor corresponding to the detection of objects
        in front of the robot
    right_us : Ultrasonic
        the Ultrasonic sensor corresponding to the detection of objects
        to the right of the robot
    thread_left : Thread
        Thread object managing the continual trigger of left_us
    thread_center : Thread
        Thread object managing the continual trigger of center_us
    thread_right : Thread
        Thread object managing the continual trigger of right_us
    triggering_thread : Thread
        Thread object managing the contiual trigger of all three ultrasonic sensors combined.
    driving_thread : Thread
        Thread object managing whether the robot should continue driving.
    driving_stopflag : Boolean
        A stop condition for whether the robot should stop driving.
    driving_pause : Boolean
        A stop condition for whether the robot should pause driving at the next intersection.
    triggering_stopflag : Boolean
        A stop condition for whether the robot should stop triggering ultrasonic sensors.
    
    Methods
    -------
    shutdown():
        Shutdown the robot properly and clear all pins.
    turn_90():
        Turn 90 degrees based on two conditions, stopping either based on time
        or based on detecting a line.
    spintonextline(desired_heading):
        Spin to the next line a designated number of times, calculated by the int desired_heading. Turn in the direction
        with the shortest turn path.
    spincheck():
        Determines the paths available at an intersection when the robot arrives at a new intersection.
    stupidlinefollow():
        Engages in a simplified line follow program to allow spiral to work, from initial lost condition.
    ultraturn(left_us, center_us, right_us):
        Turns based on the ultrasonic sensor readings.
    linefollow(center_us):
        Follows a line until it reaches an intersection, but pauses when it detects an obstacle in front of the robot.
    wall_follow(u):
        Follows a wall that is detected to the right of the robot.
    convertabsolute(paths):
        Reorders paths so that they are in the correct, absolute order.
    headback():
        Heads back towards the starting point of the robot (0, 0).
    heading_to_target():
        Heads to the target based on headingToTarget of intersections. This
        is generally called after Djikstra's algorithm which resets and adds
        new values for each headingToTarget of each intersection.
    cycle_deadends():
        Cycles through the deadends found throughout the map.
    trackmap():
        Roams through the map discovering new intersections. Continues roaming
        until all intersections are discovered and the map is completely
        understood.
    djikstra(start, goal):
        Run the Djikstra algorithm to calculate the shortest route from the start position to the target position.
    shift(long, lat, heading):
        Shift the position of the robot in accordance with its current heading and direction driven.
    intersection(long, lat):
        Look for an intersection with the same longitude and latitude as that handed in. If an intersection is found,
        return that intersection object. If it is not found, return None.
    """
    def __init__(self):
        """
        Constructs all the necessary attributes for the Robot object.
        Parameters
        ----------
            io : pigpio.io
                    pigpio.io object that contains pins, interface with Pi           
        """
        # Initialize the connection to the pigpio daemon (GPIO interface).
        self.io = pigpio.pi()
        if not self.io.connected:
            print("Unable to connection to pigpio daemon!")
            sys.exit(0)
            
        self.motor = Motor(self.io)

        self.infrared = Infrared(self.io)

        self.left_us = Ultrasonic(ULTRA1_TRIGGER, ULTRA1_ECHO, self.io)
        self.center_us = Ultrasonic(ULTRA2_TRIGGER, ULTRA2_ECHO, self.io)
        self.right_us = Ultrasonic(ULTRA3_TRIGGER, ULTRA3_ECHO, self.io)

        # driving and triggering flags
        self.driving_stopflag = False
        self.driving_pause = False
        self.triggering_stopflag = False

        # create threads 
        # self.thread_left = threading.Thread(target=self.left_us.runcontinual)
        # self.thread_center = threading.Thread(target=self.center_us.runcontinual)
        # self.thread_right = threading.Thread(target=self.right_us.runcontinual)
        # self.thread_left.start()
        # self.thread_center.start()
        # self.thread_right.start()

        # Start two new/background threads.
        # self.triggering_thread = threading.Thread(target=self.triggering_loop)
        # self.triggering_thread.start()
        # self.driving_thread = threading.Thread(target=self.driving_loop)
        # self.driving_thread.start()

        print("GPIO ready...")


    def shutdown(self): # need to cancel callback functions
        """
        Input:
            none
        Output:
            none
        
        Shutdown the robot and clear all pins.
        """
        # Clear all pins, just in case.
        print("Turning off")
        self.io.set_PWM_dutycycle(MTR1_LEGA, 0)
        self.io.set_PWM_dutycycle(MTR1_LEGB, 0)
        self.io.set_PWM_dutycycle(MTR2_LEGA, 0)
        self.io.set_PWM_dutycycle(MTR2_LEGB, 0)

        self.left_us.stopcontinual()
        self.center_us.stopcontinual()
        self.right_us.stopcontinual()
        
        self.thread_left.join()
        self.thread_center.join()
        self.thread_right.join()

        # self.triggering_stop()
        # self.triggering_thread.join()
        # self.driving_stop()
        # self.driving_thread.join()

        self.io.stop()

    def turn_90(self): # spin less aggressively?
        """
        Inputs:
            none
        Outputs:
            boolean, True if the robot detects a street in the 90 degree turn
            and False if no street detected
        Turn 90 degrees based on two conditions, stopping either based on time
        or based on detecting a line.
        """
        starttime = time.time()
        global turning
        global rightturntime
        global over_line
        global SLOPE
        global turning_starttick

        # self.motor.set(380*SLOPE, -380*SLOPE)
        self.motor.set(470*SLOPE, -470*SLOPE)
        time.sleep(0.1)
        # self.motor.set(320*SLOPE, -320*SLOPE)
        self.motor.set(340*SLOPE, -340*SLOPE)

        turning = True
        # over_line = True
        # counter = 0
        # print("getting tick")
        # print("Ticks passed: ", self.io.get_current_tick()-turning_starttick)
        # while time.time()-starttime < rightturntime: # and self.io.get_current_tick()-turning_starttick < 100000 and not over_line:
        #     (left, center, right) = self.infrared.readSensors()
        #     if center == 1:
        #         counter += 1
        #     else:
        #         counter = 0
        #     if counter < 10:
        #         time.sleep(0.0005)
        #         continue
        #     else:
        #         break
        # while time.time()-starttime < rightturntime:
        #     # add an if statement in the while loop
        #     if time.time() - starttime > 0.100:
        #         if self.io.get_current_tick()-turning_starttick < 100000:
        #             continue
        #         else:
        #             break
        #     else:
        #         continue
        while time.time()-starttime < rightturntime and turning == True:
            continue
        self.motor.setvel(0,0)
        time.sleep(0.2)
        # self.center_us.trigger()
        print("turned 90 degrees")
        if not turning:
            if self.center_us.object_present:
                return BLOCKED
            return UNEXPLORED
        else:
            return NOSTREET


    def spintonextline(self,desired_heading):
        """
        Input:
            desired_heading, an int representing the desired_heading
        Output:
            none
        
        Spin to the next line a designated number of times, calculated by the int desired_heading. Turn in the direction
        with the shortest turn path.
        """
        global rightturntime
        global turning
        global SLOPE
        global heading


        dir = (desired_heading - heading)%4
        heading = desired_heading
        # Change the turning magnitudes to the shortest turn direction
        if dir == 3:
            dir = -1
        elif dir == -3:
            dir = 1
        
        # Turn for designated number of times
        for i in range(abs(dir)):
            starttime = time.time()
            turning = True
            while turning:
                # elif manages the direction of turn
                if dir > 0:
                    self.motor.set(-380*SLOPE, 380*SLOPE)
                elif dir < 0:
                    self.motor.set(380*SLOPE, -380*SLOPE)
                self.infrared.readSensors() # Can activate callback to end turn on the condition of line detected
                if (time.time() - starttime > rightturntime + 0.05): # Can end turn on the condition of time exceeds a 90 degree turn
                    turning = False
            
        self.motor.setvel(0,0)
        time.sleep(0.2)
        print("heading: ", heading)

    def spincheck(self):
        """
        Input:
            none
        Output:
            list, a list of booleands representing whether paths exist in the
                Forward, Left, Backward, Right directions
        Determines the paths available at an intersection when the robot arrives at a new intersection.
        """
        paths = []
        for i in range(4): 
            # pathexist is a boolean representing whether that pathway exists
            pathexist = self.turn_90()
            paths.append(pathexist)
        # initial [right, back, left, forward]
        # reorder [forward, left, back, right]     
        paths_reorder = [paths[3],paths[2], paths[1], paths[0]]
        return paths_reorder
    
    # # Triggering thread: Continually trigger the ultrasound.
    # def triggering_stop(self):
    #     self.triggering_stopflag = True

    # def triggering_loop(self):
    #     triggering_stopflag = False
    #     while not triggering_stopflag:
    #         self.left_us.trigger()
    #         self.center_us.trigger()
    #         self.right_us.trigger()
    #         time.sleep(0.08 + 0.04 * random.random())

    # # # # Driving thread: drive from intersection to intersection
    # def driving_stop(self):
    #     self.driving_stopflag = True
    
    # def driving_loop(self):
    #     self.driving_stopflag = False
    #     while not self.driving_stopflag:
    #         # Pause at this intersection, if requested
    #         if self.driving_pause:
    #             continue
    #         # Move from this intersection to the next intersection
    #         self.linefollow(self.center_us)

    # def userinput(self):
    #     global long
    #     global lat

    #     while True:
    #         # Grab a command
    #         command = input("Command ? ")
    #         # Compare against possible commands.
    #         if (command == 'pause'):
    #             print("Pausing at the next intersection")
    #             self.driving_pause = True
    #         elif (command == 'explore'):
    #             print("Exploring without a target")
    #             self.driving_pause = False
    #             self.driving_stopflag = False
    #             self.triggering_stopflag = False
                
    #             shortest_dist = 100000
    #             for i in intersections:
    #                 if i.streets.index('UNEXPLORED') > 0:
    #                     dist = abs(long - i.long) + abs(lat - i.lat)
    #                     if dist < shortest_dist:
    #                         self.djikstra(self.intersection(long, lat), i)
    #                         self.heading_to_target(i)
    #                         shortest_dist = dist
    #             if shortest_dist == 100000:
    #                 self.driving_loop()

    #             self.driving_pause = False
    #         elif (command == 'goto'):
    #             goal_long = int(input("longitude? "))
    #             goal_lat = int(input("latitude? "))

    #             print("Driving to a target")
    #             if self.intersection(goal_long, goal_lat) != None:
    #                 self.djikstra(self.intersection(long, lat), self.intersection(goal_long, goal_lat))
    #                 self.heading_to_target(self.intersection(goal_long, goal_lat))
    #             else: # user input again sketch
    #                 # go to intersection not yet seen
    #                 self.userinput()

    #             # ... set flags so the robot drive to the given location.
    #             self.driving_pause = False
    #         elif (command == 'print'):
    #             print(map)
    #             # ... and/or useful debug values?
    #         elif (command == 'quit'):
    #             print("Quitting...")
    #             break
    #         elif (command == 'save'):
    #             print("Saving the map...")
    #             with open('filename.pickle', 'wb') as file:
    #                 pickle.dump(map, file)
    #         elif (command == 'load'):
    #             print("Loading the map...")
    #             with open('filename.pickle', 'rb') as file:
    #                 map = pickle.load(file)
    #         elif (command == 'home'):
    #             print("Restarting the robot at the home position")
    #             long = 0
    #             lat = 0
    #         else:
    #             print("Unknown command '%s'" % command)
    
    def ultraturn(self, l, c, r):
        """
        Input:
            l, left ultrasonic object
            c, center ultrasonic object
            r, right ultrasonic object
        Output:
            None
        
        Turns based on the ultrasonic sensor readings.
        """
        
        global VEL_NOM
        global RAD_LARGE
        global RAD_SMALL
        try:
            while True:
                if (not l.object_present and c.object_present and not r.object_present): # 010, drives away from front object
                    self.motor.setvel(-1*VEL_NOM, 0)
                elif (not l.object_present and c.object_present and r.object_present): # 011, turn away from right object
                    self.motor.setspin(-300)
                elif (not l.object_present and not c.object_present and r.object_present): # 001, turn away from right object
                    self.motor.setspin(-300)
                elif (l.object_present and c.object_present and not r.object_present): # 110, turn away from left object
                    self.motor.setspin(300)
                elif (l.object_present and not c.object_present and not r.object_present): # 100, turn away from left object
                    self.motor.setspin(300)
                elif (not l.object_present and not c.object_present and not r.object_present): # 000, keep driving straight
                    self.motor.setvel(VEL_NOM, 0)
                elif (l.object_present and c.object_present and r.object_present): # 111, surrounded, spin in place until front clear
                    self.motor.setspin(300)
                else: # 101, surrounded on either side so just drive straight 
                    self.motor.setvel(VEL_NOM, 0)
        except BaseException as ex:
                print("Ending due to exception: %s" % repr(ex))
                
    def linefollow(self):
        """
        Input:
            ultra, front ultrasonic object
        Output:
            none
        
        Follows a line until it reaches an intersection, but pauses when it detects an obstacle in front of the robot.
        """
        global VEL_NOM
        global RAD_LARGE
        global RAD_SMALL
        global turned_around
        
        state = 'C'
        exitcond = False
        turned_around = False

        global long
        global lat
        global heading
        global lastintersection

        try:
            while not exitcond:
                # Reading IR Sensors
                (ir_left_state, ir_center_state, ir_right_state) = self.infrared.readSensors()
                # Setting robot states
                if self.center_us.object_present:
                    print("turned around")
                    self.spintonextline(heading + 2)
                    turned_around = True
                elif (ir_left_state == 0 and ir_center_state == 1 and ir_right_state == 0): # centered
                    self.motor.setvel(VEL_NOM, 0)
                    lost_counter = 0
                    state = 'C'
                elif (ir_left_state == 0 and ir_center_state == 1 and ir_right_state == 1): # slight left
                    self.motor.setvel(VEL_NOM, math.sin(RAD_SMALL)*VEL_NOM/0.125)
                    lost_counter = 0
                    state = 'L'
                elif (ir_left_state == 0 and ir_center_state == 0 and ir_right_state == 1): # more left
                    self.motor.setvel(VEL_NOM, math.sin(RAD_LARGE)*VEL_NOM/0.125)
                    lost_counter = 0
                    state = 'L'
                elif (ir_left_state == 1 and ir_center_state == 1 and ir_right_state == 0): # slight right
                    lost_counter = 0
                    state = 'R'
                    self.motor.setvel(VEL_NOM, -1*math.sin(RAD_SMALL)*VEL_NOM/0.125)
                elif (ir_left_state == 1 and ir_center_state == 0 and ir_right_state == 0): # more right
                    lost_counter = 0
                    state = 'R'
                    self.motor.setvel(VEL_NOM, -1*math.sin(RAD_LARGE)*VEL_NOM/0.125)
                elif (ir_left_state == 0 and ir_center_state == 0 and ir_right_state == 0): # passed end or pushed off
                    print("000")
                    if (self.left_us.object_present and self.right_us.object_present):
                        print("senses tunnel")
                        self.tunnel_follow()
                    else:
                        # time buffer?
                        self.motor.setvel(0,0)
                        exitcond = True
                    # self.motor.setvel(0,0)
                    # time.sleep(3)
                    # exitcond = True
                elif (ir_left_state == 1 and ir_center_state == 1 and ir_right_state == 1): # seeing intersection
                    print("SAW AN INTERSECTION")
                    self.motor.setlinear(VEL_NOM)
                    time.sleep(0.42)
                    self.motor.setvel(0,0)
                    exitcond = True
                    if not turned_around: # sketch idk
                        (long, lat) = self.shift(long, lat, heading)
        except BaseException as ex:
                print("Ending due to exception: %s" % repr(ex)) 

    def tunnel_follow(self):
        # Note that turning is turned to False when a line is detected
        global turning
        global VEL_NOM
        global RAD_SMALL
        global RAD_LARGE

        turning = True
        while turning:
            if (self.left_us.distance < self.right_us.distance): # slight left
                self.motor.setvel(VEL_NOM, math.sin(RAD_SMALL)*VEL_NOM/0.125)
            elif (self.left_us.distance > self.right_us.distance): # slight right
                self.motor.setvel(VEL_NOM, -1*math.sin(RAD_SMALL)*VEL_NOM/0.125)
            else:
                self.motor.setvel(VEL_NOM, 0)

    def wall_follow(self, u):
        """
        Input:
            u, float representing the speed motor correction factor
        Output:
            none
        
        Follows a wall that is detected to the right of the robot.
        """
        # Set motor velocities depending on the error detected
        PWM_left = max(0.5, min(0.9,0.7-u))
        PWM_right = max(0.5, min(0.9, 0.7+u))
        self.motor.set(PWM_left, PWM_right)
        time.sleep(0.1)

    def convertabsolute(self, paths):
        """
        Input:
            paths, a list of paths at an intersection with the corresponding statuses (i.e. NoStreet, Unexplored, etc.)
        Output:
            paths_updated, a list of paths with the street statuses in the correct absolute order
        
        Reorders paths so that they are in the correct, absolute order.
        """
        # paths input is [Forward, Left, Backward, Right]
        # desired return order is [NORTH, WEST, SOUTH, EAST]
        global heading
        # convert = {True : UNEXPLORED, False: NOSTREET}
        # if heading == NORTH:
        #     return [convert[paths[0]], convert[paths[1]], convert[paths[2]], convert[paths[3]]]
        # elif heading == EAST:
        #     return [convert[paths[1]], convert[paths[2]], convert[paths[3]], convert[paths[0]]]
        # elif heading == WEST:
        #     return [convert[paths[3]], convert[paths[0]], convert[paths[1]], convert[paths[2]]]
        # elif heading == SOUTH:
        #     return [convert[paths[2]], convert[paths[3]], convert[paths[0]], convert[paths[1]]]

        if heading == NORTH:
            return [paths[0], paths[1], paths[2], paths[3]]
        elif heading == EAST:
            return [paths[1], paths[2], paths[3], paths[0]]
        elif heading == WEST:
            return [paths[3], paths[0], paths[1], paths[2]]
        elif heading == SOUTH:
            return [paths[2], paths[3], paths[0], paths[1]]

    def headback(self):
        """
        Input:
            none
        Output:
            none
        
        Heads back towards the starting point of the robot (0, 0).
        """
        global intersections
        global heading
        global long
        global lat
        print("heading back")
        # Iterate through intersections backwards, to go from current intersection to the first one
        # based on headingToTarget.
        for i in range(len(intersections)-1, 0,-1):
            self.spintonextline(intersections[i].headingToTarget)
            # self.spintonextline((intersections[i].headingToTarget-heading)%4) # OLD SHIT
            # heading = intersections[i].headingToTarget
            self.linefollow(self.center_us)
        # Spin to face north and await next command
        self.spintonextline(NORTH)
        # self.spintonextline((NORTH-heading)%4) # OLD SHIT
        # Reset position
        long = 0
        lat = 0
        # heading = NORTH # OLD SHIT

    def heading_to_target(self, goal):
        """
        Input:
            none
        Output:
            none
        
        Heads to the target based on headingToTarget of intersections. This
        is generally called after Djikstra's algorithm which resets and adds
        new values for each headingToTarget of each intersection.
        """
        global heading
        global lat
        global long
        current_intersection = self.intersection(long,lat)
        # Keeps running until it arrives at the target intersection whcih has a headingToTarget of None
        while current_intersection.headingToTarget != None:
            # Navigate to the intersection pointed to by current intersection's headingToTarget
            self.spintonextline(current_intersection.headingToTarget)
            self.linefollow(self.center_us)
            self.spincheck()
            self.djikstra(self.intersection(long, lat), goal)
            # Update current intersection and repeat
            current_intersection = self.intersection(long,lat)
        # Arrived at the target intersection
        self.motor.setvel(0,0)
        time.sleep(0.2)

        
    def trackmap(self):
        """
        Input:
            none
        Output:
            none
        
        Roams through the map discovering new intersections. Continues roaming
        until all intersections are discovered and the map is completely
        understood.
        """
        global heading
        global long
        global lat
        global lastintersection
        global intersections
        lasttime = time.time()
        global ultrastarttime
        
        # Line follow to the next intersection
        self.linefollow()
        # (long, lat) = self.shift(long, lat, heading) # DELETE LATER
        # If this is a new intersection, append to intersections and detect paths to
        # store to the Intersection object
        if self.intersection(long, lat) == None:
            print("Made a new intersection!")
            inter = Intersection(long, lat)
            intersections.append(inter) # append
            if self.right_us.object_present and self.left_us.object_present: # if in a tunnel
                # [forward, left, back, right]
                paths = [UNEXPLORED, NOSTREET, UNEXPLORED, NOSTREET]
            else: # if not in a tunnel
                paths = self.spincheck()
            inter.streets = self.convertabsolute(paths) #USES PATHS
            if lastintersection != None:
                inter.headingToTarget = (heading+2)%4
        else:
            inter = self.intersection(long, lat)
            paths = self.spincheck()
            inter.streets = self.convertabsolute(paths)
        # If intersection already exists at this location, update headingToTarget if it does not
        # already exist. Otherwise, set street condition to origin street as connected.
        if lastintersection != None:
            inter = self.intersection(long,lat)
            if inter.headingToTarget == None:
                inter.headingToTarget = (heading+2)%4
            if not turned_around:
                lastintersection.streets[heading] = CONNECTED
                inter.streets[(heading+2)%4] = CONNECTED
            else:
                lastintersection.streets[(heading+2)%4] = BLOCKED
        # Randomly determine which street to head down to continue exploration
        k = random.randint(0,len(self.intersection(long,lat).streets)-1)
        allc = True
        for i in range(0,len(self.intersection(long,lat).streets)):
            if self.intersection(long,lat).streets[i] == UNEXPLORED or self.intersection(long,lat).streets[i] == UNKNOWN:
                allc = False  # Found a street that is not connected at the current intersection
                break
        if allc:  # All streets are connected at current intersection and a random direction is chosen 
            k = random.randint(0,3)
            while self.intersection(long,lat).streets[k] == NOSTREET:
                k = random.randint(0,3)
        else:  # There is at least one street that is not connect, favor exploring these streets
            for i in range(0,len(self.intersection(long,lat).streets)):
                if self.intersection(long,lat).streets[i] == UNEXPLORED:
                    k = i # select this path to explore
                    break
            while self.intersection(long,lat).streets[k] == NOSTREET: # All streets are either NOSTREET or CONNECTED
                k = random.randint(0,len(self.intersection(long,lat).streets)-1) # Must be a CONNECTED street
        # Update values and orientation to keep exploring
        self.spintonextline(k)
        print("My new heading is: ", k)
        # self.spintonextline((k-heading)%4) # OLD SHIT
        # heading = k # OLD SHIT
        lastintersection = self.intersection(long,lat)
        print(intersections)

    def djikstra(self, start, goal):
        """
        start, an intersection object that is the starting position of the robot
        goal, an interseciton object that is the ending position of the robot
        Run the Djikstra algorithm to calculate the shortest route from the start position to the target position.
        """
        global intersections
        to_be_processed = []
        print("entered dijkstra")
        # Iterate through intersections, clearing headingToTarget
        for inter in intersections:
            inter.headingToTarget = None
        # Begin at goal and update headingToTarget values to pathfind
        to_be_processed.append(goal)
        temp_target = to_be_processed.pop(0)
        stop_cond = False # only set to true once a path is found from start to goal
        while temp_target.lat != start.lat or temp_target.long != start.long:
            for i in range(0,len(temp_target.streets)):
                # getting neighbors to current intersection
                if i == 0:
                    checking_intersection = self.intersection(temp_target.long,temp_target.lat+1)
                elif i == 1:
                    checking_intersection = self.intersection(temp_target.long-1,temp_target.lat)
                elif i == 2:
                    checking_intersection = self.intersection(temp_target.long,temp_target.lat-1)
                elif i == 3:
                    checking_intersection = self.intersection(temp_target.long+1,temp_target.lat)    

                # If the street has connected neighbors not already been analyzed, continue analyzing 
                if temp_target.streets[i] == CONNECTED and checking_intersection.headingToTarget == None:
                    checking_intersection.headingToTarget = (i+2)%4 # point toward temp_target
                    # Check if the intersection in question is goal intersection, headingToTarget should be None
                    if checking_intersection.long == goal.long and checking_intersection.lat == goal.lat:
                        checking_intersection.headingToTarget = None
                    # Check if the intersection in question is the start intersection, the algorithm is complete
                    if checking_intersection.long == start.long and checking_intersection.lat == start.lat:
                        to_be_processed.append(checking_intersection)
                        stop_cond = True
                        break
                    # If the intersection in question is not the start, the algorithm continues
                    else:
                        to_be_processed.append(checking_intersection)
            # Path found from start to goal
            if stop_cond:
                break
            if len(to_be_processed) != 0:
                temp_target = to_be_processed.pop(0)
    
    # New longitude/latitude value after a step in the given heading.
    def shift(self, long, lat, heading):
        """
        Input:
            long, int representing the longitude (East/West) coordinate
            lat, int representing the latitude (North/South) coordinate
            heading, int representing the current heading of the robot
        Output:
            none
        
        Shift the position of the robot in accordance with its current heading and direction driven.
        """ 
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
    def intersection(self, long, lat):
        """
        Input:
            long, an int representing longitude
            lat, an int representing latitude
        Output:
            intersection, an intersection object
        Look for an intersection with the same longitude and latitude as that handed in. If an intersection is found,
        return that intersection object. If it is not found, return None.
        """
        list = [i for i in intersections if i.long == long and i.lat == lat]
        if len(list) == 0:
            return None
        if len(list) > 1:
            raise Exception("Multiple intersections at (%2d, %2d)" % (long, lat))
        return list[0]

#
#   Main
#
if __name__ == "__main__":
    ############################################################
    
    robot = Robot()
    
    k = 0.5
    # k = 3
    # k = 0.03
    
    # try:
    #     robot.userinput()
    # except BaseException as ex:
    #     print("Ending due to exception: %s" % repr(ex))
    #     traceback.print_exc()
    
    try:
        # robot.linefollow(robot.center_us)
        # robot.spincheck()

        # while True:
        #     print(robot.center_us.get_object_present())


        allConnected = False
        robot.trackmap()
        while allConnected == False:
            allConnected = True
            print("Intersections: ", intersections)
            for i in intersections:
                for s in i.streets:
                    if s == UNEXPLORED or s == UNKNOWN or s == BLOCKED:
                        allConnected = False
            print(allConnected)
            robot.trackmap()

        # time.sleep(3)
        # robot.cycle_deadends()
        
        # while True:
        #     e = robot.right_us.distance - robot.right_us.stop_distance
        #     robot.wall_follow(-1*k*e)
        # robot.motor.setvel(0.4,0.4/right_us.stop_distance)
        
            

            #print("dist: ", [left_us.distance, center_us.distacnce, right_us.distance])
        
    # except BaseException as ex:
    #     print("Ending due to exception: %s" % repr(ex))
        
    except KeyboardInterrupt:
        robot.motor.setvel(0,0)
     
    robot.shutdown()
