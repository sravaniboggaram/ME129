
# Import constants
from constants import *
from Robot import *

# Establish global variables
intersections = [] # list of intersections
lastintersection = None # last intersection visited
long = 0 # current east/west coordinate
lat = -1 # current north/south coordinate
heading = NORTH # current heading

# turning boolean establishes whether the robot is currently turning.
turning = False
# A constant to meaasure the time it takes to turn right approximately 90
# degrees
rightturntime = 0.57
# A boolean representing whether a front object is currently present, based on
# the ultrasonic sensor
front_object = False

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
                    if s == UNEXPLORED or s == UNKNOWN:
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
    