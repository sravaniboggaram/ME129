
# Imports constants
from constants import *

class Motor:

    """
    A class to represent all motors.
    Attributes
    ----------
    io : pigpio.io
        use a passed in pigpio.io object to setup motor pins
    
    Methods
    -------
    set(self, leftdutycycle, rightdutycycle):
        Set the motor speed of left and right motors of the robot proportional
        to max motor speed.
    setlinear(self, speed):
        Set the linear speed of the robot. Speed is given in meters per
        second.
    setspin(self, speed):
        Set the speed of turn of the robot. Note that the robot spins in place
        in this function.
    setvel(self, linear, spin):
        Set a path of the robot in motion along the perimeter of a circle.
    """

    def __init__(self, io):
        """
        Constructs all the necessary attributes for the motor object.
        Parameters
        ----------
            io : pigpio.io
                pigpio.io object that contains pins, interface with Pi
        """

        self.io = io

        # set motor pins as output
        self.io.set_mode(MTR1_LEGA, pigpio.OUTPUT)
        self.io.set_mode(MTR1_LEGB, pigpio.OUTPUT)
        self.io.set_mode(MTR2_LEGA, pigpio.OUTPUT)
        self.io.set_mode(MTR2_LEGB, pigpio.OUTPUT)

        # Prepare the PWM.  The range gives the maximum value for 100%
        # duty cycle, using integer commands (1 up to max).
        self.io.set_PWM_range(MTR1_LEGA, 255)
        self.io.set_PWM_range(MTR1_LEGB, 255)
        self.io.set_PWM_range(MTR2_LEGA, 255)
        self.io.set_PWM_range(MTR2_LEGB, 255)
        
        # Set the PWM frequency to 1000Hz
        self.io.set_PWM_frequency(MTR1_LEGA, 1000)
        self.io.set_PWM_frequency(MTR1_LEGB, 1000)
        self.io.set_PWM_frequency(MTR2_LEGA, 1000)
        self.io.set_PWM_frequency(MTR2_LEGB, 1000)
        
        # Clear all motor pins, just in case.
        self.io.set_PWM_dutycycle(MTR1_LEGA, 0)
        self.io.set_PWM_dutycycle(MTR1_LEGB, 0)
        self.io.set_PWM_dutycycle(MTR2_LEGA, 0)
        self.io.set_PWM_dutycycle(MTR2_LEGB, 0)

    def set(self, leftdutycycle, rightdutycycle):
        """
        Input:
            leftdutycycle, float representing the percentage of maximum power
                of left motor
            rightdutycycle, float representing the percentage of maximum power
                of right motor
        Output:
            none
        Set the motor speed of left and right motors of the robot proportional
        to max motor speed.
        """
        # Negative leftdutycycle implies running in reverse, set MTR1_LEGB
        if leftdutycycle < 0.0:
            self.io.set_PWM_dutycycle(MTR1_LEGA, 0)
            self.io.set_PWM_dutycycle(MTR1_LEGB, int(-1*leftdutycycle*255))
        # Positive leftdutycycle implies running forward, set MTR1_LEGA
        else:
            self.io.set_PWM_dutycycle(MTR1_LEGA, int(leftdutycycle*255))
            self.io.set_PWM_dutycycle(MTR1_LEGB, 0)
        # Negative rightdutycycle implies running in reverse, set MTR2_LEGB
        if rightdutycycle < 0.0:
            self.io.set_PWM_dutycycle(MTR2_LEGA, 0)
            self.io.set_PWM_dutycycle(MTR2_LEGB, int(-1*rightdutycycle*255))
        # Positive rightdutycycle implies running forward, set MTR2_LEGA
        else:
            self.io.set_PWM_dutycycle(MTR2_LEGA, int(rightdutycycle*255))
            self.io.set_PWM_dutycycle(MTR2_LEGB, 0)
        
    def setlinear(self, speed):
        """
        Input:
            speed, integer represented speed in meters per second
        Output:
            none
        Set the linear speed of the robot. Speed is given in meters per
        second.
        """
        global SLOPE1
        # Slopes are used to adjust the motors relative to each other. For
        # testing, this is set to drive in a straight line.
        self.set(speed*SLOPE1, speed*SLOPE1)
        
    def setspin(self, speed):
        """
        Input:
            speed, float representing the speed in degrees per second
        Output:
            none
        
        Set the speed of turn of the robot. Note that the robot spins in place
        in this function. Speed is given in degrees per second where positive
        is clockwise, negative is counter clockwise.
        """
        # Slope adjusts so that we can get units of deg/s
        global SLOPE
        self.set(speed*SLOPE, -1*speed*SLOPE)
    
    def setvel(self, linear, spin):
        """
        Input:
            linear, float representing linear speed of the robot in meters/sec
            spin, float representing the angular speed of the robot in rad/sec
        Output:
            none
        
        Set a path of the robot in motion along the perimeter of a circle.
        """
        # If spin is 0, we are travelling linearly and call setlinear()
        # function.
        global SLOPE1
        if (spin == 0):
            self.setlinear(linear)
        else:
            # Use mathematical relationship between angular and linear velocity
            # to find motor speeds
            d = 2*linear/spin       
            width = 0.1285875
            T = 2*math.pi/spin
            v_outer = math.pi*(d+width)/T
            v_inner = math.pi*(d-width)/T
            # Use calculated values to set motor speeds
            self.set(v_outer*SLOPE1, v_inner*SLOPE1)
