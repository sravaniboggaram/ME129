
# Import constants
from constants import *

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
    
    def stop_on_line(self, gpio, level, tick):
        """
        Establish a callback that enables the robot to stop once it has
        determined to have seen a line.
        """
        # Set global turning to False. Motors will then respond.
        global turning
        turning = False

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
