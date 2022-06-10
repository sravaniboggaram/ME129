
# Import constants
from constants import *

class Ultrasonic:
    """
    A class to instantiate instances of the object ultrasonic sensors.
    Attributes
    ----------
    io : pigpio.io
        use a passed in pigpio.io object to setup ultrasonic pins
    triggerpin : int
        an int representing the pin of the pigpio.io pin corresponding to
        the trigger
    echopin : int
        an int representing the pin of the pigpio.io pin corresponding to
        the echo
    start_time : float
        a float representing the time that the ultrasonic trigger was
        activated
    object_present : boolean
        a boolean representing whether an object is or is not in front
    stop_distance : float
        a float representing a distance threshold in meters that would cause
        a reaction from the ultrasonic sensor input
    distance : float
        a float representing distance to closest object detected by the
        ultrasonic sensor
    stopflag : boolean
        a boolean used to control the stopping of threads
    cbrise : callback
            a callback that is activated by rising_edge of Ultrasonic sensor
    cfall : callback
            a callback that is activated by falling_edge of Ultrasonic sensor
    Methods
    -------
    rising(gpio, level, tick):
        Set start_time in order to calculate distance to the closest object.
    falling(gpio, level, tick):
        Use time measured for ultrasonic pulse to and from the object in
        order to measure distance. Change the boolean object_present based
        on distance to object.
    trigger():
        Trigger the ultrasonic sensor to pulse.
    stopcontinual():
        Set stopflag to True to stop threading.
    runcontinual():
        Keep running and triggering ultrasonic sensors to detect distance.
        Part of threading.
    """
    def __init__(self, triggerpin, echopin, io):
        """
        Constructs all the necessary attributes for the Ultrasonic object.
        Parameters
        ----------
            triggerpin : int
                an int representing the pin of the corresponding trigger
                for the Ultrasonic sensor
            echopin : int
                an int representing the pin of the corresponding echo
                for the Ultraonic sensor
            io : pigpio.io
                pigpio.io object that contains pins, interface with Pi
        """
        # Initialize the connection to the pigpio daemon (GPIO interface).
        self.io = io
        
        # Establish the pins for the trigger and echo
        self.triggerpin = triggerpin
        self.echopin = echopin
        
        # Functions of each variable described in docstring
        self.start_time = 0
        self.object_present = False
        self.stop_distance = 0.18
        self.distance = 0
        self.stopflag = False
        
        # Establish the pins for this ultrasonic sensor
        self.io.set_mode(triggerpin, pigpio.OUTPUT)
        self.io.set_mode(echopin, pigpio.INPUT)
        
        # Establish the callback functions for this ultrasonic sensor
        self.cbrise = self.io.callback(echopin, pigpio.RISING_EDGE, self.rising)
        self.cbfall = self.io.callback(echopin, pigpio.FALLING_EDGE, self.falling)

    def get_object_present(self):
        return self.object_present
    
    def rising(self, gpio, level, tick):
        """
        Input:
            gpio, level, and tick are all handled by the callback function.
        Output:
            none
        Set start_time in order to calculate distance to the closest object.
        """
        self.start_time = tick
    
    def falling(self, gpio, level, tick):
        """
        Input:
            gpio, level, and tick are all handled by the callback function.
        Output:
            none
        
        Use time measured for ultrasonic pulse to and from the object in
        order to measure distance. Change the boolean object_present based
        on distance to object.
        """
        self.distance = 343/2 * (tick - self.start_time) * (10**-6)
        if self.distance > self.stop_distance:
            self.object_present = False
        if self.distance < self.stop_distance:
            self.object_present = True
    
    def trigger(self):
        """
        Input:
            none
        Output:
            none
        
        Trigger the ultrasonic sensor to pulse.
        """
        self.io.write(self.triggerpin, 1)
        time.sleep(0.00001)
        self.io.write(self.triggerpin, 0)
        
    def stopcontinual(self):
        """
        Input:
            none
        Output:
            none
        
        Set stopflag to True to stop threading.
        """
        self.stopflag = True
        
    def runcontinual(self):
        """
        Input:
            none
        Output:
            none
        Keep running and triggering ultrasonic sensors to detect distance.
        Part of threading.
        """
        self.stopflag = False
        while not self.stopflag:
            self.trigger()
            time.sleep(0.05 + 0.01*random.random())      
