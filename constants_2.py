
# Imports
import pigpio
import sys
import time
import math
import random
import statistics
import threading
import traceback
import pickle

# Define the motor pins.
MTR1_LEGA = 7   # motor 1: right motor
MTR1_LEGB = 8
MTR2_LEGA = 5   # motor 2: left motor
MTR2_LEGB = 6

# Define the IR pins
IR_LEFT = 14
IR_CENTER = 15
IR_RIGHT = 18

# Define the ultrasound pins
ULTRA1_ECHO = 16
ULTRA2_ECHO = 20
ULTRA3_ECHO = 21
ULTRA1_TRIGGER = 13
ULTRA2_TRIGGER = 19
ULTRA3_TRIGGER = 26

# Headings
NORTH = 0
WEST = 1
SOUTH = 2
EAST = 3
# For printing
HEADING = {NORTH:'North', WEST:'West', SOUTH:'South', EAST:'East', \
    None: 'None'}

# Street status
UNKNOWN = 'Unknown'
NOSTREET = 'NoStreet'
UNEXPLORED = 'Unexplored'
CONNECTED = 'Connected'
BLOCKED = 'Blocked'
TEST = 'Test'

# Slopes to adjust for linear drive of wheels
SLOPE1 = 1/0.552776
SLOPE = 1/469.19

# Values for velocity
VEL_NOM = 0.4
RAD_SMALL = 20*math.pi/180
RAD_LARGE = 40*math.pi/180

