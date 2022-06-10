
# Import constants
from constants import *

class Intersection:
    """
    A class to represent each instance of Intersection as an object.
    Attributes
    ----------
    long : int
        an integer representing current longitude (East/West)
    lat : int
        an integer representing current latitude (North/South)
    streets : list of str
        a list with the status of 4 streets in string form, in the order of NWSE
    headingToTarget : int
        an int representing the heading that it arrived at the intersection from
    Methods
    -------
    none
    """
    def __init__(self, long, lat):
        """
        Constructs all the necessary attributes for the Intersection object located at (long, lat).
        Parameters
        ----------
            long : int
                an int representing the current longitude (East/West)
            lat : int
                an int representing the current latitude (North/South)
        """
        # save the parameters
        self.long = long
        self.lat = lat
        # status of streets at the intersection, in NWSE dirdctions.
        self.streets = [UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN]
        # direction to head from this intersection in planned move.
        self.headingToTarget = None
    
    # Print format.
    def __repr__(self):
        """
        Input:
            none
        Output:
            none
        
        Establish print format.
        """
        return ("(%2d, %2d) N:%s W:%s S:%s E:%s - head %s\n" %
                (self.long, self.lat, self.streets[0],
                self.streets[1], self.streets[2], self.streets[3],
                HEADING[self.headingToTarget]))
