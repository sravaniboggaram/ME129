#dijkstra's algorithm
#target = destination intersection, start = starting intersection

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

        #NEW!!!!!!
        self.neighbors = []
        
        # add this to the global lsit of intersections to make it searchable
        global intersections
        if intersection(long, lat) is not None:
            raise Exception("Duplicate intersection at (%2d,%2d)" % (long,lat))

#in trackmap: as the bot finds all the streets at an intersection, add neighbor intersections to some list using the shift method
#add this list to the neighbors field for the current intersection

def dijkstra(target, start):
    global intersections
    for i in intersections:
        i.headingToTarget = None
    inter_tbp = [target] #intersections to be processed
    for inter in inter_tbp:
        if len(inter_tbp) != 0:
            temp_target = inter_tbp.pop(0)
        else:
            #no path??
            return
        #if the starting intersection is seen exit algorithm
        if temp_target == start:
            return
        for neighbor in temp_target.neighbors:
            heading = ?? #define heading that goes from temp_target to the current neighbor
            if neighbor.headingToTarget == None:
                neighbor.headingToTarget = heading 
                inter_tbp.append(neighbor) #append to inter_tbp so that its neighbors can be checked