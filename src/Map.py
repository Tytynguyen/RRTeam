#   Map.py
#
#   Stores data structures for Map and Nodes

import numpy as np
from planarutils import *
from utilities import *

## GLOBAL CONSTANTS
SENSOR_RANGE = 1 # in (x, y) units
## END GLOBAL CONSTANTS

'''
Master class for mapping.
Stores:
- List of Segments identified as walls (map.segments)
- List of Nodes (TODO: may not be necessary) (map.nodes)
'''
class Map():
    ''' init()
    INPUTS:
    xrange: [xmin, xmax] tuple with max/min x
    yrange: [ymin, ymax] tuple with max/min y
    '''
    def __init__(self, xrange, yrange):
        # define class variables
        self.segments = [] # list of segments
        self.nodes = [] # list of nodes

        # add each wall as a segment
        segments.append(Segment(Node(xrange[0], yrange[0]),
                                Node(xrange[1], yrange[0]), 1)) # lower wall

        segments.append(Segment(Node(xrange[1], yrange[0]),
                                Node(xrange[1], yrange[1]), 1)) # right wall

        segments.append(Segment(Node(xrange[1], yrange[1]),
                                Node(xrange[0], yrange[1]), 1)) # top wall

        segments.append(Segment(Node(xrange[1], yrange[1]),
                                Node(xrange[0], yrange[1]), 1)) # left wall

    ''' addSegment()
    INPUTS:
    Node1: node describing first endpoint
    Node2: node describing second endpoint
    prob: probability that segment is truly a wall
    '''
    def addSegment(self, Node1, Node2, prob):
        segments.append(Segment(Node1, Node2, prop))
        # TODO: do some checking here to see if we can elide our segment with another
        # i.e. check if this segment is "near" any other segments
        # for each nearby segment, check colinearity
        # if within a specific threshold, combine the segments
        # be careful not to close doorway gaps (@Tyler)


'''
Robot class handles robot motion and wall collision checking
Stores:
- Current robot position (robot.pos)
- Current robot orientation, as an angle in radians (robot.theta)

Relies on:
- true map of world (robot.world)
- SENSOR_RANGE value (robot.range)
- visualizer (robot.viz)
'''
class Robot():
    ''' init()
    INPUTS:
    world: array (could be image) describing walls (1) and free space (0)
    pinit: [x, y] describing position of robot center at start
    tinit: angle (radians) describing orientation of robot at start
    viz: Visualization class instance to update
    '''
    def __init__(self, world, pinit, tinit, viz):
        # define class variables
        self.world = world
        self.pos = pinit
        self.theta = tinit
        self.range = SENSOR_RANGE
        self.viz = viz

    ''' goto()
    Robot will turn and move towards a point until it encounters an obstacle.
    Turn will happen first, then movement.
    INPUTS:
    p2: [x, y] describing final position of robot
    speed: gridpoints/second to travel at (will be discretized into smaller chunks)

    RETURNS:
    True if successfully travelled, false if wall hit and map updated
    '''
    def goto(self, p2, speed = 1):
        # TODO


    ''' turn()
    Robot will turn to the absolute angle provided, given it does not hit a wall
    INPUTS:
    p2: [x, y] describing final position of robot

    RETURNS:
    True if successfully travelled, false if wall hit and map updated
    '''
    def turn(self, tabs):
        # TODO


    '''
    Checks if there is a wall ahead of the robot. Adds wall to map if so
    RETURNS:
    T if wall encountered, F otherwise
    '''
    def distSensor(self):
        # TODO



def TestVisualization():
    # Generate example world with walls
    walls = (((2,  4), (5,  9)),
             ((5,  9), (4,  4)),
             ((4,  4), (2,  4)),
             ((2, 12), (9, 12)),
             ((2, 14), (9, 12)),
             ((2, 12), (2, 14)))

    start = (1, 1)
    goal  = (9, 14)

    # Gimme some test segments to visualize.
    nodes = (Node(2, 4), Node(4, 4), Node(8, 12), Node(5, 9))
    segments = (Segment(nodes[0], nodes[1], 0.46),
                Segment(nodes[1], nodes[2], 0.63),
                Segment(nodes[0], nodes[2], 0.20),
                Segment(nodes[3], nodes[2], 0.83),
                Segment(nodes[1], nodes[3], 1))
    visual = Visualization((10, 15))
    visual.ShowWorld(walls, start, goal)
    visual.ShowBot(start, 0)
    visual.ShowNodes(nodes)
    visual.ShowSegments(segments)
    visual.ShowFigure()
    input("Test displayed.")

def main():
    TestVisualization()

if __name__== "__main__":
    main()
