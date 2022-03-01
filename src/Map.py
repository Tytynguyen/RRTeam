#   Map.py
#
#   Stores data structures for Map and Points

import numpy as np
from planarutils import *
from utilities import *
from RRTStar import *

## GLOBAL CONSTANTS
SENSOR_RANGE = 1 # in (x, y) units

ROBOT_WIDTH = 0.5
## END GLOBAL CONSTANTS

'''
Master class for mapping.
Stores:
- List of Segments identified as walls (map.segments)
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
        self.xlim = xrange
        self.ylim = yrange

        # add each wall as a segment
        self.segments.append(Segment(Point(xrange[0], yrange[0]),
                                     Point(xrange[1], yrange[0]), 1)) # lower wall

        self.segments.append(Segment(Point(xrange[1], yrange[0]),
                                     Point(xrange[1], yrange[1]), 1)) # right wall

        self.segments.append(Segment(Point(xrange[1], yrange[1]),
                                     Point(xrange[0], yrange[1]), 1)) # top wall

        self.segments.append(Segment(Point(xrange[1], yrange[1]),
                                     Point(xrange[0], yrange[1]), 1)) # left wall

    ''' addSegment()
    INPUTS:
    pt1: pt describing first endpoint
    pt2: pt describing second endpoint
    prob: probability that segment is truly a wall
    '''
    def addSegment(self, pt1, pt2, prob):
        self.segments.append(Segment(pt1, pt2, prob))
        # TODO: do some checking here to see if we can elide our segment with another
        # i.e. check if this segment is "near" any other segments
        # for each nearby segment, check colinearity
        # if within a specific threshold, combine the segments
        # be careful not to close doorway gaps (@Tyler)

    def localPlanner(self, pt1, pt2):
        moveSegment = Segment(pt1, pt2)

        for segment in self.segments:
            # If plan crosses a wall -> FAIL
            if SegmentCrossSegment(moveSegment.seg, segment.seg):
                return False

        return True



'''
Robot class handles robot motion and wall collision checking
Stores:
- Current robot position (robot.pos)
- Current robot orientation, as an angle in radians (robot.theta)
- Robot map (robot.map)

Relies on:
- true map of world (robot.world)
- SENSOR_RANGE value
- ROBOT_WIDTH value
'''
class Robot():
    ''' init()
    INPUTS:
    walls: list of wall segments in world (TODO: make grid points instead)
    map: Map class instance
    pinit: Point describing position of robot center at start
    tinit: angle (radians) describing orientation of robot at start
    '''
    def __init__(self, walls, map, pinit, tinit):
        # define class variables
        self.walls = walls
        self.map = map
        self.pos = pinit
        self.theta = tinit

    ''' goto()
    Robot will turn and move towards a point until it encounters an obstacle.
    Turn will happen first, then movement.
    INPUTS:
    p2: Point describing final position of robot

    RETURNS:
    True if successfully travelled, false if wall hit and map updated
    '''
    def goto(self, p2):
        ## Turn to face where we need to go
        # compute angle to go to p2
        t = self.pos.getAngle(p2)
        # turn!
        if not turn(t):
            # collision occured while turning. Map is updated.
            # THIS SHOULD NOT HAPPEN IN CURRENT VERSION
            return False

        ## Move towards our point!
        # also handles collision checking and map updating
        self.pos = self.distSensor(p2)
        return



    ''' turn()
    Robot will turn to the absolute angle provided, given it does not hit a wall
    INPUTS:
    tabs: angle (radians) to reach relative to world coord frame.

    RETURNS:
    True if successfully travelled, false if wall hit and map updated
    (always returns true for now)
    '''
    def turn(self, tabs):
        self.theta = tabs
        return True


    '''
    Checks if there is a wall ahead of the robot. Adds wall to map if so
    RETURNS:
    final position of robot after travel (stopping in front of walls)
    '''
    def distSensor(self, p2):
        # create a segment between current position and final position
        path = Segment(self.pos, p2)

        closestWall = -1
        closestWallDist = -1
        closestWallSegPos = -1
        # check each wall for a collision
        for wall in self.walls:
            # TODO: implement SegmentCrossRectangle
            (i1, i2, segPos) = SegmentCrossRectangle(wall, path.seg, ROBOT_WIDTH / 2)
            if (i1 == -1):
                # no collision
                continue
            else:
                # check to see if this wall is closest
                if closestWall == -1:
                    closestWall = (i1, i2)
                    closestWallDist = getdist(i1, self.pos)
                    closestWallSegPos = segPos
                else:
                    d = getDist(i1, self.pos)
                    if (d < closestWallDist):
                        # this intersection is closer, so this wall is the one that matters
                        closestWall = (i1, i2)
                        closestWallDist = d
                        closestWallSegPos = segPos

        # all walls processed
        if (closestWall == -1):
            # no wall found!
            return p2
        else:
            # Hit a wall!!
            # update map
            map.addSegment(Point(i1[0], i1[1]),
                           Point(i2[0], i2[1]), 1)

            # back away from intersection by a little
            # TODO: BACK AWAY FROM THE SEGMENT BY A little
            # and double check that you aren't moving past the starting point
            # return that final position
            # (for now, just go to segment)
            return closestWallSegPos






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
    points = (Point(2, 4), Point(4, 4), Point(8, 12), Point(5, 9))
    segments = (Segment(points[0], points[1], 0.46),
                Segment(points[1], points[2], 0.63),
                Segment(points[0], points[2], 0.20),
                Segment(points[3], points[2], 0.83),
                Segment(points[1], points[3], 1))
    visual = Visualization((10, 15))
    visual.ShowWorld(walls, start, goal)
    visual.ShowBot(start, 0)
    visual.ShowPoints(points)
    visual.ShowSegments(segments)
    visual.ShowFigure()
    input("Test displayed.")

def main():
    TestVisualization()

if __name__== "__main__":
    main()
