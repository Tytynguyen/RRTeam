#   Map.py
#
#   Stores data structures for Map and Points

import numpy as np
from planarutils2 import *
from utilities import *
from RRTStar import *
import worlds as worlds
import random

## GLOBAL CONSTANTS
SENSOR_RANGE = 0.1 # km

ROBOT_WIDTH = 0.1
CUSHION = 1
ELIDE_JUMP = 0.05
## END GLOBAL CONSTANTS

'''
Master class for mapping.
Stores:
- List of Segments identified as walls (map.segments)
'''
class Map():
    ''' init()
    INPUTS:
    minpt: Point containing coords of bottom left
    maxpt: Point containing coords of top right
    '''
    def __init__(self, minpt, maxpt):
        # define class variables
        self.segments = [] # list of segments
        self.xlim = (minpt.x, maxpt.x)
        self.ylim = (minpt.y, maxpt.y)
        self.elideCounter = 0

        # add each wall as a segment
        self.segments.append(Segment(Point(self.xlim[0], self.ylim[0]),
                                     Point(self.xlim[1], self.ylim[0]), 1)) # lower wall

        self.segments.append(Segment(Point(self.xlim[1], self.ylim[0]),
                                     Point(self.xlim[1], self.ylim[1]), 1)) # right wall

        self.segments.append(Segment(Point(self.xlim[1], self.ylim[1]),
                                     Point(self.xlim[0], self.ylim[1]), 1)) # top wall

        self.segments.append(Segment(Point(self.xlim[1], self.ylim[1]),
                                     Point(self.xlim[0], self.ylim[1]), 1)) # left wall

    ''' addSegment()
    INPUTS:
    pt1: pt describing first endpoint
    pt2: pt describing second endpoint
    prob: probability that segment is truly a wall
    '''
    def addSegment(self, s):
        # self.segments.append(s)
        # do some checking here to see if we can elide our segment with another
        # i.e. check if this segment is "near" any other segments
        # for each nearby segment, check colinearity
        # if within a specific threshold, combine the segments

        eliding = False
        for seg in self.segments:
            # check if they intersect
            # if (SegmentCrossSegment(s.seg, seg.seg)):
            # compute slopes
            if (s.xdif() == 0):
                slope1 = 100
            else:
                slope1 = s.ydif() / s.xdif()
                if (slope1 < -100):
                    slope1 = 100

            if (seg.xdif() == 0):
                slope2 = 100
            else:
                slope2 = seg.ydif() / seg.xdif()
                if (slope2 < -100):
                    slope2 = 100

            # check slopes
            ratio = (slope1 - slope2) / (1+slope1*slope2)
            if abs(ratio) < 0.05:
                # check proximity
                if (SegmentEndpointsNearSegment(ELIDE_JUMP, seg.seg, s.seg)):
                    eliding = True
                    self.elideCounter += 1
                    self.segments.remove(seg)
                    # combine
                    s1 = Segment(s.pt1, seg.pt1)
                    s1len = s1.getLength()
                    s2 = Segment(s.pt1, seg.pt2)
                    s2len = s2.getLength()
                    s3 = Segment(s.pt2, seg.pt1)
                    s3len = s3.getLength()
                    s4 = Segment(s.pt2, seg.pt2)
                    s4len = s4.getLength()

                    # pick the longest resulting segment
                    if (s1len > s2len):
                        if (s1len > s3len):
                            if (s1len > s4len):
                                self.segments.append(s1)
                            else:
                                self.segments.append(s4)
                        else:
                            if (s3len > s4len):
                                self.segments.append(s3)
                            else:
                                self.segments.append(s4)
                    else:
                        if (s2len > s3len):
                            if (s2len > s4len):
                                self.segments.append(s2)
                            else:
                                self.segments.append(s4)
                        else:
                            if (s3len > s4len):
                                self.segments.append(s3)
                            else:
                                self.segments.append(s4)
        if not eliding:
            self.segments.append(s)


    def localPlanner(self, pt1, pt2):
        moveSegment = Segment(pt1, pt2)
        # rectEdges = RectangleFromCenterline(moveSegment, ROBOT_WIDTH / 2)

        for segment in self.segments:
            # If plan crosses a wall -> FAIL
            if SegmentCrossSegment(moveSegment.seg, segment.seg):
                return False

            # intersection = SegmentCrossRectangle(segment, rectEdges)
            # if (intersection is not None):
            #     return False



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
    walls: list of wall segments in world
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
        self.distanceTraveled = 0

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
        if not self.turn(t):
            # collision occured while turning. Map is updated.
            # THIS SHOULD NOT HAPPEN IN CURRENT VERSION
            return False

        ## Move towards our point!
        # also handles collision checking and map updating
        newPos = self.distSensor(p2)
        self.distanceTraveled += self.pos.dist(newPos)
        self.pos = newPos

        if (self.pos == p2):
            return True
        else:
            return False



    ''' turn()
    Robot will turn to the absolute angle provided, given it does not hit a wall
    INPUTS:
    tabs: angle (radians) to reach relative to world coord frame.

    RETURNS:
    True if successfully travelled, false if wall hit and map updated
    (always returns true for now)
    '''
    def turn(self, tabs):
        # TODO check for impact when turning
        self.theta = tabs
        return True


    '''
    Checks if there is a wall ahead of the robot. Adds wall to map if so.
    UPDATED VERSION--USES PLANARUTILS2
    RETURNS:
    final position of robot after travel (stopping in front of walls)
    '''
    def distSensor(self, p2):
        # create segment between current position and p2
        path = Segment(self.pos, p2)
        pathDist = path.getLength()
        # Extended path for sensor reading when wall just past endpoint
        extPath = path.rescale(pathDist + SENSOR_RANGE)

        # loop through all walls to find ones that intersect the path
        rectEdges = RectangleFromCenterline(extPath, ROBOT_WIDTH / 2)

        # loop variables to store best outcome
        shortestDist = -1
        shortestPoint = None
        shortestWall = None

        for wallpts in self.walls:
            wall = Segment(wallpts[0], wallpts[1])

            # find intersection with this wall
            intersection = SegmentCrossRectangle(wall, rectEdges)

            ## CASE 1: NO INTERSECTION
            if (intersection == None):
                # wall is irrelevant
                continue

            ## CASE 2: POINT INTERSECTION
            # very rare
            if (intersection.pt1 == intersection.pt2):
                # TODO: do... something?
                print("Reached point intersection case. Ignoring...")
                continue

            ## CASE 3: FULL INTERSECTION
            # need to get:
            # 1. robot final position (closest position to wall along centerline)
            # 2. revised segment based on sensor range at final position (TODO)
            # need to compare with other final positions
            closestPoint = WhereSegmentOnPath(intersection, extPath, CUSHION)

            # check against other wall intersections
            dist = closestPoint.dist(self.pos)
            if (shortestDist == -1 or dist < shortestDist):
                # this is the closest intersection found
                shortestWall = intersection
                # Double check if the wall is past the endpoint
                if (dist > pathDist):
                    shortestDist = dist
                    shortestPoint = p2
                else:
                    shortestDist = dist
                    shortestPoint = closestPoint

            # intersection is behind some other known wall

        # all walls processed
        # CASE 1: NO WALLS IN PATH
        if (shortestWall == None):
            return p2

        # CASE 2: WALL FOUND IN PATH
        self.map.addSegment(shortestWall)
        return shortestPoint

def MapFromPath():
    ## SETUP
    # create a world (walls). Pick from any world you want in the worlds.py file
    world = worlds.door
    
    walls = world.walls
    minPt = world.minPt
    maxPt = world.maxPt

    # define starting postion
    startPt = world.startPt
    # define goal position
    goalPt = world.goalPt

    # create a map
    robotmap = Map(minPt, maxPt)

    # create a robot
    robot = Robot(walls, robotmap, Point(1, 1), 0)

    # create RRT object
    planner = RRTStar(startPt, goalPt, robot, robotmap, minPt, maxPt)

    # create viz
    visual = Visualization(walls, startPt, goalPt, minPt, maxPt)

    visual.ShowWorld()
    visual.ShowBot(robot)
    visual.ShowFigure()
    input("Initial world created. (hit return to continue)")

    ## Main loop: loop until hit goal or get stuck
    stepCounter = 0
    while True:
        # Create a path from goal to start
        goalNode = planner.update()
        visual.ClearFigure()
        visual.ShowNodes(planner.tree)
        visual.ShowSegments(robotmap.segments)
        if (goalNode is not None):
            #visual.ShowRRTSegments(planner.getPathSegments(goalNode))
            visual.ShowRRTSegments(planner.getTreeSegments())
        else:
            input("Planning failed, try changing dstep or Nmax")
            break
        visual.ShowBot(robot)
        visual.ShowFigure()

        if (robot.pos == goalPt):
            # TODO: add some statistics about path
            print("")
            print("DESTINATION REACHED")
            print("-------------------")
            print("*",stepCounter,"Steps")
            print("*",len(robotmap.segments), "Segments")
            print("*", robotmap.elideCounter, "Segments Elided")
            print("*", robot.distanceTraveled, "km Travelled")
            input("")
            break
        # input("Step")
        print("--",stepCounter,"--")
        print("Map segments:", len(robotmap.segments))
        stepCounter += 1


def main():
    random.seed(0)

    MapFromPath()

if __name__== "__main__":
    main()
