from utilities import *

# All worlds 10x15, start in bottom left, goal in top right

'''
Represents a world for the robot to traverse. All worlds are rectangular.

STORES: min and max of rectangular world, startpoint of robot, goal point
'''
class World():
    def __init__(self, minPt, maxPt, startPt, goalPt, walls):
        self.minPt = minPt
        self.maxPt = maxPt
        self.startPt = startPt
        self.goalPt = goalPt
        self.walls = walls

smallWorldMinPt = Point(0,  0)
smallWorldMaxPt = Point(10, 15)
bottomLeftStartPt = Point(1, 1)
topRightGoalPt = Point(9, 14)

# Two triangles
triangles = World(smallWorldMinPt, smallWorldMaxPt, 
                  bottomLeftStartPt, topRightGoalPt,
                  ((Point(2,  4), Point(5,  9)),
                   (Point(5,  9), Point(4,  4)),
                   (Point(4,  4), Point(2,  4)),
                   (Point(2, 12), Point(9, 12)),
                   (Point(2, 14), Point(9, 12)),
                   (Point(2, 12), Point(2, 14)),
                   (Point(0,  0), Point(0, 15)),
                   (Point(0, 15), Point(10,15)),
                   (Point(10,15), Point(10, 0)),
                   (Point(10, 0), Point( 0, 0))))

# A vertical and horizontal line
simple =    World(smallWorldMinPt, smallWorldMaxPt, 
                  bottomLeftStartPt, topRightGoalPt,
                  ((Point(4,  0), Point(4,  6)),
                   (Point(4, 10), Point(9, 10)),
                   (Point(0,  0), Point(0, 15)),
                   (Point(0, 15), Point(10,15)),
                   (Point(10,15), Point(10, 0)),
                   (Point(10, 0), Point( 0, 0))))

# Obstruction forces robot to find an unintuitive door
door =      World(smallWorldMinPt, smallWorldMaxPt, 
                  bottomLeftStartPt, topRightGoalPt,
                  ((Point(0,   2), Point(8,  2)),
                   (Point(8,   2), Point(8,  8)),
                   (Point(10, 10), Point(2, 10)),
                   (Point(0,   0), Point(0, 15)),
                   (Point(0,  15), Point(10,15)),
                   (Point(10, 15), Point(10, 0)),
                   (Point(10,  0), Point( 0, 0))))

bigWorldMinPt = Point(0,   0  )
bigWorldMaxPt = Point(100, 150)
bigStartPt = Point(1, 1)
bigGoalPt = Point(90, 140)

# find your way out of a small hallway
exit =      World(bigWorldMinPt, bigWorldMaxPt, 
                  bigStartPt, bigGoalPt,
                  ((Point(0,    2), Point(50,  2)),
                   (Point(55,   0), Point(55,  5)),
                   (Point(50,   2), Point(50,  7))))