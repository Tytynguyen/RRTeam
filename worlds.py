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

# Obstruction forces robot to find an unintuitive door
maze =      World(smallWorldMinPt, smallWorldMaxPt, 
                  bottomLeftStartPt, topRightGoalPt,
                  ((Point(0,   3), Point(7,  3)),
                   (Point(10,  6), Point(3,  6)),
                   (Point(0,   9), Point(7,  9)),
                   (Point(10, 12), Point(3, 12))))

toughmaze = World(smallWorldMinPt, smallWorldMaxPt, 
                  bottomLeftStartPt, topRightGoalPt,
                  ((Point( 3,  3), Point(7,  3)),
                   (Point(10,  6), Point(5,  6)),
                   (Point( 0,  9), Point(7,  9)),
                   (Point(10, 12), Point(3, 12)),
                   (Point( 0,  5), Point(3,  5)),
                   (Point( 3,  3), Point(3, 10))))

slantmaze  = World(smallWorldMinPt, smallWorldMaxPt, 
                  bottomLeftStartPt, topRightGoalPt,
                  ((Point(0,   2), Point(7,  3)),
                   (Point(10,  5), Point(3,  6)),
                   (Point(0,   8), Point(7,  9)),
                   (Point(10, 11), Point(3, 12))))

hexagon    = World(smallWorldMinPt, smallWorldMaxPt, 
                  bottomLeftStartPt, topRightGoalPt,
                  ((Point(2.5, 5.5), Point(  3,   4)),
                   (Point(  3,   4), Point(  5,   4)),
                   (Point(  5,   4), Point(5.5, 5.5)),
                   (Point(5.5, 5.5), Point(  5,   7)),
                   (Point(  5,   7), Point(  3,   7)),
                   (Point(2.5, 5.5), Point(  3,   7))))

horWorldMinPt = Point(0,  0)
horWorldMaxPt = Point(15, 10)
horBottomLeftStartPt = Point(1, 1)
horTopRightGoalPt = Point(14, 9)

slantmazehor = World(horWorldMinPt, horWorldMaxPt, 
                  horBottomLeftStartPt, horTopRightGoalPt,
                  ((Point(2,   0), Point(3,  7)),
                   (Point(5,  10), Point(6,  3)),
                   (Point(8,   0), Point(9,  7)),
                   (Point(11, 10), Point(12, 3))))

hexagonhor  = World(horWorldMinPt, horWorldMaxPt, 
                  horBottomLeftStartPt, horTopRightGoalPt,
                  ((Point(5.5, 2.5), Point(  4,   3)),
                   (Point(  4,   3), Point(  4,   5)),
                   (Point(  4,   5), Point(5.5, 5.5)),
                   (Point(5.5, 5.5), Point(  7,   5)),
                   (Point(  7,   5), Point(  7,   3)),
                   (Point(5.5, 2.5), Point(  7,   3))))

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

# find your way out of a small hallway
bigmaze =      World(bigWorldMinPt, bigWorldMaxPt, 
                  bigStartPt, bigGoalPt,
                  ((Point(10,   0), Point(10, 50)),
                   (Point(0,   60), Point(15, 60)),
                   (Point(15,  60), Point(20, 55)),
                   (Point(20,  55), Point(20, 30)),
                   (Point(30,   0), Point(30, 70)),
                   (Point(70,  20), Point(70, 150)),
                   ))