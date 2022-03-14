from utilities import *

# All worlds 10x15, start in bottom left, goal in top right

# Two triangles
triangles = ((Point(2,  4), Point(5,  9)),
             (Point(5,  9), Point(4,  4)),
             (Point(4,  4), Point(2,  4)),
             (Point(2, 12), Point(9, 12)),
             (Point(2, 14), Point(9, 12)),
             (Point(2, 12), Point(2, 14)),
             (Point(0,  0), Point(0, 15)),
             (Point(0, 15), Point(10,15)),
             (Point(10,15), Point(10, 0)),
             (Point(10, 0), Point( 0, 0)))

# A vertical and horizontal line
simple =    ((Point(4,  0), Point(4,  6)),
             (Point(4, 10), Point(9, 10)),
             (Point(0,  0), Point(0, 15)),
             (Point(0, 15), Point(10,15)),
             (Point(10,15), Point(10, 0)),
             (Point(10, 0), Point( 0, 0)))


# Obstruction forces robot to find an unintuitive door
door =      ((Point(0,   2), Point(8,  2)),
             (Point(8,   2), Point(8,  8)),
             (Point(10, 10), Point(2, 10)),
             (Point(0,  0), Point(0, 15)),
             (Point(0, 15), Point(10,15)),
             (Point(10,15), Point(10, 0)),
             (Point(10, 0), Point( 0, 0)))
