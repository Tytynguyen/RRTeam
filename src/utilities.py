#!/usr/bin/env python3
#
#   utilities.py
#
#   A Visualization class to help show how the robot is mapping, a Segment
#   class to help store information about segments between points, and a Point
#   class to encode points that have been seen.
#
#   visual = Visualization(maxPt, minPt)
#
#  The variables are:
#
#   segments      List of segments defining edges between Points.
#   cutoff        Probability cutoff below which segments are not displayed.

import matplotlib.pyplot as plt
import numpy as np
from RRTStar import *

'''
Aggregates visualization of each aspect of the space (mapping segments, points,
planning nodes, etc.).

STORES: Min/max bounds in x and y directions, 
        walls (list of (Point, Point) tuples), 
        start (Point), 
        goal (Point)
'''
class Visualization():
    def __init__(self, walls, start, goal, maxPt, minPt = (0, 0), ):
        # Clear and show.
        self.min      = minPt
        self.max      = maxPt
        self.walls    = walls
        self.start    = start
        self.goal     = goal
        self.ClearFigure()
        self.ShowFigure()

    '''
    Clear figure and prep space to draw something new
    '''
    def ClearFigure(self):
        # Clear the current, or create a new figure.
        plt.clf()

        # Create and prepare the axes.
        plt.axes()
        plt.gca().axis('on')
        plt.gca().set_xlim(self.min[0], self.max[0])
        plt.gca().set_ylim(self.min[1], self.max[1])
        plt.gca().set_aspect('equal')

        # Show the world
        self.ShowWorld()

    '''
    Takes a list of segments to display, as well as a probability cutoff
    such that segments with probability below the cutoff are not displayed.

    ARGS: segments - list of segments to print
    cutoff - probability below which segments are not displayed

    RETURNS: none
    '''
    def ShowSegments(self, segments, cutoff=0):
        # Show the segments, with opacity determined by probability.
        # Do not display segments with probabilities below the cutoff.
        for segment in segments:
            plt.plot([segment.pt1.x, segment.pt2.x],
                     [segment.pt1.y, segment.pt2.y],
                     'k', linewidth=2,
                     alpha = segment.prob * (segment.prob > cutoff))

    '''
    Takes the list of existing walls (in (Point, Point) form) and displays them.
    Also takes start and goal states (in Point form) and displays crosses.

    ARGS: None (uses walls, start, and goals properties)
    RETURNS: None
    '''
    def ShowWorld(self):
        for wall in self.walls:
            plt.plot([wall[0].x, wall[1].x],
                     [wall[0].y, wall[1].y],
                     'r', linewidth=3)
        plt.plot(self.start.x, self.start.y, 'rx', markersize = 5)
        plt.plot(self.goal.x,  self.goal.y,  'gx', markersize = 5)

    '''
    Takes the location and orientation of the robot and displays it.
    ARGS: loc - location of the bot (x, y)
    theta - orientation of the box, relative to positive x axis, in degrees
    RETURNS: None
    '''
    def ShowBot(self, loc, theta):
        plt.plot(loc.x, loc.y, marker=(3, 0, theta+90), markersize=10)

    '''
    Takes a point and displays it
    ARGS: point - Point object to display
    disp - string indicating how point should be displayed
    RETURNS: None
    '''
    def ShowPoint(self, point, disp):
        plt.plot(point.x, point.y, disp, markersize=5)


    '''
    Takes a list of points and displays them
    ARGS: points - list of Point objects
    RETURNS: None
    '''
    def ShowPoints(self, points):
        for point in points:
            # Points associated with walls are black
            self.ShowPoint(point, 'ko')


    '''
    Takes a list of nodes and displays their points
    ARGS: nodes - list of RRT Node objects
    RETURNS: None
    '''
    def ShowNodes(self, nodes):
        for node in nodes:
            # Points associated with RRT nodes are cyan
            self.ShowPoint(node.point, 'co')

    '''
    Shows the plot
    '''
    def ShowFigure(self):
        # Show the plot.
        plt.pause(0.001)

    #@@@@@@@@@@@@@@@@@@@ RRTStar Visualization @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

'''
Saves the coordinates of a point.

STORES: Coordinates of the point
'''
class Point:
    def __init__(self, x, y):
        # Save the state matching this point.
        self.x = x
        self.y = y

    def __add__(self, o):
        return Point(self.x + o.x, self.y + o.y)

    def __eq__(self, o):
        return self.x == o.x and self.y == o.y

    # Report distance to another poimt (Euclidean)
    def dist(self, other):
        return np.sqrt((other.x - self.x)**2 + (other.y - self.y)**2)

    def getAngle(self, other):
        return np.atan2(p2[1] - p1[1], p2[0] - p1[0])

#
'''
The special Segment class carries a probability and is used only for mapping
predicted segments. Walls are represented as pairs of points.

STORES: A tuple representing the two points, as well as each point
'''
class Segment:
    def __init__(self, pt1, pt2, prob = 1):
        # Save the state matching this segment between points
        self.seg  = ((pt1.x, pt1.y), (pt2.x, pt2.y))
        self.pt1  = pt1
        self.pt2  = pt2
        self.prob = prob

    def SetProb(self, prob):
        self.prob = prob
