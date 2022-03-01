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

'''
Aggregates visualization of each aspect of the space (mapping segments, points,
planning nodes, etc.).

STORES: Min/max bounds in x and y directions
'''
class Visualization():
    def __init__(self, maxPt, minPt = (0, 0)):
        # Clear and show.
        self.min      = minPt
        self.max      = maxPt
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

    '''
    Takes a list of segments to display, as well as a probability cutoff
    such that segments with probability below the cutoff are not displayed.
    '''
    def ShowSegments(self, segments, cutoff=0):
        # Show the segments, with opacity determined by probability.
        # Do not display segments with probabilities below the cutoff.
        for segment in segments:
            plt.plot([segment.seg[0][0], segment.seg[1][0]],
                     [segment.seg[0][1], segment.seg[1][1]],
                     'k', linewidth=2,
                     alpha = segment.prob * (segment.prob > cutoff))

    '''
    Takes the list of existing walls (in (point, point) form) and displays them.
    Also takes start and goal states (in point form) and displays checks.
    '''
    def ShowWorld(self, walls, start, goal):
        for wall in walls:
            plt.plot([wall[0][0], wall[1][0]],
                     [wall[0][1], wall[1][1]],
                     'r', linewidth=3)
        plt.plot(start[0], start[1], 'rx', markersize = 5)
        plt.plot( goal[0],  goal[1], 'gx', markersize = 5)

    '''
    Takes the location and orientation of the robot and displays it
    '''
    def ShowBot(self, loc, theta):
        plt.plot(loc[0], loc[1], marker=(3, 0, theta+90), markersize=10)

    '''
    Takes a list of points and displays them!
    '''
    def ShowPoints(self, points):
        for point in points:
            plt.plot(point.x, point.y, 'ko', markersize=5)

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

    # Report distance to another poimt (Euclidean)
    def dist(self, other):
        return np.sqrt((other.x - self.x)**2 + (other.y - self.y)**2)

    def getAngle(self, other):
        return np.atan2(p2[1] - p1[1], p2[0] - p1[0])

# The special Segment class carries a probability and is used only for mapping
# predicted segments. Walls are represented as pairs of points.
class Segment:
    def __init__(self, pt1, pt2, prob = 1):
        # Save the state matching this segment between points
        self.seg  = ((pt1.x, pt1.y), (pt2.x, pt2.y))
        self.prob = prob

    def SetProb(self, prob):
        self.prob = prob
