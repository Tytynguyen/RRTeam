#!/usr/bin/env python3
#
#   utilities.py
#
#   A Visualization class to help show how the robot is mapping, a Segment
#   class to help store information about segments between nodes, and a Node
#   class to encode points that have been seen.
#
#   visual = Visualization(maxPt, minPt)
#
#  The variables are:
#
#   segments      List of segments defining edges between Nodes.
#   cutoff        Probability cutoff below which segments are not displayed.

import matplotlib.pyplot as plt
import numpy as np

#
# Probability Segment Plotting
#
class Visualization():
    def __init__(self, maxPt, minPt = (0, 0)):
        # Clear and show.
        self.min      = minPt
        self.max      = maxPt
        self.ClearFigure()
        self.ShowFigure()

    def ClearFigure(self):
        # Clear the current, or create a new figure.
        plt.clf()

        # Create and prepare the axes.
        plt.axes()
        plt.gca().axis('on')
        plt.gca().set_xlim(self.min[0], self.max[0])
        plt.gca().set_ylim(self.min[1], self.max[1])
        plt.gca().set_aspect('equal')

    # Takes a list of segments to display, as well as a probability cutoff
    # such that segments with probability below the cutoff are not displayed.
    def ShowSegments(self, segments, cutoff=0):
        # Show the segments, with opacity determined by probability.
        # Do not display segments with probabilities below the cutoff.
        for segment in segments:
            plt.plot([segment.seg[0][0], segment.seg[1][0]],
                     [segment.seg[0][1], segment.seg[1][1]],
                     'k', linewidth=2,
                     alpha = segment.prob * (segment.prob > cutoff))

    # Takes the list of existing walls (in (point, point) form) and displays them.
    # Also takes start and goal states (in point form) and displays checks.
    def ShowWorld(self, walls, start, goal):
        for wall in walls:
            plt.plot([wall[0][0], wall[1][0]],
                     [wall[0][1], wall[1][1]],
                     'r', linewidth=3)
        plt.plot(start[0], start[1], 'rx', markersize = 5)
        plt.plot( goal[0],  goal[1], 'gx', markersize = 5)

    # Takes the location and orientation of the robot and displays it
    def ShowBot(self, loc, theta):
        plt.plot(loc[0], loc[1], marker=(3, 0, theta-90), markersize=10)

    # Takes a list of nodes and displays them!
    def ShowNodes(self, nodes):
        for node in nodes:
            plt.plot(node.x, node.y, 'ko', markersize=5)

    def ShowFigure(self):
        # Show the plot.
        plt.pause(0.001)

# TODO: may not be necessary
class Node:
    def __init__(self, x, y):
        # Save the state matching this node.
        self.x = x
        self.y = y

    # Report distance to another node (Euclidean)
    def Dist(self, other):
        return np.sqrt((other.x - self.x)**2 + (other.y - self.y)**2)

    # TODO: Report probability between 0-1 of a wall existing between two nodes
    def ProbWall(self, other):
        # Perhaps use Dist() as a metric? Low distance between two nodes -> high probability of wall
        # TODO: Problem if distance is less than 1! Fix
        return 1/self.Dist(other)

# The special Segment class carries a probability and is used only for mapping
# predicted segments. Walls are represented as pairs of points.
class Segment:
    def __init__(self, Node1, Node2, prob):
        # Save the state matching this segment between nodes
        self.seg  = ((Node1.x, Node1.y), (Node2.x, Node2.y))
        self.prob = prob

    def SetProb(self, prob):
        self.prob = prob
