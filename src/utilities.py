#!/usr/bin/env python3
#
#   utilities.py
#
#   A Visualization class to help show how the robot is mapping, and a Segment
#   class to help store information about segments between nodes.
#   
#   visual = Visualization(segments)
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
        self.ClearFigure()
        self.ShowFigure()
        self.min      = minPt
        self.max      = maxPt

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
        # Define the region (axis limits).
        plt.gca().set_xlim(self.min[0], self.max[0])
        plt.gca().set_ylim(self.min[1], self.max[1])

        # Show the segments, with opacity determined by probability.
        # Do not display segments with probabilities below the cutoff.
        for segment in segment:
            plt.plot([segment.seg[0][0], segment.seg[1][0]],
                     [segment.seg[0][1], segment.seg[1][1]], 
                     'k', linewidth=2, 
                     alpha = segment.prob * (segment.prob > cutoff))

    def ShowFigure(self):
        # Show the plot.
        plt.pause(0.001)
    
class Segment:
    def __init__(self, Node1, Node2, prob):
        # Save the state matching this segment between nodes
        self.seg  = ((Node1.x, Node1.y), (Node2.x, Node2.y))
        self.prob = prob

    def SetProb(self, prob):
        self.prob = prob