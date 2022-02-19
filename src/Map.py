#   Map.py
#
#   Stores data structures for Map and Nodes

import numpy as np
from planarutils import *

class Node:
    def __init__(self, x, y, neighbors):
        # Save the state matching this node.
        self.x = x
        self.y = y

        # TODO: Figure out what to store here
        self.neighbors = neighbors  # {Segment, Segment, Segment...}

    # Report distance to another node (Euclidean)
    def Dist(self, other):
        return np.sqrt((other.x - self.x)**2 + (other.y - self.y)**2)

    # TODO: Report probability between 0-1 of a wall existing between two nodes
    def ProbWall(self, other):
        # Perhaps use Dist() as a metric? Low distance between two nodes -> high probability of wall
        # TODO: Problem if distance is less than 1! Fix
        return 1/self.Dist(other)

class Segment:
    def __init__(self, Node1, Node2):
        # TODO add option to use a provided probability to initialize instead of calculating probability between the nodes
        # Save the state matching this segment between nodes
        self.seg = ((Node1.x, Node1.y), (Node2.x, Node2.y))
        self.prob  = Node1.ProbWall(Node2)

    def SetProb(self, prob):
        self.prob = prob

# TODO: class Map

def TestVisualization():
    # Gimme some test segments to visualize.
    segments = () # TODO fill these out and decide how big space will be

def main():
    TestVisualization()

if __name__== "__main__":
    main()