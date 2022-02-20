#   Map.py
#
#   Stores data structures for Map and Nodes

import numpy as np
from planarutils import *
from utilities import *

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



# TODO: class Map

def TestVisualization():
    # Gimme some test segments to visualize.
    nodes = (Node(2, 5), Node(4, 2), Node(8, 12))
    segments = (Segment(nodes[0], nodes[1], 0.46),
                Segment(nodes[1], nodes[2], 0.98),
                Segment(nodes[0], nodes[2], 0.20))
    visual = Visualization((10, 15))
    visual.ShowSegments(segments)
    visual.ShowFigure()
    input("Test displayed.")
        
def main():
    TestVisualization()

if __name__== "__main__":
    main()