#   Map.py
#
#   Stores data structures for Map and Nodes

import numpy as np
from planarutils import *
from utilities import *




# TODO: class Map

def TestVisualization():
    # Generate example world with walls
    walls = (((2,  4), (5,  9)),
             ((5,  9), (4,  4)),
             ((4,  4), (2,  4)),
             ((2, 12), (9, 12)),
             ((2, 14), (9, 12)),
             ((2, 12), (2, 14)))

    start = (1, 1)
    goal  = (9, 14)
    
    # Gimme some test segments to visualize.
    nodes = (Node(2, 4), Node(4, 4), Node(8, 12), Node(5, 9))
    segments = (Segment(nodes[0], nodes[1], 0.46),
                Segment(nodes[1], nodes[2], 0.63),
                Segment(nodes[0], nodes[2], 0.20),
                Segment(nodes[3], nodes[2], 0.83),
                Segment(nodes[1], nodes[3], 1))
    visual = Visualization((10, 15))
    visual.ShowWorld(walls, start, goal)
    visual.ShowBot(start, 0)
    visual.ShowNodes(nodes)
    visual.ShowSegments(segments)
    visual.ShowFigure()
    input("Test displayed.")
        
def main():
    TestVisualization()

if __name__== "__main__":
    main()