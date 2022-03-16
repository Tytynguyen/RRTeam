# TExplore.py
#
# Move to goal by exploring open spaces where walls likely don't exist, and moving closer to goal
#
# Wall nodes are given (direct sensor hits)
# Between nearest K nodes, wall segments exist with:
# 	Probability of being a wall (percent) 0-1, determined by distance from other node.
# Scheme:
# 	Sort wall segments by percent of being wall and distance from goal node
# 	Pick lowest cost (closest to goal node and lowest chance of being a wall)
# Try to move to point between both node

# TODO: Avoid recalculating probability every time node is added, somehow know when needs to be updated?

#DEBUG
from Map import *
from RRTStar import *
import numpy as np
from planarutils2 import *
from utilities import *
import worlds as worlds


from utilities import *
import numpy as np
from sklearn.neighbors import KDTree

K = 5   # Connect nearest K wall points
C_wall = 1  # Scale for wall probability of segment length
deadreckoning = 0.05

class TExplore:
    def __init__(self, goalPt, robot, map):
        # goal location
        self.goalPoint = goalPt

        # robot and map objects
        self.robot = robot
        self.map = map

        # world limits
        self.xmin = map.xlim[0]
        self.xmax = map.xlim[1]
        self.ymin = map.ylim[0]
        self.ymax = map.ylim[1]

    def update(self):
        """
        Make one "step" with robot
        Move robot to space between two wall nodes with lowest cost
        """
        # Remake roadmap #TODO: don't remake every time
        roadmap = []

        # Move to goal node if close
        distToGoal = self.robot.pos.dist(self.goalPoint)
        if random.uniform(0.0, 1.0) <= (1/(distToGoal+1)):
            # Pick goal as target
            self.robot.goto(self.goalPoint)
            return roadmap


        # Connect K nearest neighbors:
        X = np.array([(seg.getMidpoint().x, seg.getMidpoint().y) for seg in self.map.segments])
        kdt = KDTree(X)
        idx = kdt.query(X, k=(min([3,K]) + 1), return_distance=False)

        # Add the edges (from parent to child).  Ignore the first neighbor
        # being itself.
        for i, nbrs in enumerate(idx):
            for n in nbrs[1:]:
                seg1 = self.map.segments[i]
                seg2 = self.map.segments[n]

                midseg1 = Segment(seg1.pt1, seg2.pt1)
                midseg4 = Segment(seg1.pt1, seg2.pt2)

                for curmidseg in [midseg1, midseg4]:
                    if curmidseg.getLength() < 1:
                        continue

                    mpoint = curmidseg.getMidpoint()
                    newMoveNode = MoveNode(mpoint, self.calcWallProb(curmidseg), self.goalPoint)
                    roadmap.append(newMoveNode)

        # Move to lowest cost node
        nextnode = min(roadmap)
        self.robot.goto(nextnode.point)
        return roadmap


    def calcWallProb(self, segment):
        """
        Calculate the probability of there being a wall for a segment
        robotwidth 0.05
        """
        return 1 / (segment.getLength() + 1)



class MoveNode:
    """
    Store a node that can be moved to. Represents midpoints of wall segments, with cost. Also used for goal node
    """
    def __init__(self, point, wallprob, goalPoint):
        self.point = point
        self.cost = C_wall*wallprob + goalPoint.dist(self.point)

    def __lt__(self, other):
        return self.cost < other.cost


def main():
    ## SETUP
    # create a world (walls)
    walls = worlds.door

    minPt = [0, 0]
    maxPt = [10, 15]

    # define starting postion
    startPt = Point(1, 1)
    # define goal position
    goalPt = Point(9, 14)

    # create a map
    robotmap = Map(minPt, maxPt)

    # create a robot
    robot = Robot(walls, robotmap, Point(1, 1), 0)

    TExploreObj = TExplore(goalPt, robot, robotmap)

    # create viz
    visual = Visualization(walls, startPt, goalPt, maxPt)

    visual.ShowWorld()
    visual.ShowBot(robot)
    visual.ShowFigure()
    input("Initial world created. (hit return to continue)")


    while True:

        rm = TExploreObj.update()
        visual.ClearFigure()

        visual.ShowPoints([node.point for node in rm])
        visual.ShowBot(robot)
        visual.ShowFigure()

        if robot.pos == goalPt:
            input("Done!")
            return
        input("step")

if __name__== "__main__":
    main()



# DEPRECATED:
# class WallNode:
#     """
#     Store a wall node with:
#         point = Point(x,y)
#         neighborNodesandSegments = [(neighbor node, segment),...]
#     """
#     def __init__(self, point, neighborNodesandSegments = None):
#         self.point = point
#
#         # Store neighbor node and edge between them (node, seg)
#         if neighborNodesandSegments is None:
#             self.neighborNodesandSegments = []
#         else:
#             self.neighborNodesandSegments = neighborNodesandSegments
#
#
#     def addNeighbor(self, neighborWallNode, prob = None):
#         """
#         Add a neighbor node to this node and add this node to neighbor node
#         Also create segment between them and calculate its probability of being a wall
#         CALL ONCE PER NEIGHBOR (do not call for neighbor node as well)
#         """
#
#         # Make new segment between the two nodes, probability uncalculated
#         newSegment = Segment(self.point, neighborWallNode.point)
#
#         # If probability hasn't been given, calculate:
#         if prob is None:
#             prob = self.calcWallProb(newSegment)
#         newSegment.SetProb(prob)
#
#         # Add neighbor node and connecting segment to self.neighborNodesandSegments list
#         self.neighborNodesandSegments.append((neighborWallNode, newSegment))
#
#         # Add this node and segment to other node's neighborNodesandSegments list
#         neighborWallNode.neighborNodesandSegments.append((self, newSegment))
#
#     def clearNeighborNodes(self):
#         """
#         Delete all neighbor segments from this node, and remove this node from neighbor's neighbors list
#         """
#         # Remove this node from each neighbor's neighbors list
#         for (neighbornode,seg) in self.neighborNodesandSegments:
#             neighbornode.removeNeighborNode(self)
#
#         # Remove all neighbors from this node
#         self.neighborNodesandSegments = []
#
#     def removeNeighborNode(self, wallNode):
#         """
#         Remove a neighbor node from this node's neighborNodesandSegments list
#         """
#         # Search for the node in the list
#         for curi in range(len(self.neighborNodesandSegments)):
#             (neighborNode, seg) = self.neighborNodesandSegments[curi]
#
#             # If points match, remove
#             if neighborNode.point == wallNode.point:
#                 # Remove neighbor node from this node
#                 self.neighborNodesandSegments.pop(curi)
#                 # remove this node from neighbor wallNode list #TODO: Do not do unnecessary recursion on last step
#                 neighborNode.removeNeighborNode(self)
#                 return True
#
#         # Did not find node
#         return False
#
#     def calcWallProb(self, segment):
#         """
#         Calculate the probability of there being a wall for a segment
#         robotwidth 0.05
#         """
#         return 1/(segment.getLength()+1)