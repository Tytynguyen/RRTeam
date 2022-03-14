# TExplore.py
#
# Move to goal by exploring open spaces where walls likely don't exist, and moving closer to goal
#
# Wall nodes are given (direct sensor hits)
# Between nearest K nodes, wall segments exist with:
# 	Probability of being a wall (percent) 0-1, determined by distance from other node.
# Scheme:
# 	Sort wall segments by percent of being wall and distance from goal node
#		Try cost = c1*wallchance + c2*d_goal ? or other mathematical scheme
# 	Pick lowest cost (closest to goal node and lowest chance of being a wall)
# Try to move to point between both node

# TODO: Avoid recalculating probability every time node is added, somehow know when needs to be updated?

#DEBUG
from Map import *
from worlds import *
from RRTStar import *

from utilities import *

K = 5   # Connect nearest K wall points
C_wall = 1  # Scale for wall probability of segment length

class WallNode:
    """
    Store a wall node with:
        point = Point(x,y)
        neighborNodesandSegments = [(neighbor node, segment),...]
    """
    def __init__(self, point, neighborNodesandSegments = None):
        self.point = point

        # Store neighbor node and edge between them (node, seg)
        if neighborNodesandSegments is None:
            self.neighborNodesandSegments = []
        else:
            self.neighborNodesandSegments = neighborNodesandSegments


    def addNeighbor(self, neighborWallNode, prob = None):
        """
        Add a neighbor node to this node and add this node to neighbor node
        Also create segment between them and calculate its probability of being a wall
        CALL ONCE PER NEIGHBOR (do not call for neighbor node as well)
        """

        # Make new segment between the two nodes, probability uncalculated
        newSegment = Segment(self.point, neighborWallNode.point)

        # If probability hasn't been given, calculate:
        if prob is None:
            prob = self.calcWallProb(newSegment)
        newSegment.SetProb(prob)

        # Add neighbor node and connecting segment to self.neighborNodesandSegments list
        self.neighborNodesandSegments.append((neighborWallNode, newSegment))

        # Add this node and segment to other node's neighborNodesandSegments list
        neighborWallNode.neighborNodesandSegments.append((self, newSegment))

    def clearNeighborNodes(self):
        """
        Delete all neighbor segments from this node, and remove this node from neighbor's neighbors list
        """
        # Remove this node from each neighbor's neighbors list
        for (neighbornode,seg) in self.neighborNodesandSegments:
            neighbornode.removeNeighborNode(self)

        # Remove all neighbors from this node
        self.neighborNodesandSegments = []

    def removeNeighborNode(self, wallNode):
        """
        Remove a neighbor node from this node's neighborNodesandSegments list
        """
        # Search for the node in the list
        for curi in range(len(self.neighborNodesandSegments)):
            (neighborNode, seg) = self.neighborNodesandSegments[curi]

            # If points match, remove
            if neighborNode.point == wallNode.point:
                # Remove neighbor node from this node
                self.neighborNodesandSegments.pop(curi)
                # remove this node from neighbor wallNode list #TODO: Do not do unnecessary recursion on last step
                neighborNode.removeNeighborNode(self)
                return True

        # Did not find node
        return False

    def calcWallProb(self, segment):
        """
        Calculate the probability of there being a wall for a segment
        robotwidth 0.05
        """
        return 1/(segment.getLength()+1)

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

        self.wallNodes = []    # Store the wallNodes

        # Initialize wall corner nodes assuming four points/rectangular world
        node_bot_left = WallNode(Point(self.xmin, self.ymin))
        node_top_left = WallNode(Point(self.xmin, self.ymax))
        node_bot_right = WallNode(Point(self.xmax, self.ymin))
        node_top_right = WallNode(Point(self.xmax, self.ymax))

        # add wall neighbors to corners
        node_bot_left.addNeighbor(node_bot_right, 1)    # TODO: Why is this adding the neighbor twice????

        node_bot_left.addNeighbor(node_top_left, 1)
        node_top_right.addNeighbor(node_top_left, 1)
        node_top_right.addNeighbor(node_bot_right, 1)

        # add diagonals, with non-1 probability
        node_bot_left.addNeighbor(node_top_right)
        node_top_left.addNeighbor(node_bot_right)

        # add to wallNodes list
        self.wallNodes.append(node_bot_left)
        self.wallNodes.append(node_top_left)
        self.wallNodes.append(node_bot_right)
        self.wallNodes.append(node_top_right)


    # # TODO: Use KDtree to connect nearest neighbors
    # #
    # Connect the nearest K neighbors
    # #
    # def ConnectNearestNeighbors(nodeList, K):
    #     # Clear any existing neighbors.
    #     for node in nodeList:
    #         node.childrenandcosts = []
    #         node.parents = []
    #
    #     # Determine the indices for the nearest neighbors.  This also
    #     # reports the node itself as the closest neighbor, so add one
    #     # extra here and ignore the first element below.
    #     X = np.array([node.state.Coordinates() for node in nodeList])
    #     kdt = KDTree(X)
    #     idx = kdt.query(X, k=(K + 1), return_distance=False)
    #
    #     # Add the edges (from parent to child).  Ignore the first neighbor
    #     # being itself.
    #     for i, nbrs in enumerate(idx):
    #         # print(i)
    #         children = [child for (child, _) in nodeList[i].childrenandcosts]
    #         for n in nbrs[1:]:
    #             if not nodeList[n] in children:
    #                 plan = LocalPlan(nodeList[i].state, nodeList[n].state)
    #                 if plan.Valid():
    #                     cost = plan.Length()
    #                     nodeList[i].childrenandcosts.append((nodeList[n], cost))
    #                     nodeList[n].childrenandcosts.append((nodeList[i], cost))
    #                     nodeList[n].parents.append(nodeList[i])
    #                     nodeList[i].parents.append(nodeList[n])


def main():
    ## SETUP
    # create a world (walls)
    walls = worlds.simple

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
    print("done")

if __name__== "__main__":
    main()