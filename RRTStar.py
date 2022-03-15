#
#   RRTStar.py
#
#   Search algorithm based upon D* utilizing RRT sampling, developed for dynamic map generation
#
#   Procedure:
#   Given the location of the robot and the goal,
#   and given a map of the current know world barrier of nodes and probability of a wall existing between a node:
#
#   Start all EDGES to ONDECK
#
#   Loop:
#       if ONDECK = empty -> Failure
#       Pick lowest cost on deck
#           Cost = distance to goal and wall probability metric
#
#           Goto(ONDECK.xcenter,ONDECK.ycener)
#
#           If can't go to center of edge
#               Update edge probability
#

import random

import numpy as np
# from planarutils2 import *
from utilities import *

dstep = 1
deadreckoning = 0.05
Nmax = 1000


class RRTNode:
    """
    Tree node class that stores parent information and the point vector
    """
    def __init__(self, point, childrennodes, parentnode=None):
        self.point = point
        self.parent = parentnode
        self.children = childrennodes

    def __eq__(self, other):
        return self.point == other.point

    def __repr__(self):
        return "(" + str(self.point) + ")"

class RRTStar:

    def __init__(self, robotPt, goalPt, robot, map, minPt, maxPt):
        # define class variables
        self.robotPoint = robotPt
        self.robotNode = RRTNode(self.robotPoint, [])
        self.goalPoint = goalPt
        self.robot = robot
        self.map = map
        self.tree = [RRTNode(self.goalPoint, [])]
        self.minPt = minPt
        self.maxPt = maxPt
        self.newpath = True

        # class variables for viz

    # RETURN GOAL NODE
    def update(self):
        if self.newpath:
            robotNode = self.RRT(self.robotPoint,Nmax,self.minPt[0],self.maxPt[0],self.minPt[1],self.maxPt[1],self.map)

            # If too many nodes, we fail
            if robotNode is None:
                return None
            self.newpath = False
            # print(len(self.tree))

            return robotNode
        else:
            # print(self.robot.pos)
            return self.TStar(self.robot)

    def RRT(self, robotpoint, Nmax, xmin, xmax, ymin, ymax, mapobj):
        """
        Generate a RRT for use in planning.

        @param tree List of all tree nodes
        @param startpoint Where tree should start growing from (The end goal)
        @param goalpoint Where tree should attempt to grow to (The robot)
        @param Nmax Maximum number of nodes in tree allowed
        @param xmin
        @param xmax
        @param ymin
        @param ymax

        @return tree list of nodes
        """

        # Loop.
        while True:
            # Determine the target point.
            # Try directly towards the goal by deadreckoning some times, but otherwise
            # Choose a uniformly random target
            if random.uniform(0.0, 1.0) < deadreckoning:
                # Pick goal as target
                targetpoint = self.robot.pos
            else:
                # Uniformly pick target
                x = random.uniform(xmin, xmax)
                y = random.uniform(ymin, ymax)
                targetpoint = Point(x, y)


            # Find the nearest node TODO: Make this more efficient...
            list = [(node.point.dist(targetpoint), node) for node in self.tree]
            (d2, nearnode)  = min(list)

            nearpoint = nearnode.point

            # Check if the node already connects...
            if mapobj.localPlanner(nearpoint, robotpoint):
                robotnode = RRTNode(robotpoint, [], nearnode)
                self.tree.append(robotnode)

                if len(nearnode.children) == 0:
                    nearnode.children = [robotnode]
                else:
                    nearnode.children.append(robotnode)
                self.robotNode = robotnode
                return robotnode

            # Determine the next point, a step size (dstep) away.
            t = np.arctan2((targetpoint.y - nearpoint.y), (targetpoint.x - nearpoint.x))
            nx = dstep*np.cos(t) + nearpoint.x
            ny = dstep*np.sin(t) + nearpoint.y
            nextpoint = Point(nx, ny)
            nextnode = RRTNode(nextpoint, [], nearnode)

            # Check whether nearpoint connects to next generated point
            if mapobj.localPlanner(nearpoint, nextpoint):
                self.tree.append(nextnode)

                if len(nearnode.children) == 0:
                    nearnode.children = [nextnode]
                else:
                    nearnode.children.append(nextnode)

                nextnode.parent = nearnode

                # Also try to connect the goal.
                if mapobj.localPlanner(nextpoint, robotpoint):
                    robotnode = RRTNode(robotpoint, [], nextnode)
                    self.tree.append(robotnode)

                    if len(nextnode.children) == 0:
                        nextnode.children = [robotnode]
                    else:
                        nextnode.children.append(robotnode)

                    self.robotNode = robotnode

                    return robotnode

            # Abort if tree is too large
            if (len(self.tree) >= Nmax):
                return None


    def TStar(self, robot):
        """
        Run TStar given a start node and end node, using RRT to generate a tree.
        """
        # Build initial tree

        path = self.getPathNodes(self.robotNode)  # Get the path from the start node to goal
        path.append(self.tree[0])
        curnode = path[1]

        # Fails to make it
        if not robot.goto(curnode.point):
            p = robot.pos

            # Kill previous node, no longer can get there
            self.killNode(path[0])

            # Make new RRT
            self.robotNode = RRTNode(p, [])
            self.robotPoint = p
            self.newpath = True
            return self.robotNode
        else:
            p = robot.pos
            self.robotNode = curnode

            self.killNode(path[0])

            self.robotPoint = p
            self.newpath = False
            return self.robotNode

    '''
    Return segments for every parent-child connection in the whole darn tree
    '''
    def getTreeSegments(self):
        basenode = self.tree[0]
        segmentList = []
        self.getChildSegments(basenode, segmentList)
        return segmentList

    '''
    Add to segmentList in place for every child of the given node, recursively
    '''
    def getChildSegments(self, parent, segmentList):
        if parent.children == []:
            return

        for child in parent.children:
            segmentList.append(Segment(child.point, parent.point))
            self.getChildSegments(child, segmentList)

    def getPathSegments(self, node):
        """
        Get the segments from the given node to the base of the tree
        """
        segments = []

        while node.parent is not None:
            segments.append(Segment(node.point, node.parent.point))
            node = node.parent
        return segments

    def getPathNodes(self, node):
        """
        Get the nodes from the given node to the base of the tree
        """
        nodes = []

        while node.parent is not None:
            nodes.append(node)
            node = node.parent

        return nodes

    def getPoints(self, tree, list=[]):
        list.append(tree)
        for child in tree.children:
            self.getPoints(tree, list)

        return list

    def killNode(self, node):
        # Kill all of the node's kids!

        for child in node.children:
            self.killNode(child)

        # Kill node by removing it from the children list of its parent
        parent = node.parent
        parent.children.remove(node)

        # Also remove node from tree list:
        self.tree.remove(node)


    # def getAllChildren(self, node, nodelist = []):
    #     """
    #     Get all nodes that are children, grandchildren, etc. of a specific node
    #     """
    #     nodelist.append(node.children)
    #     for child in node.children:
    #         self.getAllChildren(child, nodelist)
    #
    #     return nodelist
