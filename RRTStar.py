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
from planarutils import *
from utilities import *

dstep = 0.5
deadreckoning = 0.05
Nmax = 1000


class RRTNode:
    """
    Tree node class that stores parent information and the point vector
    """
    def __init__(self, point, parentnode=None, childrennodes=None):
        self.point = point
        self.parent = parentnode
        self.children = childrennodes


class RRTStar:

    def __init__(self, robotPt, goalPt, robot, map, minPt, maxPt):
        # define class variables
        self.robotPoint = robotPt
        self.robotNode = RRTNode(self.robotPoint)
        self.goalPoint = goalPt
        self.robot = robot
        self.map = map
        self.tree = [RRTNode(self.goalPoint)]
        self.minPt = minPt
        self.maxPt = maxPt
        self.newpath = True

        # class variables for viz

    # RETURN GOAL NODE
    def update(self):
        if self.newpath:
            robotNode = self.RRT(self.robotPoint,Nmax,self.minPt[0],self.maxPt[0],self.minPt[1],self.maxPt[1],self.map)
            self.newpath = False
            # print(len(self.tree))

            return robotNode
        else:
            # print(self.robot.pos)
            return self.TStar(self.tree, self.robot)

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
        #TODO: Check if goal connects to start

        # Loop.
        while True:
            # Determine the target point.
            # Try directly towards the goal by deadreckoning some times, but otherwise
            # Choose a uniformly random target
            if random.uniform(0.0, 1.0) <= deadreckoning:
                # Pick goal as target
                targetpoint = robotpoint
            else:
                # Uniformly pick target
                x = random.uniform(xmin, xmax)
                y = random.uniform(ymin, ymax)
                targetpoint = Point(x, y)

            # Find the nearest node TODO: Make this more efficient...
            list = [(node.point.dist(targetpoint), node) for node in self.tree]
            (d2, nearnode)  = min(list)

            nearpoint = nearnode.point

            # Determine the next point, a step size (dstep) away.
            t = np.arctan2((targetpoint.y - nearpoint.y), (targetpoint.x - nearpoint.x))
            nx = dstep*np.cos(t) + nearpoint.x
            ny = dstep*np.sin(t) + nearpoint.y
            nextpoint = Point(nx, ny)
            nextnode = RRTNode(nextpoint, nearnode)

            # Check whether nearpoint connects to next generated point
            if mapobj.localPlanner(nearpoint, nextpoint):
                self.tree.append(nextnode)

                if nearnode.children is None:
                    nearnode.children = [nextnode]
                else:
                    nearnode.children.append(nextnode)

                nextnode.parent = nearnode

                # Also try to connect the goal.
                if mapobj.localPlanner(nextpoint, robotpoint):
                    robotnode = RRTNode(robotpoint, nextnode)
                    self.tree.append(robotnode)

                    if nextnode.children is None:
                        nextnode.children = [robotnode]
                    else:
                        nextnode.children.append(robotnode)
                    self.robotNode = robotnode
                    return robotnode

            # Abort if tree is too large
            if (len(self.tree) >= Nmax):
                return None


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
            getPoints(tree,list)

        return list

    def TStar(self, tree, robot):
        """
        Run TStar given a start node and end node, using RRT to generate a tree.
        """
        # Build initial tree
        path = self.getPathNodes(self.robotNode)  # Get the path from the start node to goal

        for curnodei in range(1,len(path)):
            curnode = path[curnodei]

            # Fails to make it
            if not robot.goto(curnode.point):
                p = robot.pos
                # Kill previous node
                prevnode = path[curnodei-1]
                prevpoint = prevnode.point

                # Kill previous node by removing it from the children list of its parent
                for curchildi in range(len(curnode.children)):
                    curpoint = curnode.children[curchildi].point
                    if curpoint.x == prevpoint.x and curpoint.y == prevpoint.y:
                        curnode.children.pop(curchildi)

                # Also remove node from tree list:
                for curnodei in range(len(tree)):
                    curpoint = tree[curnodei].point
                    if curpoint.x == prevpoint.x and curpoint.y == prevpoint.y:
                        tree.pop(curnodei)

                # Make new RRT
                self.newpath = True
                self.robotPoint = p
                self.robotNode = RRTNode(p)
                return None

            else:
                return path

def TestVisualization():
    # Generate example world with walls
    walls = ((Point(2,  4), Point(5,  9)),
             (Point(5,  9), Point(4,  4)),
             (Point(4,  4), Point(2,  4)),
             (Point(2, 12), Point(9, 12)),
             (Point(2, 14), Point(9, 12)),
             (Point(2, 12), Point(2, 14)))

    start = Point(1, 1)
    goal  = Point(9, 14)
    minPt = (0, 0)   # (xmin, ymin)
    maxPt = (10, 15) # (xmax, ymax)
    mapobj = Map(minPt, maxPt)

    # Gimme some test segments to visualize.
    points = (Point(2, 4), Point(4, 4), Point(8, 12), Point(5, 9))
    segments = (Segment(points[0], points[1], 0.46),
                Segment(points[1], points[2], 0.63),
                Segment(points[0], points[2], 0.20),
                Segment(points[3], points[2], 0.83),
                Segment(points[1], points[3], 1))

    visual = Visualization(walls, start, goal, (10, 15))

    visual.ShowWorld()
    visual.ShowBot(start, 0)
    visual.ShowPoints(points)
    visual.ShowSegments(segments)
    visual.ShowFigure()
    input("ProbSegment test displayed. (hit return to continue)")
    visual.ClearFigure()
    visual.ShowFigure()
    input("World cleared. (hit return to continue)")

    # TEST RRT
    # Start the tree with start state and no parent
    # execute the search
    tree = [RRTNode(start, None, None)]
    tree = RRT(tree, goal, Nmax, minPt[0], minPt[1], maxPt[0], maxPt[1], mapobj) # TODO try to reduce arguments to RRT to maxPt, minPt form

    if tree is None:
        print("UNABLE TO FIND A PATH in %d steps", Nmax)
        input("(hit return to exit)")
        return

    # Show the path.
    visual.ShowNodes(tree)
    print("PATH found after ", len(tree), " samples.")
    input("(hit return to exit)")
    return

def main():
    TestVisualization()

if __name__== "__main__":
    main()
