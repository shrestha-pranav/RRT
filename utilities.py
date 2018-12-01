from __future__ import division
import math
import numpy as np
from matplotlib.path import Path

def get_obstacle_course(obstacle_path):
    vertices = list()
    codes = [Path.MOVETO]
    with open(obstacle_path) as f:
        quantity = int(f.readline())
        lines = 0
        for line in f:
            coordinates = tuple(map(int, line.strip().split(' ')))
            if len(coordinates) == 1:
                codes += [Path.MOVETO] + [Path.LINETO]*(coordinates[0]-1) + [Path.CLOSEPOLY]
                vertices.append((0,0)) #Always ignored by closepoly command
            else:
                vertices.append(coordinates)
    vertices.append((0,0))
    vertices = np.array(vertices, float)
    path = Path(vertices, codes)
    return path

def get_start_and_goal(start_goal_path):
    start, goal = None, None
    with open(start_goal_path) as f:
        start = tuple(map(int, f.readline().strip().split(' ')))
        goal  = tuple(map(int, f.readline().strip().split(' ')))
    
    return start, goal

def gen_next(q_near, q_rand, scale=20):
    # Generates q_new from 'q_near' at 'scale' units in direction of 'q_rand'
    vec  = q_rand[0] - q_near[0], q_rand[1]-q_near[1]
    norm = math.hypot(*vec)
    x = q_near[0] + vec[0] * scale / norm
    y = q_near[1] + vec[1] * scale / norm
    return (x, y)

# trees.py
class PathNode:
    def __init__(self, coords=None, parent=None):
        self.coords   = coords
        self.children = []
        self.parent   = parent
    
    __str__     = lambda self:    str(self.coords)
    __getitem__ = lambda self, x: self.coords[x]
    addChild    = lambda self, x: self.children.append(x)
    
class PathTree:
    """ Tree class for generating final path """
    def __init__(self, root):
        self.root   = PathNode(root)
        self.dict   = {root: self.root}
        self.length = 0

    def addPath(self, start, end):
        newNode   = PathNode(coords=end, parent=self.dict[start])
        self.dict[start].addChild(newNode)
        self.dict[end]  = newNode
        
    __getitem__  = lambda self, x: self.dict[x]
    __contains__ = lambda self, x: x in self.dict
        
    def printTree(self, node, depth=0, newNode=None):
        if node is None: return
        if newNode is not None:
            print(" | "*depth, node, "%0.4f"%hypot(newNode[0]-node[0], newNode[1]-node[1]))
        else:
            print(" | "*depth, node)

        for child in node.children:
            self.printTree(child, depth+1, newNode)