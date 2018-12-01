from math import hypot

class Node:
    def __init__(self, node=None, alpha=0, left=None, right=None):
        self.node = node
        self.alpha = alpha
        self.left = left
        self.right = right
    __str__     = lambda self:    str(self.node)
    __getitem__ = lambda self, x: self.node[x]

class KDTree:
    def __init__(self, root, alpha=None):
        self.root   = Node(root, alpha)
        self.length = 0
        
    def addNode(self, new, alpha=None):
        axis = 0
        cur = self.root
        if not isinstance(new, Node): new = Node(new, alpha)
        
        while True:
            if new[axis] < cur[axis]:
                if cur.left is None:
                    cur.left = new
                    break
                cur = cur.left
                
            else:
                if cur.right is None:
                    cur.right = new
                    break
                cur = cur.right
            
            axis = 1 - axis
        self.length += 1
    
    def nearestNode(self, new, alpha=None, return_node=False):
        new = Node(new, alpha)
        node, dist = self._nearestNode(new, self.root, 0, None, float('inf'))
        return node if return_node else tuple(node.node), dist

    def _nearestNode(self, new, cur, axis, minNode, minDist):
        dist = hypot(new[0]-cur[0], new[1]-cur[1])
        if dist < minDist:
            minNode, minDist = cur, dist

        if new[axis] < cur[axis]:
            if cur.left is not None:
                minNode, minDist = self._nearestNode(new, cur.left, 1-axis, minNode, minDist)
            
            if cur.right is not None and cur[axis] - new[axis] < minDist:
                minNode, minDist = self._nearestNode(new, cur.right, 1-axis, minNode, minDist)
            
        else:
            if cur.right is not None:
                minNode, minDist = self._nearestNode(new, cur.right, 1-axis, minNode, minDist)

            if cur.left is not None and new[axis] - cur[axis] < minDist:
                minNode, minDist = self._nearestNode(new, cur.left, 1-axis, minNode, minDist)
        
        return minNode, minDist
        
    def __str__(self):
        self.printTree(self.root, 0)
        return ""
    
    def printTree(self, node, depth=0, newNode=None):
        if node is None: return
        if newNode is not None:
            print(" | "*depth, node, "%0.4f"%hypot(newNode[0]-node[0], newNode[1]-node[1]))
        else:
            print(" | "*depth, node)
        self.printTree(node.left, depth+1, newNode)
        self.printTree(node.right, depth+1, newNode)