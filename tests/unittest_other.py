strpts = lambda *pts: "\t".join(["(%.3f, %.3f)"%(x,y) for (x,y) in pts])

def find_nearest(V, p, verbose=False):
    """ DEPRACATED Brute force algorithm for nearest neighbor. """
    mindist, minq = float('inf'), None
    if verbose: print("Finding nearest to ", strpts(p))
    for v in V:
        dist = math.hypot(p[0]-v[0], p[1]-v[1])
        if dist < mindist:
            mindist = dist 
            minq = (v[0], v[1])
        if verbose: print(strpts(v, minq, (dist, mindist)))
    if verbose:
        print("Solution found ", strpts(minq), "%.4f"%math.hypot(minq[0]-p[0], minq[1]-p[1]))
        
    return minq, mindist