from __future__ import division

import math
import numpy as np

from KDTree    import KDTree
from obstacles import Obstacles
from utilities import gen_next, PathTree

def run(obstacles, start, goal, step_size, max_size, plotter):
    circ_rad = min(step_size/5, 5)
    final_pos = [np.array(goal), np.array(start)]

    # Pre-compute for generating new random points in Q_free
    minp, maxp   = obstacles.vertices.min(0), obstacles.vertices.max(0)
    span, offset = (maxp-minp)*1.1, minp-(maxp-minp)*0.05

    def gen_valid_rand(valid_function):
        """ Generates a valid q_rand in Q_free given a validity function """
        tmp     = np.random.random(2) * span + offset
        while not valid_function(*tmp):
            tmp = np.random.random(2) * span + offset
        return tmp

    KD    = [KDTree(start), KDTree(goal)]
    RRT   = [PathTree(start), PathTree(goal)]
    n     = 1
    rnd_display = False
    obstacles = Obstacles(obstacles.to_polygons())

    """
    • Expand tree T_1 randomly, add node q_new
    • Expand T_2 towards q_new
        • If tree T_2 connects to q_new, path formed else add a q_new for tree T_2
    • Now expand T_1 to q_new in tree T_2
    • Keep swapping T_1 and T_2 for expansion towards the
    other tree until they meet
    """

    trials = 0
    q_new, last_expanded = None, -1
    while KD[0].length + KD[1].length < 1000:
        trials += 1
        if rnd_display: circ1.remove(); rnd_display = False
        n = 1 - n
        
        # If the last expanded node was in the other tree, try expanding towards q_new
        if last_expanded != n and q_new is not None:
            q_near, dist = KD[n].nearestNode(q_new)
            q_next = gen_next(q_near, q_new, step_size) if dist>step_size else q_new
            
            # Expansion towards q_new is possible. Add to path and goal check
            if obstacles.point_is_valid(*q_next) and \
                not obstacles.check_collisions((q_near, q_next)):
                RRT[n].addPath(q_near, q_next)
                KD[n].addNode(q_next)
                
                plotter.draw_circle(q_next, circ_rad, edgecolor='k', facecolor='w', zorder=1)
                plotter.draw_line(q_near, q_next, color='kb'[n], zorder=1)

                if q_next == q_new: break # Path found
                q_new, last_expanded, trials = q_next, n, 0 # Update for next iteration
                continue

        # If last expanded node was not in the other tree or expansion to q_new not possible
        # Try to expand to q_rand if possible
        q_rand = gen_valid_rand(obstacles.point_is_valid) if np.random.randint(0,100)>5 else final_pos[n]
        rnd_display, circ1 = True, plotter.draw_circle(q_rand, 5, zorder=5)

        q_near, dist = KD[n].nearestNode(q_rand)
        if dist < step_size:
            if trials < 10: continue
            q_next = tuple(q_rand)
        else:
            q_next = gen_next(q_near, q_rand, step_size)
            if not obstacles.point_is_valid(*q_next): continue
        
        if obstacles.check_collisions((q_near, q_next)): continue

        KD[n].addNode(q_next)
        RRT[n].addPath(q_near, q_next)

        plotter.draw_line(q_near, q_next, color='kb'[n], zorder=1)
        plotter.draw_circle(q_next, circ_rad, edgecolor='k', facecolor='w', zorder=1)

        q_new, last_expanded, near_count = q_next, n, 0

    print("n =", KD[0].length + KD[1].length, "(%d, %d)"%(KD[0].length, KD[1].length))

    # Plot out goal path
    cur = RRT[0][tuple(q_next)]
    while cur.parent:
        plotter.draw_line(cur, cur.parent, update=False, color='y', zorder=3)
        plotter.draw_circle(cur, circ_rad*1.5, update=False, facecolor='xkcd:green', edgecolor='k', zorder=4)
        cur = cur.parent

    cur = RRT[1][tuple(q_next)]
    while cur.parent:
        plotter.draw_line(cur, cur.parent, update=False, color='y', zorder=3)
        plotter.draw_circle(cur, circ_rad*1.5, update=False, facecolor='xkcd:green', edgecolor='k', zorder=4)
        cur = cur.parent
    plotter.update()

if __name__ == '__main__':
    from ImageGenerator import ImageGenerator
    from utilities      import get_obstacle_course, get_start_and_goal

    obstacles   = get_obstacle_course("world_obstacles.txt")
    start, goal = get_start_and_goal("start_goal.txt")

    plotter    = ImageGenerator()
    plotter.draw_obstacle_course(obstacles)
    plotter.draw_start_and_goal(start,goal)

    run(obstacles, start, goal, 50, 2000, plotter)

    input("Press enter to exit : ")