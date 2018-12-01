from __future__ import division

import math
import numpy as np

from KDTree    import KDTree
from obstacles import Obstacles
from utilities import gen_next, PathTree

def run(obstacles, start, goal, step_size, max_size, plotter):
    circ_rad  = min(step_size/5, 5)
    final_pos = np.array(goal[:2])

    # Pre-compute for generating new random points in Q_free
    minp, maxp   = obstacles.vertices.min(0), obstacles.vertices.max(0)
    span, offset = (maxp-minp)*1.1, minp-(maxp-minp)*0.05

    def gen_valid_rand(valid_function):
        """ Generates a valid q_rand in Q_free given a validity function """
        tmp     = np.random.random(2) * span + offset
        while not valid_function(*tmp):
            tmp = np.random.random(2) * span + offset
        return tmp

    KD    = KDTree(start)
    RRT   = PathTree(start)
    circ1 = plotter.draw_circle(start, 1, time=1, zorder=5)
    obstacles = Obstacles(obstacles.to_polygons())

    trials = 0
    while KD.length < max_size:
        trials += 1
        circ1.remove()

        # Select a random point q_rand \in Q_free
        q_rand = gen_valid_rand(obstacles.point_is_valid) if np.random.randint(0, 100)>5 else final_pos
        circ1 = plotter.draw_circle(q_rand, 5, time=0.01, zorder=5)
            
        # Find the nearest node and distance to it
        q_near, dist = KD.nearestNode(q_rand)
        
        # Generate the next node in the direction of q_rand
        if dist < step_size:
            if trials < 10: continue # Prevents step_size too big bug
            q_next = tuple(q_rand)
        else:
            q_next = gen_next(q_near, q_rand, step_size)
            if not obstacles.point_is_valid(*q_next): continue
        
        # Check validity and update tree
        if obstacles.check_collisions((q_near, q_next)): continue

        KD.addNode(q_next)
        RRT.addPath(q_near, q_next)

        plotter.draw_line(q_near, q_next, color='k', zorder=1, update=False)
        plotter.draw_circle(q_next, circ_rad, edgecolor='k', facecolor='w', zorder=2)

        if not obstacles.check_collisions((q_next, goal)):
            # IF there is a direct line to the goal, then TAKE IT
            goal_distance = math.hypot(q_next[0]-goal[0], q_next[1]-goal[1])
            while goal_distance > 0:
                q_new = gen_next(q_next, goal, min(goal_distance, step_size))
                RRT.addPath(q_next, q_new)
                plotter.draw_line(q_next, q_new, color='k', zorder=1, update=False)
                plotter.draw_circle(q_new, circ_rad, edgecolor='k', facecolor='w', zorder=2)
                q_next = q_new
                goal_distance -= step_size
            break

        trials = 0

    print("n =", KD.length)

    cur = RRT[goal]
    while cur.parent:
        plotter.draw_line(cur, cur.parent, update=False, color='b', zorder=3)
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