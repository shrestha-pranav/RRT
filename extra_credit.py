from __future__ import division

import math
import numpy as np

from KDTree    import KDTree
from obstacles import Obstacles
from utilities import gen_next

init_t = math.atan(2.5)
r      = math.hypot(10, 25)
thetas = [init_t, math.pi-init_t, math.pi+init_t, 2*math.pi-init_t]
def gen_rect_pts(x, y, alpha):
    return [(x+r*math.cos(theta+alpha), y+r*math.sin(theta+alpha))
               for theta in thetas]

def _check_collision(obstacles, verts):
    """ For each side of the rectangle, check if it intersects an obstacle """ 
    for side in range(4):
        if obstacles.check_collisions((verts[side], verts[(side+1)%4])):
            return True
    return False

def check_collision(obstacles, start, end, distance):
    # Check for collision every 5 units translation or every 10 degrees rotation
    steps      = max(int(distance/5), int(abs(end[-1]-start[-1])/math.radians(10)))
    x,  y,  a  = start
    x2, y2, a2 = end

    final_state = gen_rect_pts(x2, y2, a2)
    if _check_collision(obstacles, final_state): return True
    
    if steps == 0: return False
    dx, dy, da = (x2-x)/steps, (y2-y)/steps, (a2-a)/steps

    for i in range(steps):
        x, y, a = x+dx, y+dy, a+da
        new_rect = gen_rect_pts(x, y, a)
        if _check_collision(obstacles, new_rect): return True
    return False

def plot_steps(start, end, distance, plotter):
    steps      = int(distance/10)+1
    x,  y,  a  = start
    x2, y2, a2 = end
    dx, dy, da = (x2-x)/steps, (y2-y)/steps, (a2-a)/steps
    
    rect1      = plotter.draw_rectangle(gen_rect_pts(x, y, a), color='yellow')
    for i in range(steps):
        rect1.remove()
        x, y, a = x+dx, y+dy, a+da
        rect1   = plotter.draw_rectangle(gen_rect_pts(x, y, a))

    rect1.remove()

    # Draw final state
    plotter.draw_rectangle(gen_rect_pts(x2, y2, a2), edgecolor='black')
    plotter.draw_circle((x2,y2), 4, facecolor='w', edgecolor='k', zorder=5)
    plotter.draw_line(start, (x2,y2), color='k', zorder=4)

def run(obstacles, start, goal, max_size, plotter):
    step_size   = 50
    final_pos   = np.array(goal[:2])
    
    # Pre-compute for generating new random points in Q_free
    minp, maxp   = obstacles.vertices.min(0), obstacles.vertices.max(0)
    span, offset = (maxp-minp)*1.1, minp-(maxp-minp)*0.05

    def gen_valid_rand(valid_function):
        """ Generates a valid q_rand in Q_free given a validity function """
        tmp     = np.random.random(2) * span + offset
        while not valid_function(*tmp):
            tmp = np.random.random(2) * span + offset
        return tmp

    KD    = KDTree(start[:2], 0)
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
        q_near, dist = KD.nearestNode(q_rand, return_node=True)

        # Generate the next node in the direction of q_rand
        if dist < 0.5: continue
        if dist < step_size:
            if trials < 10: continue
            q_next = tuple(q_rand)
        else:
            q_next = gen_next(tuple(q_near.node), q_rand, step_size)
            if not obstacles.point_is_valid(*q_next): continue
        
        dist = math.hypot(q_next[0]-q_near[0], q_next[1]-q_near[1])
        # Check validity and update tree
        for i in range(10):
            alpha_new = np.random.random() * 2*math.pi #( + q_near.alpha) - math.pi
            collides = check_collision(obstacles, (*q_near.node, q_near.alpha), (*q_next, alpha_new), dist)
            if not collides: break
        else: continue
        
        KD.addNode(q_next, alpha_new)
        plot_steps((*q_near.node, q_near.alpha), (*q_next, alpha_new), dist, plotter)

        goal_distance = math.hypot(q_next[0]-goal[0], q_next[1]-goal[1]) 
        collides = check_collision(obstacles, (*q_next, alpha_new), goal, goal_distance)
        if not collides:
            plot_steps((*q_next, alpha_new), goal, goal_distance, plotter)
            plotter.draw_rectangle(gen_rect_pts(*goal), facecolor='red', edgecolor='k')
            break

        trials = 0

    print("n =", KD.length)

if __name__ == '__main__':
    from ImageGenerator import ImageGenerator
    from utilities      import get_obstacle_course, get_start_and_goal

    obstacles   = get_obstacle_course("world_obstacles.txt")
    start, goal = (75., 50., 0.), (482.,577.,math.pi/2)

    plotter    = ImageGenerator()
    plotter.draw_obstacle_course(obstacles)
    plotter.draw_start_and_goal(start,goal)

    run(obstacles, start, goal, 2000, plotter)

    input("Press enter to exit : ")