from __future__ import division
import matplotlib.pyplot as plt
from matplotlib.path import Path
from matplotlib.lines import Line2D
import matplotlib.patches as patches
from collections import namedtuple
import numpy as np
import random, math
from shapely.geometry import Polygon, LineString
from shapely.geometry import Point as SPoint
from obstacles import Range, Point, Line, Obstacle, Obstacles

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

def draw_obstacle_course(path, ax):
    pathpatch = patches.PathPatch(path, facecolor='None', edgecolor='xkcd:violet')
    
    ax.add_patch(pathpatch)
    ax.set_title('Rapidly-exploring Random Tree')

    ax.dataLim.update_from_data_xy(path.vertices)
    ax.autoscale_view()
    ax.invert_yaxis()

def draw_start_and_goal(start, goal, ax):
    ax.add_patch(patches.Circle(start, facecolor='xkcd:bright green', zorder=10))
    ax.add_patch(patches.Circle(goal, facecolor='xkcd:fuchsia', zorder=10))

def gen_next(q_near, q_rand, scale=1):
    vec = q_rand - q_near
    return tuple(q_near + vec * scale / np.linalg.norm(vec))

def shapely_intersect(line):
    line = LineString(line)
    for shape in shapes:
        if line.intersects(shape): return True
    return False

def shapely_contains(point):
    point = SPoint(*point)
    for shape in shapes:
        if shape.contains(point) or shape.boundary.contains(point): return True
    return False

def gen_rand(low, high, size):
    arrs = []
    for i in range(len(low)):
        span = high[i]-low[i]
        arrs.append((np.random.random(size)*span*1.1 + low[i]-span*0.05).reshape(-1,1))
    return np.concatenate(arrs, 1)

if __name__ == '__main__':
    obstacle_path, goal_path = "../hw3/world_obstacles.txt", "../hw3/goal.txt"
    # obstacle_path, goal_path = "world_obstacles.txt", "start_goal.txt"
    path = get_obstacle_course(obstacle_path)
    start, goal = get_start_and_goal(goal_path)

    obstacles = Obstacles(path.to_polygons())
    shapes = [Polygon(polyg) for polyg in path.to_polygons()]
    num_tests = 10000

    """ Unit test for line in obstacle """
    minp, maxp = path.vertices.min(0), path.vertices.max(0)
    step_size = 50

    starts = gen_rand(minp, maxp, num_tests)
    rands  = gen_rand(minp, maxp, num_tests)

    # starts, rands = starts.astype(int), rands.astype(int)

    dirs  = starts - rands
    ends  = starts + dirs / np.hypot(dirs[:,0], dirs[:,1])[:,None] * step_size
    paths = [Path([starts[i], ends[i]], [Path.MOVETO, Path.LINETO]) for i in range(num_tests)]

    # Shapely answer
    my_ans = [obstacles.check_collisions(paths[i].vertices) for i in range(num_tests)]
    ex_ans = [shapely_intersect(paths[i].vertices) for i in range(num_tests)]

    correct_positives, correct_negatives = [], []
    false_positives, false_negatives = [], []
    for i in range(num_tests):
        if   my_ans[i] and ex_ans[i]:         correct_positives.append(i)
        elif not my_ans[i] and not ex_ans[i]: correct_negatives.append(i)
        elif ex_ans[i] and not my_ans[i]:     false_negatives.append(i)
        elif my_ans[i] and not ex_ans[i]:     false_positives.append(i)
            
    print("Correct : ", len(correct_positives), len(correct_negatives))
    print("False   : ", len(false_positives), len(false_negatives))

    for idx in false_negatives:
        print(idx, Line(Point(*paths[idx].vertices[0]), Point(*paths[idx].vertices[1])), my_ans[idx], ex_ans[idx])

    %matplotlib notebook
    fig, ax = plt.subplots()
    draw_obstacle_course(path, ax)
    draw_start_and_goal(start, goal, ax)
    plt.show()

    axtext = lambda x, y: "(%d, %d)"%(x,y)
        
    for (x,y) in path.vertices:
        if x == 0 and y == 0: continue
        ax.text(x, y, axtext(x, y), fontsize=8, color='xkcd:purple')
        
    # for idx in correct_positives:
    #     ax.add_patch(patches.PathPatch(paths[idx], color='xkcd:red', lw=1))

    # for idx in correct_negatives:
    #     ax.add_patch(patches.PathPatch(paths[idx], color='xkcd:green', lw=1))

    for idx in false_negatives:
        ax.add_patch(patches.PathPatch(paths[idx], color='xkcd:red', lw=2))

    for idx in false_positives:
        ax.add_patch(patches.PathPatch(paths[idx], color='xkcd:green', lw=3))

    plt.show()

    if len(false_negatives) > 0:
        test_point = false_negatives[0]
        test_path = paths[test_point].vertices
        obs_path  = Line(Point(*test_path[0]), Point(*test_path[1]))
        shp_path  = LineString(test_path)

        found = None
        for idx in range(len(obstacles.obss)):
            if obstacles.obss[idx].check_collisions(obs_path) != shapes[idx].intersects(shp_path):
                found = idx
                break

        if found is not None:
            idx = found
            print(idx, obs_path, obstacles.check_collisions(test_path))

            obstacle = obstacles.obss[idx]    
            for l in obstacle.lines:
                print(l, l.intersects(obs_path))
