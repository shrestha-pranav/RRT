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

    """ Unit test for point in obstacle """
    minp, maxp = path.vertices.min(0), path.vertices.max(0)
    points = gen_rand(minp, maxp, num_tests)

    # points = points.astype(int)

    my_ans = [not obstacles.point_is_valid(*points[i]) for i in range(num_tests)]
    ex_ans = [shapely_contains(points[i]) for i in range(num_tests)]

    correct_positives, correct_negatives = [], []
    false_positives, false_negatives = [], []
    for i in range(num_tests):
        if   my_ans[i] and ex_ans[i]:         correct_positives.append(i)
        elif not my_ans[i] and not ex_ans[i]: correct_negatives.append(i)
        elif ex_ans[i] and not my_ans[i]:     false_negatives.append(i)
        elif my_ans[i] and not ex_ans[i]:     false_positives.append(i)
            
    print("Correct : ", len(correct_positives), len(correct_negatives))
    print("False   : ", len(false_positives), len(false_negatives))

    for i in false_positives: print(points[i], my_ans[i], ex_ans[i])
    for i in false_negatives: print(points[i], my_ans[i], ex_ans[i])

    %matplotlib notebook
    fig, ax = plt.subplots()
    draw_obstacle_course(path, ax)
    draw_start_and_goal(start, goal, ax)
    plt.show()

    axtext = lambda x, y: "(%d, %d)"%(x,y)
        
    for node in path.vertices:
        x, y = node
        if x == 0 and y == 0: continue
        ax.text(x, y, axtext(x, y), fontsize=5, color='xkcd:purple')

    for idx in correct_positives:
        ax.add_patch(patches.Circle(points[idx], 3, color='xkcd:green'))

    for idx in correct_negatives:
        ax.add_patch(patches.Circle(points[idx], 3, color='xkcd:red'))

    for idx in false_negatives:
        ax.add_patch(patches.Circle(points[idx], 3, color='xkcd:red'))
    #     ax.text(points[idx][0]+3, points[idx][1]-3 , axtext(points[idx][0], points[idx][1]), fontsize=5, zorder=5)

    for idx in false_positives:
        ax.add_patch(patches.Circle(points[idx], 3, color='xkcd:green'))
    #     ax.text(points[idx][0]+3, points[idx][1]-3 , axtext(points[idx][0], points[idx][1]), fontsize=5, zorder=5)

    if len(false_negatives)> 0:
        test_point = points[false_negatives[0]]

        pt, spt = Point(*test_point), SPoint(*test_point)
        found = None
        for idx in range(len(obstacles.obss)):
            if obstacles.obss[idx].point_in_obstacle(pt) != shapes[idx].contains(spt):
                found = idx
                break

        if found is not None:
            idx = found
            print(idx)
            obstacle = obstacles.obss[idx]
            projection = Line(pt, Point(obstacle.x.max+5, pt.y))
            print(projection)
            num_cross  = 0
            for l in obstacle.lines:
                print(l, l.intersects(projection), projection.intersects(l))
                # Exception, if point is in line (very rare) return true
                if l.point_on_line(pt): print(True); break
                num_cross += l.intersects(projection)
            
            print(num_cross, bool(num_cross % 2), shapes[idx].contains(spt))