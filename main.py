import sys
import argparse
import math
import numpy as np
from matplotlib.path import Path

from obstacles         import Obstacles
from ImageGenerator    import ImageGenerator
from utilities         import get_obstacle_course, get_start_and_goal

from unidirectionalrrt import run as RRT
from bidirectionalrrt  import run as BRRT
from extra_credit      import run as EXTRA

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
            description="Python implementation of a RRT Path Planner",
            epilog="- Pranav Shrestha (ps2958), Greyson Barrera (gmb2167)",
            formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("-m", "--mode", default="unidirectional", 
                            choices=["unidirectional", "bidirectional", "extra"],
                            help="RRT mode")

    parser.add_argument("--step-size",  metavar="step_size", default=50, type=int,
                            help="Step size for uni/bi-directional RRT")
    parser.add_argument("--max-search", metavar="max_search", default=2000, type=int,
                            help="Max number of nodes to expand RRT")
    parser.add_argument("--obstacle-path", metavar="obstacle_path", default="world_obstacles.txt",
                            help="Obstacle filepath")
    parser.add_argument("--goal-path", metavar="goal_path", default="start_goal.txt",
                            help="Start/Goal filepath")
    
    args = parser.parse_args()

    obstacles    = get_obstacle_course(args.obstacle_path)
    start, goal  = (75., 50., 0.), (482.,577.,math.pi/2)
    if args.mode != "extra": start, goal = get_start_and_goal(args.goal_path)

    plotter    = ImageGenerator()
    plotter.draw_obstacle_course(obstacles)
    plotter.draw_start_and_goal(start,goal)

    if args.mode == "unidirectional":
        RRT(obstacles, start, goal, args.step_size, args.max_search, plotter)
    elif args.mode =="bidirectional":
        BRRT(obstacles, start, goal, args.step_size, args.max_search, plotter)
    else:
        EXTRA(obstacles, start, goal, args.max_search, plotter)

    input("Press enter to exit : ")