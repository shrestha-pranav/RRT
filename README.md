# Rapidly-exploring Random Trees

Implementation of the [Rapidly-exploring random tree](https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree)

## Demo
- [Unidirectional RRT](https://www.youtube.com/watch?v=KVDZGo25JLw)
- [Bidirectional RRT](https://www.youtube.com/watch?v=E1kAa4vjrjQ)
- [Rotational RRT](https://www.youtube.com/watch?v=fUlszOxx0Qw)

## Dependencies

The project should run on a vanilla Python 3 installation with numpy and matplotlib installed

## Usage

There are two ways of running the program. The recommended way is using the `main.py` file. Command-line arguments are parsed by the program and detailed usage can be found using `python main.py -h`:

```
usage: main.py [-h] [-m {unidirectional,bidirectional,extra}]
               [--step-size step_size] [--max-search max_search]
               [--obstacle-path obstacle_path] [--goal-path goal_path]

Python implementation of a RRT Path Planner

optional arguments:
  -h, --help            show this help message and exit
  -m {unidirectional,bidirectional,extra}, --mode {unidirectional,bidirectional,extra}
                        RRT mode (default: unidirectional)
  --step-size step_size
                        Step size for uni/bi-directional RRT (default: 50)
  --max-search max_search
                        Max number of nodes to expand RRT (default: 2000)
  --obstacle-path obstacle_path
                        Obstacle filepath (default: world_obstacles.txt)
  --goal-path goal_path
                        Start/Goal filepath (default: start_goal.txt)

- Pranav Shrestha (ps2958), Greyson Barrera (gmb2167)
```

Example usages:
```bash
python main.py -m unidirectional --step-size 50 --max-search 2000
python main.py -m bidirectional --step-size 10 --max-search 4000
python main.py -m extra
```

If default values are desired, then the program can simply be run using the individual files
```bash
python unidirectionalrrt.py
python bidirectionalrrt.py
python extra_credit.py
```

## Implementation

The program was implemented using the default RRT algorithm, as shown in class. The files used and a few choice classes are explained below

| File | Function |
| --- | --- |
| `main.py` | Responsible for parsing command-line arguments and setting up obstacles, start, and goal states|
| `unidirectionalrrt.py`| Runs the Unidirectional RRT algorithm |
| `bidirectionalrrt.py`| Runs the Bidirectional RRT algorithm |
| `extra_credit.py`| Runs the RRT algorithm with a 2D rectangle |
| `KDTree.py`| Implements a KD-Tree for the nearest neighbor algorihtm |
| `ImageGenerator.py`| Responsible for all plotting functions, including setting up an interactive canvas and drawing the obstacles, circles, lines and rectangles |
| `obstacles.py`| (Most important) Contains classes responsible for collision-check|
| `utilities.py`| Helper functions including `gen_next` which generates `q_new` and helper class `PathTree` |

| Class | Function |
| --- | --- |
| `Obstacle` | Represents a single obstacle with methods for collision-check |
| `Line` | Represents a line and is responsible for most of the collision-check math logic|
| `ImageGenerator` | Responsible for all plotting functions |
| `KDTree` | KD-Tree for nearest neighbor algorithm |
| `PathTree` | Tree class for storing discovered the RRT |

