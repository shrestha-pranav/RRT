from __future__ import division
import matplotlib.pyplot as plt
from matplotlib.path import Path
from matplotlib.lines import Line2D
import matplotlib.patches as patches
import numpy as np
import random, math
from obstacles import Range, Point, Line, Obstacle, Obstacles

from importlib import reload

import KDTree
from utilities import *
from shapely.geometry.polygon import LinearRing, Polygon
from descartes import PolygonPatch

init_t = math.atan(2.5)
r = math.sqrt(10**2+25**2)
thetas = [init_t, math.pi-init_t, math.pi+init_t, 2*math.pi-init_t]
def gen_rect_pts(x, y, alpha):
    return [(x+r*math.cos(theta+alpha), y+r*math.sin(theta+alpha))
               for theta in thetas]

fig, ax = plt.subplots()
plt.ion()
plt.show()

for pt in [(-50,-50), (-50,50), (50,50), (50,-50)]: ax.plot(*pt)
ax.autoscale_view()
plt.draw()
plt.pause(0.1)

i = 0.
while i <= 2*math.pi:
    print(math.degrees(i + init_t), ["(%.2f,%.2f)"%pt for pt in gen_rect_pts(10, 25, i)])
    ax.add_patch(PolygonPatch(Polygon(gen_rect_pts(10, 25, i))))
    i += math.pi/64
    plt.draw()
    plt.pause(0.01)

plt.ioff()
plt.show()