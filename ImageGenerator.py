import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.lines import Line2D
from matplotlib.path import Path

class ImageGenerator:
    """ Class that handles all matplotlib plotting tasks """
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        plt.ion()
        plt.show()

    def draw_obstacle_course(self, obstacle_path):
        pathpatch = patches.PathPatch(obstacle_path, facecolor='None', edgecolor='xkcd:violet')
        
        self.ax.add_patch(pathpatch)
        self.ax.set_title('Rapidly-exploring Random Tree')

        self.ax.dataLim.update_from_data_xy(obstacle_path.vertices)
        self.ax.autoscale_view()
        self.ax.invert_yaxis()

        self.update(0.1)

    def draw_start_and_goal(self, start, goal):
        self.ax.add_patch(patches.Circle(start, facecolor='xkcd:bright green', zorder=10))
        self.ax.add_patch(patches.Circle(goal,  facecolor='xkcd:fuchsia', zorder=10))

        self.update(0.1)

    def draw_circle(self, center, radius, time=0.01, update=True, **kwargs):
        circle_patch = patches.Circle(center[:2], radius, **kwargs)
        image = self.ax.add_patch(circle_patch)
        
        if update: self.update(time)
        return image

    def draw_line(self, start, end, time=0.01, update=True, **kwargs):
        line_patch = Line2D((start[0], end[0]), (start[1], end[1]), **kwargs)
        image      = self.ax.add_line(line_patch)

        if update: self.update(time)
        return image

    def draw_rectangle(self, vertices, time=0.01, update=True, **kwargs):
        rect_patch = patches.Polygon(vertices, fill=True, **kwargs)
        image      = self.ax.add_patch(rect_patch)

        if update: self.update(time)
        return image

    def update(self, time=0.01):
        plt.draw()
        plt.pause(time)

    def __del__(self):
        plt.ioff()
        plt.show()
