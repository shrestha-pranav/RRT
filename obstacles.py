from __future__ import division

class Range:
    """ Represents a Range (min, max) given a series of values """
    def __init__(self, vals):    self.min, self.max = min(vals), max(vals)
    def __contains__(self, val): return self.min <= val < self.max
    def __str__(self):           return "%.3f %.3f"%(self.min, self.max)

class Point:
    """ Represents a 2D point """
    def __init__(self, x, y): self.x, self.y = x, y
    def __str__(self):        return "Point(%.3f, %.3f)"%(self.x, self.y)

class Line:
    """ Represents a 2D Line responsible for most collision calculations """
    def __init__(self, start_point, end_point):
        self.start  = start_point
        self.end    = end_point
        self.y_bar  = start_point.x == end_point.x
        self.x_bar  = start_point.y == end_point.y
        if not self.y_bar:
            self.slope  = self._slope()
            self.intercept = self.end.y - self.slope*self.end.x
        self.xrange = Range([start_point.x, end_point.x])
        self.yrange = Range([start_point.y, end_point.y])
    
    def _slope(self):         return (self.end.y - self.start.y)/(self.end.x - self.start.x)
    def __str__(self):        return "Line(%s,%s)" %(str(self.start), str(self.end))
    def __getitem__(self, i): return [self.start, self.end][i]
    
    def intersects(self, other):
        """ Main intersection calculation logic between two lines """
        if self.x_bar:
            if other.x_bar: return False
            if other.y_bar: return self.end.y in other.yrange and other.end.x in self.xrange
            xp = (self.end.y-other.intercept)/other.slope
            return xp in self.xrange and xp in other.xrange and self.end.y in other.yrange
        if self.y_bar:
            if other.x_bar: return other.end.y in self.yrange and self.end.x in other.xrange
            if other.y_bar: return False
            yp = (other.slope * self.end.x + other.intercept)
            return yp in self.yrange and yp in other.yrange and self.end.x in other.xrange

        if other.x_bar:
            xp = (other.end.y-self.intercept)/self.slope
            return xp in other.xrange and xp in self.xrange and other.end.y in self.yrange
        if other.y_bar:
            yp = (self.slope * other.end.x + self.intercept)
            return yp in other.yrange and other.end.x in self.xrange and yp in self.yrange
        
        if self.slope == other.slope: return False
        xp = (other.intercept - self.intercept) / (self.slope - other.slope)
        yp = self.slope * xp + self.intercept

        return xp in self.xrange and xp in other.xrange and \
                yp in self.yrange and yp in other.yrange
    
    def point_on_line(self, point):
        """ Checks if point is on the line, with tolerance 0.001 """
        if self.x_bar: return point.y==self.end.y and point.x in self.xrange
        if self.y_bar: return point.x==self.end.x and point.y in self.yrange
        return abs(point.y - self.slope * point.x - self.intercept) < 0.001

class Obstacle:
    """ Represents a polygonal obstacle given a sequence of vertices """
    def __init__(self, vertices):
        self.x = Range([v[0] for v in vertices])
        self.y = Range([v[1] for v in vertices])
        
        self.lines = [Line(Point(*vertices[i]), Point(*vertices[i+1])) for i in range(len(vertices)-1)]
    
    def check_collisions(self, line, verbose=False):
        # Safety check disabled
        # if self.point_in_obstacle(line[0]) or self.point_in_obstacle(line[1]): return True
        
        if line.xrange.max < self.x.min or line.xrange.min > self.x.max or \
            line.yrange.max < self.y.min or line.yrange.min > self.y.max:
            return False
        
        for l in self.lines:
            if l.intersects(line): return True
        return False
    
    def point_in_obstacle(self, pt, verbose=False):
        # Sanity check, point is in obstacle range
        if not (self.x.min <= pt.x <= self.x.max and self.y.min <= pt.y <= self.y.max):
            return False
        
        projection = Line(pt, Point(self.x.max+5, pt.y))
        num_cross  = 0
        
        for l in self.lines:
            # Exception, if point is in line (very rare) return true
            if l.point_on_line(pt): return True
            num_cross += l.intersects(projection)
            
        return bool(num_cross % 2)

class Obstacles:
    """ Container object for obstacles """
    def __init__(self, obstacles):
        self.obss = [Obstacle([tuple(pt.astype(int)) for pt in vertices]) for vertices in obstacles]
    
    def point_is_valid(self, x, y):
        """ Returns whether or not q=(x,y) is in Q_free """
        point = Point(x, y)
        for obs in self.obss:
            if obs.point_in_obstacle(point): return False
        return True

    def check_collisions(self, line, verbose=False):
        line = Line(Point(*line[0]), Point(*line[1]))
        
        for obs in self.obss:
            if obs.check_collisions(line, verbose):
                return True
        return False