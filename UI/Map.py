from PyQt5.QtGui import QBrush, QPen, QColor, QLinearGradient
from PyQt5.QtCore import Qt, pyqtSignal, QObject
from PyQt5.QtWidgets import (QGraphicsScene, QGraphicsItemGroup,
                             QGraphicsEllipseItem, QGraphicsRectItem,
                             QGraphicsLineItem)
import numpy as np
import matplotlib.pyplot as plt
from math import pi, degrees, copysign, sqrt, atan, sin, cos

#all length are in meters
#robot dimensions
max_tof_length = 0.5
robot_diameter = 0.074
ir_angles = (-pi/12, -pi/4, -pi/2, -5*pi/6, 5*pi/6, pi/2, pi/4, pi/12)

#colors and pens
background_color = QColor(216,222,233) #chose because I like it
high_color = QColor("#d558c8")
low_color = QColor("#24d292")
obstacle_light = QColor("#909d9e")
obstacle_dark = QColor("#181b1b")
startup_pen_width = 0.001

startup_resolution = 0.05
max_sample_nb = 10 #Max value of n parameter (see update_grid_robot_on)
min_obstacle_probability = 0.5
prior_obstacle_prob = 0.1 #Prior probability that a random place is an obstacle

#bayes factors
obstacle_bayes_factor = 1.6 #Empirical
no_obstacle_bayes_factor = 1/obstacle_bayes_factor #Empirical
bayes_factor_robot_on = 0.55 #Empirical
ir_bayes_factor = 1.8

def bayes_inference(prior, bayes_factor):
    posterior = (bayes_factor*prior)/(1-prior+bayes_factor*prior)
    return posterior

class Map(QObject):
    height_extrema_changed = pyqtSignal([float, float])

    def __init__(self, parent=None):
        super(Map, self).__init__(parent)
        # The grid format is as follows:
        # {(x, y): (p, (h1, h2), (s1, s2), n)}
        # With x and y the coordinate of the grid element,
        # p the probability that it is an obstacle,
        # hx the respective height of the heighest and lowest corners
        # gx the coordinates of the slope vector (see update_grid_robot_on
        # function for a more thourough explanation)
        # n the number of samples taken from the robot for the slope
        # (useful only for updates)
        self.min_height = 0
        self.max_height = 0
        self.should_update = False
        self.scene = QGraphicsScene()
        self.resolution = startup_resolution
        self.orientation = 0 #random value, which is unimportant

        self.grid: dict[tuple[int, int],
                        tuple[float, tuple[float, float],
                              tuple[float, float], int]] = {}

        self.robot = QGraphicsItemGroup()
        self.tof_ray = QGraphicsLineItem()
        self.tof_dot = QGraphicsEllipseItem()
        self.tof_length = 0
        self.init_robot()
        self.robot_hidden = True

        pen = QPen(Qt.white)
        pen.setWidthF(startup_pen_width)

        self.grid_width = startup_pen_width
        self.items = {}

    def plot3d(self):
        vertices = {}
        for grid in self.grid:
            values = self.grid[grid]
            s_vec = np.array(self.grid[grid][2])
            if any(s_vec != np.zeros(2)):
                [h_max, h_min] = values[1]
                center = self.resolution*np.array(grid)

                p_max = np.sign(s_vec)*self.resolution/2
                p_min = -p_max
                p1 = np.array([-1, 1]) * p_max
                p2 = -p1

                h_center = (h_max + h_min)/2
                delta_h = np.dot(p1, s_vec)/np.dot(s_vec, s_vec)*(h_max - h_center)
                h1 = h_center + delta_h
                h2 = h_center - delta_h

                vertices[tuple(p_max+center)] = h_max
                vertices[tuple(p_min+center)] = h_min
                vertices[tuple(p1+center)] = h1
                vertices[tuple(p2+center)] = h2

        xy = np.array(list(vertices.keys()))
        if len(xy) != 0:
            x = xy[:,0]
            y = xy[:,1]
            z = np.array(list(vertices.values()))

            fig = plt.figure()
            ax = fig.add_subplot(projection='3d')
            ax.plot_trisurf(x, y, z)
            plt.show()

    def show_robot(self):
        if self.robot_hidden:
            self.scene.addItem(self.robot)
        self.robot_hidden = False


    def hide_robot(self):
        if not self.robot_hidden:
            self.scene.removeItem(self.robot)
        self.robot_hidden = True

    def reset_scene(self):
        for key in self.items:
            self.scene.removeItem(self.items[key])
        self.items = {}
        self.grid = {}

    def compute_height_extrema(self):
        #We first find the max and min
        old_max = self.max_height
        old_min = self.min_height

        first = True #Because there is no sensible
                     #value to initialize the extrema with
        for key in self.grid.keys():
            values = self.grid[key]
            [max_height, min_height] = values[1]
            if first:
                self.min_height = min_height
                self.max_height = max_height
                first = False
            if max_height > self.max_height:
                self.max_height = max_height
            if min_height < self.min_height:
                self.min_height = min_height
        if self.min_height != old_min or self.max_height != old_max:
            self.height_extrema_changed.emit(self.min_height, self.max_height)
            self.should_update = True

    def update_scene(self):
        for key in self.grid.keys():
            values = self.grid[key]

            if key not in self.items:
                rect = self.add_rectangle(key)
            else:
                rect = self.items[key]

            x = self.resolution * key[0]
            y = self.resolution * (-key[1])
            self.color_rectangle(rect, x, y, values)

        self.should_update = False

    def add_rectangle(self, key):
        x = self.resolution * (key[0] - 0.5)
        y = self.resolution * (-key[1] - 0.5)
        rect = QGraphicsRectItem(x, y, self.resolution, self.resolution)
        self.scene.addItem(rect)
        self.items[key] = rect
        return rect

    def color_rectangle(self, rect, x, y, values):
        [max_height, min_height] = values[1]

        def interpolate_color(coeff, low_color, high_color):
            def interpolate(coeff, low, high):
                return (1-coeff)*low + coeff*high

            low_r = int(interpolate(coeff, low_color.red(), high_color.red()))
            low_g = int(interpolate(coeff, low_color.green(), high_color.green()))
            low_b = int(interpolate(coeff, low_color.blue(), high_color.blue()))
            return QColor(low_r, low_g, low_b)

        p = values[0]
        slope = values[2]
        n = values[3]
        if p >= min_obstacle_probability:
            color = interpolate_color((p - min_obstacle_probability)\
                                    / (1-min_obstacle_probability),\
                                      obstacle_light, obstacle_dark)
            brush = QBrush(color)
        elif slope[0] != 0 or slope [1] != 0:
            x_gradient = slope[0]
            y_gradient = slope[1]

            gradient = QLinearGradient(x-x_gradient, y+y_gradient,\
                                       x+x_gradient, y-y_gradient)

            #We scale the grading according to the global extrema
            max_delta_height = self.max_height - self.min_height
            if max_delta_height == 0:
                low_coeff = 0.5
                high_coeff = 0.5
            else:
                low_coeff = (min_height-self.min_height)/max_delta_height
                high_coeff = (max_height-self.min_height)/max_delta_height

            low_color_interpolate = interpolate_color(low_coeff, low_color, high_color) 
            high_color_interpolate = interpolate_color(high_coeff, low_color, high_color) 
            
            gradient.setColorAt(0, low_color_interpolate)
            gradient.setColorAt(1, high_color_interpolate)
            brush = QBrush(gradient)
        elif n > 0: #if there is a measure
            brush = QBrush(interpolate_color(0.5, low_color, high_color))
        else:
            brush = QBrush(background_color)

        pen = QPen(Qt.white)
        pen.setWidthF(self.grid_width)
        rect.setPen(pen)
        rect.setBrush(brush)

    def init_robot(self):
        robot_pen = QPen(Qt.black)
        robot_pen.setWidthF(startup_pen_width)

        ellipse = QGraphicsEllipseItem(-robot_diameter/2, -robot_diameter/2,\
                                       robot_diameter, robot_diameter)
        arrow_1 = QGraphicsLineItem(0, robot_diameter/3, 0, -robot_diameter/3)
        arrow_2 = QGraphicsLineItem(0, -robot_diameter/3, robot_diameter/4, 0)
        arrow_3 = QGraphicsLineItem(0, -robot_diameter/3, -robot_diameter/4, 0)

        ellipse.setPen(robot_pen)
        arrow_1.setPen(robot_pen)
        arrow_2.setPen(robot_pen)
        arrow_3.setPen(robot_pen)

        tof_pen = QPen(Qt.red)
        tof_pen.setWidthF(startup_pen_width)
        tof_brush = QBrush(Qt.red)
        self.tof_ray.setPen(tof_pen)
        self.tof_dot.setBrush(tof_brush)
        self.tof_dot.setPen(tof_pen)

        self.robot.addToGroup(ellipse)
        self.robot.addToGroup(arrow_1)
        self.robot.addToGroup(arrow_2)
        self.robot.addToGroup(arrow_3)
        self.robot.addToGroup(self.tof_ray)
        self.robot.addToGroup(self.tof_dot)
        self.robot.setZValue(1)

    def update_grid_robot_on(self, position, orientation_matrix):
        cells = self.find_grid_robot_on()
        robot_center = position[0:2]
        robot_height = position[2]

        #Vector normal to ground
        z_robot = np.array(orientation_matrix)[:,2]
        dir_vec = -z_robot[0:2] #Vector point towards steepest ascent
        diag_vec = np.array([self.resolution/2, self.resolution/2]) 
        #Diagonal vector closest to dir_vec
        diag_vec = np.sign(dir_vec) * diag_vec

        # This strange vector points in the same direction as dir_vec, but its tip
        # lies on the same isoline as the highest corner of the grid element
        # (which is useful for drawing the gradient coloring)
        # It's precisely this that we'll store in the grid dictionary
        # Note that it gives no information about the slope itself but only the direction
        epsilon = 1e-6
        if np.linalg.norm(dir_vec) < epsilon:
            s_vec = np.array([0,0])
        else:
            s_vec = np.dot(diag_vec, dir_vec) / np.dot(dir_vec, dir_vec) * dir_vec
        slope = np.linalg.norm(dir_vec)/z_robot[2] # definition of the slope
        delta_h = slope*np.linalg.norm(s_vec) #delta between center and corners

        for grid_coordinate in cells:
            cell_center = self.resolution*np.array(grid_coordinate)
            rel_pos_vec = cell_center - np.array(robot_center)
            delta_s = np.dot(cell_center, dir_vec) / np.linalg.norm(dir_vec)
            height = robot_height + slope*delta_s #cell center height
            h_max = height + delta_h
            h_min = height - delta_h

            if grid_coordinate in self.grid:
                [p, h, s, n] = self.grid[grid_coordinate]
                if n >= max_sample_nb:
                    n = max_sample_nb - 1
                h_max = (n*h[0] + h_max)/(n+1)
                h_min = (n*h[1] + h_min)/(n+1)
                s_new_x = (n*s[0] + s_vec[0])/(n+1)
                s_new_y = (n*s[1] + s_vec[1])/(n+1)
                self.grid[grid_coordinate] = (p, (h_max, h_min),\
                                              (s_vec[0], s_vec[1]), n+1)
                rect = self.items[grid_coordinate]
            else:
                self.grid[grid_coordinate] = (prior_obstacle_prob, (h_max, h_min), \
                                              (s_vec[0], s_vec[1]), 1)
                rect = self.add_rectangle(grid_coordinate)
            rel_dist = np.linalg.norm(rel_pos_vec)
            #bayes_factor is smaller towards the edges
            bayes_factor = bayes_factor_robot_on\
                         + 4 * (1-bayes_factor_robot_on)\
                             * (rel_dist / (self.resolution + robot_diameter))**2
            if bayes_factor > 1:
                bayes_factor = 1
            # bayes_factor = bayes_factor_robot_on
            self.update_obstacle_probability(grid_coordinate, bayes_factor)

        self.compute_height_extrema()
        if not self.should_update:
            for grid_coordinate in cells:
                rect = self.items[grid_coordinate]
                x = grid_coordinate[0]*self.resolution
                y = grid_coordinate[1]*self.resolution
                self.color_rectangle(rect, x, y, self.grid[grid_coordinate])

    def world_to_grid(self, coordinate):
        x = round(coordinate[0]/self.resolution)
        y = round(coordinate[1]/self.resolution)
        return (x, y)

    def find_intersection(self):
        if self.tof_length > max_tof_length:
            length = max_tof_length
        else:
            length = self.tof_length

        angle = self.orientation
        dir_vec = np.array([cos(angle), sin(angle)])
        start_point = np.array([self.robot.pos().x(), -self.robot.pos().y()])
        start_point += robot_diameter/2*dir_vec
        line_vec = length * dir_vec
        end_point = start_point + line_vec
        top_right = np.maximum(start_point, end_point)
        bottom_left = np.minimum(start_point, end_point)
        
        first_intersections = bottom_left \
                            - (bottom_left - self.resolution/2)%self.resolution \
                            + self.resolution*np.ones(2)
        y_intersections = np.arange(first_intersections[0], top_right[0],\
                                    self.resolution)
        x_intersections = np.arange(first_intersections[1], top_right[1],\
                                    self.resolution)
        
        grid_cells = []
        for y in x_intersections:
            k = (y - start_point[1])/(line_vec[1])
            x = start_point[0] + k*line_vec[0]
            x = round(x/self.resolution)
            y1 = round(y/self.resolution - 0.5)
            y2 = round(y/self.resolution + 0.5)
            grid_cells.append((x, y1))
            grid_cells.append((x, y2))
        for x in y_intersections:
            k = (x - start_point[0])/(line_vec[0])
            y = start_point[1] + k*line_vec[1]
            y = round(y/self.resolution)
            x1 = round(x/self.resolution - 0.5)
            x2 = round(x/self.resolution + 0.5)
            grid_cells.append((x1, y))
            grid_cells.append((x2, y))
        grid_cells = list(dict.fromkeys(grid_cells)) #remove duplicate

        if self.tof_length > max_tof_length:
            obstacle_coordinate = None
        else:
            obstacle_coordinate = self.world_to_grid(end_point)
            if obstacle_coordinate in grid_cells: grid_cells.remove(obstacle_coordinate)

        return [grid_cells, obstacle_coordinate]

    def find_grid_robot_on(self):
        left = self.robot.pos().x() - robot_diameter/2
        right = left + robot_diameter
        bottom = -self.robot.pos().y() - robot_diameter/2
        top = bottom + robot_diameter

        first_intersection_y = left\
                             - (left - self.resolution/2)%self.resolution \
                             + self.resolution
        first_intersection_x = bottom\
                             - (bottom - self.resolution/2)%self.resolution \
                             + self.resolution
        y_intersections = np.arange(first_intersection_y, right,\
                                    self.resolution)
        x_intersections = np.arange(first_intersection_x, top,\
                                    self.resolution)

        grid_cells = []
        for x in y_intersections:
            for y in x_intersections:
                if (x - self.robot.pos().x())**2\
                 + (y + self.robot.pos().y())**2 < robot_diameter**2/4:
                     grid_cells.append(\
                             self.world_to_grid((x - self.resolution/2,\
                                                 y - self.resolution/2)))
                     grid_cells.append(\
                             self.world_to_grid((x + self.resolution/2,\
                                                 y - self.resolution/2)))
                     grid_cells.append(\
                             self.world_to_grid((x - self.resolution/2,\
                                                 y + self.resolution/2)))
                     grid_cells.append(\
                             self.world_to_grid((x + self.resolution/2,\
                                                 y + self.resolution/2)))
        
        grid_cells.append(self.world_to_grid((left, -self.robot.pos().y())))
        grid_cells.append(self.world_to_grid((right, -self.robot.pos().y())))
        grid_cells.append(self.world_to_grid((self.robot.pos().x(), top)))
        grid_cells.append(self.world_to_grid((self.robot.pos().x(), bottom)))
        grid_cells.append(self.world_to_grid((self.robot.pos().x(),\
                                              -self.robot.pos().y())))

        return list(dict.fromkeys(grid_cells))
        
    def update_obstacles(self, ir):
        [cells, obstacle_coordinate] = self.find_intersection()

        for cell in cells:
            self.update_obstacle_probability(cell, no_obstacle_bayes_factor)

        if obstacle_coordinate != None:
            self.update_obstacle_probability(obstacle_coordinate,\
                                             obstacle_bayes_factor)
        for i in range(0, 8):
            robot_pos = np.array([self.robot.pos().x(), -self.robot.pos().y()])
            ir_pos = robot_pos\
                   + np.array([ cos(ir_angles[i]+self.orientation),\
                                sin(ir_angles[i]+self.orientation) ])\
                   * robot_diameter/2
            if ir[i]:
                bayes_factor = ir_bayes_factor
            else:
                bayes_factor = 1/ir_bayes_factor

            self.update_obstacle_probability(self.world_to_grid(ir_pos),
                                             bayes_factor)

    def update_obstacle_probability(self, grid_coordinate, bayes_factor):
        if grid_coordinate in self.grid:
            [p, h, s, n] = self.grid[grid_coordinate]
            if p == 0:
                p=prior_obstacle_prob
            p = bayes_inference(p, bayes_factor)
            values = (p, h, s, n)
            self.grid[grid_coordinate] = values
            rect = self.items[grid_coordinate]
        else:
            p = bayes_inference(prior_obstacle_prob, bayes_factor)
            values = (p, (0, 0), (0, 0), 0)
            self.grid[grid_coordinate] = values
            rect = self.add_rectangle(grid_coordinate)
        
        if not self.should_update:
            x = grid_coordinate[0]*self.resolution
            y = grid_coordinate[1]*self.resolution
            self.color_rectangle(rect, x, y, self.grid[grid_coordinate])
                    
    def move_robot(self, position, orientation_matrix):
        self.robot.setPos(position[0], -position[1])
        forward_vec = [orientation_matrix[1][1], -orientation_matrix[0][1]]
        if forward_vec[0] == 0:
            angle = copysign(forward_vec[1], pi/2)
        else:
            angle = atan(forward_vec[1]/forward_vec[0])
            if forward_vec[0] < 0:
                angle += pi
        self.orientation = angle - pi/2 #because robot front is -y
        self.robot.setRotation(90-degrees(self.orientation))
        #because Qt 0 degrees is up and indirect oriented

    def update_tof(self, length):
        self.tof_length = length
        self.tof_ray.setLine(0, -robot_diameter/2, 0, -robot_diameter/2-length)
        self.tof_dot.setRect(-robot_diameter/20, -11*robot_diameter/20-length,
                              robot_diameter/10, robot_diameter/10)

    def update_resolution(self, resolution: float):
        new_grid = {}
        for key in self.grid:
            [x, y] = key
            interval_x = [(x-1/2)*self.resolution, (x+1/2)*self.resolution]
            interval_y = [(y-1/2)*self.resolution, (y+1/2)*self.resolution]
            
            low_x = int(interval_x[0]/resolution + 0.5+copysign(0.5, interval_x[0]))
            high_x = int(interval_x[1]/resolution + 0.5+copysign(0.5, interval_x[1]))
            low_y = int(interval_y[0]/resolution + 0.5+copysign(0.5, interval_y[0]))
            high_y = int(interval_y[1]/resolution + 0.5+copysign(0.5, interval_y[1]))

            old_h_max = self.grid[key][1][0]
            old_h_min = self.grid[key][1][1]
            s_vec = np.array(self.grid[key][2])
            for new_x in range(low_x, high_x):
                for new_y in range(low_y, high_y):
                    if (old_h_max != old_h_min) and (s_vec[0] != 0 or s_vec[1] != 0):
                        #We interpolate the new max and min height of each corners
                        new_max_vec = np.array([resolution/2, resolution/2])*np.sign(s_vec)
                        new_max_vec = new_max_vec + np.array([resolution*new_x, resolution*new_y])
                        old_max_vec = np.sign(s_vec)*np.array([self.resolution/2, self.resolution/2])
                        old_max_vec = old_max_vec + np.array([self.resolution*x, self.resolution*y])
                        delta_max_vec = old_max_vec - new_max_vec
                        delta_h_max = np.dot(delta_max_vec, s_vec)/(2*np.dot(s_vec, s_vec))*(old_h_max - old_h_min)
                        h_max = old_h_max - delta_h_max

                        new_min_vec = -np.sign(s_vec)*np.array([resolution/2, resolution/2])
                        new_min_vec = new_min_vec + np.array([resolution*new_x, resolution*new_y])
                        old_min_vec = -np.sign(s_vec)*np.array([self.resolution/2, self.resolution/2])
                        old_min_vec = old_min_vec + np.array([self.resolution*x, self.resolution*y])
                        delta_min_vec = old_min_vec - new_min_vec
                        delta_h_min = np.dot(delta_min_vec, s_vec)/(2*np.dot(s_vec, s_vec))*(old_h_max - old_h_min)
                        h_min = old_h_min - delta_h_min
                    else:
                        h_max = 0
                        h_min = 0

                    [p, [s, n]] = [self.grid[key][0], self.grid[key][2:4]]
                    new_grid[(new_x, new_y)] = (p, (h_max, h_min), s, n) #We assign it its height values, the other stay unchanged

        self.reset_scene()
        self.grid = new_grid
        self.resolution = resolution
        self.grid_width = sqrt(resolution/startup_resolution)*startup_pen_width
        self.update_scene()
