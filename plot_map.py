from PyQt5.QtWidgets import (QApplication, QWidget,
                             QVBoxLayout, QHBoxLayout,
                             QPushButton, QLabel, QLineEdit,
                             QGroupBox, QDoubleSpinBox,
                             QGraphicsView, QGraphicsScene,
                             QGraphicsItem, QGraphicsItemGroup,
                             QGraphicsEllipseItem, QGraphicsRectItem,
                             QGraphicsLineItem)
from PyQt5.QtGui import QBrush, QPen, QColor, QPainter, QLinearGradient
from PyQt5.QtCore import QTimer, Qt
import numpy as np
from math import *

#all length are in meters
robot_diameter = 0.074
startup_scale = 2000
startup_pen_width = 0.001
startup_resolution = 0.1
startup_tof_length = 1
background_color = QColor(216,222,233) #chose because I like it
high_color = QColor("#d558c8")
low_color = QColor("#24d292")
maximum_color_gradient = 1
obstacle_light = QColor("#909d9e")
obstacle_dark = QColor("#181b1b")
min_obstacle_probability = 0.5
prior_obstacle_prob = 0.1 #Prior probability that a random place is an obstacle (should be low)
obstacle_bayes_factor = 1.6 #Empirical

def bayes_inference(prior, bayes_factor):
    posterior = (bayes_factor*prior)/(1-prior*bayes_factor*prior)
    return posterior

class Map:
    def __init__(self):
        # The grid format is as follows:
        # {(x, y): (p, (h1, h2), (s1, s2), n)}
        # With x and y the coordinate of the grid element,
        # p the probability that it is an obstacle,
        # hx the respective height of the heighest and lowest corners
        # gx the coordinates of the slope vector (see update_slope_and_height function for a more thourough explanation)
        # n the number of samples taken from the robot for the slope (useful only for updates)
        self.min_height = 0
        self.max_height = 0
        self.scene = QGraphicsScene()
        self.resolution = startup_resolution
        self.orientation = 0 #We begin facing upwards

        theta = pi/4
        vec_dir = np.array([cos(theta),sin(theta)])
        vec_diag = np.array([self.resolution/2,self.resolution/2])*np.sign(vec_dir)
        vec_w = np.dot(vec_dir, vec_diag)*vec_dir

        self.grid = {(-1,0): (0, (0.005,-0.005), (0,0), 0),
                      (0,0): (0, (0.01,0), (vec_w[0], vec_w[1]), 0),
                      (0,1): (0, (0.015,0.005), (vec_w[0], vec_w[1]), 0),
                      (0,-1): (0, (0.005,-0.005), (vec_w[0], vec_w[1]), 0),
                      (1,-1): (0, (0.01,0), (vec_w[0], vec_w[1]), 0),
                      (1,1): (0, (0.02,0.01), (vec_w[0], vec_w[1]), 0),
                      (-1,-1): (0, (0,-0.01), (vec_w[0], vec_w[1]), 0),
                      (-1,1): (0, (0.01,0), (vec_w[0], vec_w[1]), 0),
                      (1,0): (0, (0.015,0.005), (vec_w[0], vec_w[1]), 0)}

        self.robot = QGraphicsItemGroup()
        self.draw_robot()
        self.robot_hidden = True

        pen = QPen(Qt.white)
        pen.setWidthF(startup_pen_width)

        self.grid_width = startup_pen_width
        self.items = {}

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
        self.hide_robot()
        self.items = {}

    def update_scene(self):
        #We first find the max and min
        self.max_height = 0
        self.min_height = 0
        for key in self.grid.keys():
            values = self.grid[key]
            [max_height, min_height] = values[1]
            if max_height > self.max_height:
                self.max_height = max_height
            if min_height < self.min_height:
                self.min_height = min_height

        for key in self.grid.keys():
            values = self.grid[key]

            if key not in self.items:
                rect = self.add_rectangle(key)
            else:
                rect = self.items[key]

            x = self.resolution * key[0]
            y = self.resolution * (-key[1])
            self.color_rectangle(rect, x, y, values)
        self.show_robot()

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
        if p >= min_obstacle_probability:
            color = interpolate_color((p - min_obstacle_probability)/(1-min_obstacle_probability), obstacle_light, obstacle_dark)
            brush = QBrush(color)
        elif slope[0] != 0 or slope [1] != 0:
            x_gradient = values[2][0]
            y_gradient = values[2][1]

            gradient = QLinearGradient(x-x_gradient, y+y_gradient, x+x_gradient, y-y_gradient)

            #We scale the grading according to the global extrema
            max_delta_height = self.max_height - self.min_height
            low_coeff = (min_height-self.min_height)/max_delta_height
            high_coeff = (max_height-self.min_height)/max_delta_height

            low_color_interpolate = interpolate_color(low_coeff, low_color, high_color) 
            high_color_interpolate = interpolate_color(high_coeff, low_color, high_color) 
            
            gradient.setColorAt(0, low_color_interpolate)
            gradient.setColorAt(1, high_color_interpolate)
            brush = QBrush(gradient)
        else:
            brush = QBrush(background_color)

        pen = QPen(Qt.white)
        pen.setWidthF(self.grid_width)
        rect.setPen(pen)
        rect.setBrush(brush)

    def draw_robot(self):
        robot_pen = QPen(Qt.black)
        robot_pen.setWidthF(startup_pen_width)

        ellipse = QGraphicsEllipseItem(-robot_diameter/2, -robot_diameter/2, robot_diameter, robot_diameter)
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
        tof_ray = QGraphicsLineItem(0, -robot_diameter/2, 0, -robot_diameter/2-startup_tof_length)
        tof_dot = QGraphicsEllipseItem(-robot_diameter/20, -11*robot_diameter/20-startup_tof_length, robot_diameter/10, robot_diameter/10)
        tof_ray.setPen(tof_pen)
        tof_dot.setBrush(tof_brush)
        tof_dot.setPen(tof_pen)

        self.robot.addToGroup(ellipse)
        self.robot.addToGroup(arrow_1)
        self.robot.addToGroup(arrow_2)
        self.robot.addToGroup(arrow_3)
        self.robot.addToGroup(tof_ray)
        self.robot.addToGroup(tof_dot)
        self.robot.setZValue(1)

        self.robot.setRotation(-degrees(self.orientation-pi/2))

    def update_slope_and_height(self, position, orientation_matrix):
        grid_coordinate = self.world_to_grid(position[0:2])
        height = position[2]
        z_robot = orientation_matrix[:,2] #Vector normal to ground
        dir_vec = -z_robot[0:2] #Vector point towards steepest ascent
        diag_vec = np.sign(dir_vec)*np.array([self.resolution/2, self.resolution/2]) #Diagonal vector closest to dir_vec

        # This strange vector points in the same direction as dir_vec, but it's tip
        # lie on the same isoline as the highest corner of the grid element
        # (which is useful for drawing the gradient coloring)
        # It's precisely this that we'll store in the grid dictionary
        # Note that it gives no information about the slope itself but only the direction
        s_vec = np.dot(diag_vec, dir_vec)/np.dot(dir_vec, dir_vec) * dir_vec
        slope = np.linalg.norm(dir_vec)/z_robot[2] # definition of the slope
        delta_h = slope*np.linalg.norm(s_vec) #
        h_max = height + delta_h
        h_min = height - delta_h

        if grid_coordinate in self.grid:
            [p, h, s, n] = self.grid[grid_coordinate]
            h_max = (n*h[0] + h_max)/(n+1)
            h_min = (n*h[1] + h_min)/(n+1)
            s_new_x = (n*s[0] + s_vec[0])/(n+1)
            s_new_y = (n*s[1] + s_vec[1])/(n+1)
            self.grid[grid_coordinate] = (p, (h_max, h_min), (s_new_x, s_new_y), n+1)
            rect = self.items[grid_coordinate]
        else:
            self.grid[grid_coordinate] = (prior_obstacle_prob, (h_max, h_min), (s_vec[0], s_vec[1]), 1)
            rect = self.add_rectangle(grid_coordinate)

        if h_max > self.max_height:
            self.max_heigh = h_max
        if h_min > self.min_height:
            self.min_heigh = h_min

        x = self.resolution * grid_coordinate[0]
        y = self.resolution * (-grid_coordinate[1])
        self.color_rectangle(rect, x, y, self.grid[grid_coordinate])


    def world_to_grid(self, coordinate):
        x = round(coordinate[0]/self.resolution)
        y = round(coordinate[1]/self.resolution)
        return (x,y)

    #is clear let you inform that you saw no obstacle and that you should decrease the probability
    def update_obstacle_probability(self, position, bayes_factor):
        grid_coordinate = self.world_to_grid(position)

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
        
        x = self.resolution * grid_coordinate[0]
        y = self.resolution * (-grid_coordinate[1])
        self.color_rectangle(rect, x, y, values)
            
    def move_robot(self, position, orientation_matrix):
        self.robot.setPos(position[0], -position[1])

    def update_resolution(self, resolution):
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
                        new_max_vec = np.sign(s_vec)*np.array([resolution/2, resolution/2])
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

                    [p, h, s, n] = self.grid[key]
                    new_grid[(new_x, new_y)] = (p, (h_max, h_min), s, n) #We assign it its height values, the other stay unchanged

        self.grid = new_grid
        self.reset_scene()
        self.resolution = resolution
        self.grid_width = sqrt(resolution/startup_resolution)*startup_pen_width
        self.update_scene()


class My_view(QGraphicsView):
    def __init__(self, parent=None):
        super(My_view, self).__init__(parent)

        self.setTransformationAnchor(QGraphicsView.NoAnchor)
        self.setResizeAnchor(QGraphicsView.NoAnchor)

    def wheelEvent(self, event):
        zoomFactor = 1.25

        oldPos = self.mapToScene(event.pos())

        if event.angleDelta().y() < 0:
            zoomFactor = 1/zoomFactor
        self.scale(zoomFactor, zoomFactor)

        newPos = self.mapToScene(event.pos())

        delta = newPos - oldPos
        self.translate(delta.x(), delta.y())

class Main_window:
    def __init__(self):
        self.port = "/dev/ttyACM1"
        self.listening = False

        self.app = QApplication([])
        self.window = QWidget()

        self.map = Map()

        self.main_layout = QHBoxLayout()

        self.controls_layout = QVBoxLayout()
        self.view = My_view(self.map.scene)

        self.port_box = QGroupBox()
        self.port_box_layout = QVBoxLayout()

        self.port_line_layout = QHBoxLayout()
        self.port_line = QLineEdit(self.port)
        self.port_line_label = QLabel('Port:')

        self.button_listen = QPushButton('Listen')

        self.resolution_box = QGroupBox()
        self.resolution_box_layout = QHBoxLayout()
        self.resolution_label = QLabel("Resolution: ")
        self.resolution_spin_box = QDoubleSpinBox()
        
        self.reset_button = QPushButton("Reset")

        self.window.setLayout(self.main_layout)

        self.init_controls(self.main_layout)
        self.init_graphics(self.main_layout)

        self.view.show()
        self.window.show()
        self.app.exec()

    def init_graphics(self, parent_layout):
        parent_layout.addWidget(self.view)
        self.view.setMinimumSize(500,500)
        self.view.setRenderHint(QPainter.Antialiasing)
        self.view.scale(startup_scale, startup_scale)

        brush = QBrush(background_color)
        self.view.setBackgroundBrush(brush)

    def init_controls(self, parent_layout):
        parent_layout.addLayout(self.controls_layout)

        self.init_port_box(self.controls_layout)
        self.init_resolution_box(self.controls_layout)

        self.reset_button.clicked.connect(self.reset_button_event)
        self.controls_layout.addWidget(self.reset_button)

        self.controls_layout.addStretch()

    def init_port_box(self, parent_layout):
        self.port_box.setMaximumWidth(300)
        parent_layout.addWidget(self.port_box)
        self.port_box.setLayout(self.port_box_layout)

        self.init_port_line(self.port_box_layout)

        self.button_listen.setCheckable(True)
        self.button_listen.clicked.connect(self.toggle_listen)
        self.port_box_layout.addWidget(self.button_listen)

    def init_port_line(self, parent_layout):
        self.port_line.editingFinished.connect(self.update_port)
        self.port_line.setMinimumWidth(100)
        self.port_line_layout.addWidget(self.port_line_label)
        self.port_line_layout.addWidget(self.port_line)
        parent_layout.addLayout(self.port_line_layout)

    def init_resolution_box(self, parent_layout):
        parent_layout.addWidget(self.resolution_box)
        self.resolution_box.setLayout(self.resolution_box_layout)
        self.resolution_box_layout.addWidget(self.resolution_label)
        self.resolution_box_layout.addWidget(self.resolution_spin_box)

        self.resolution_spin_box.setSuffix(' cm')
        self.resolution_spin_box.setDecimals(1)
        self.resolution_spin_box.setMinimum(1)
        self.resolution_spin_box.setMaximum(20)
        self.resolution_spin_box.setStepType(QDoubleSpinBox.AdaptiveDecimalStepType)

        self.resolution_spin_box.setValue(100*self.map.resolution)
        self.resolution_spin_box.valueChanged.connect(self.update_resolution)

    def update_resolution(self, resolution):
        def delayed_update_resolution():
            self.map.update_resolution(0.01*resolution)
            self.resolution_timer.stop()
        self.resolution_timer = QTimer()
        self.resolution_timer.timeout.connect(delayed_update_resolution)
        self.resolution_timer.start(1000)

    def reset_button_event(self):
        self.map.reset_scene()

    def update_port(self):
        self.port = self.line_port.text()

    def toggle_listen(self):
        self.listening = self.button_listen.isChecked()
        if self.listening:
            self.map.update_scene()
        else:
            self.map.hide_robot()
        print(self.listening)

Main_window()

