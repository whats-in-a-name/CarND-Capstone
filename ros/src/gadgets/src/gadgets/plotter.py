#! /usr/bin/env python

from collections import namedtuple

import matplotlib.pyplot as pp
from matplotlib.animation import FuncAnimation

import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32

from styx_msgs.msg import Lane, TrafficLightArray, Waypoint


Plots = namedtuple('Plots', ['route', 'next_light', 'red_lights', 'yellow_lights', 'green_lights', 'path', 'pose'])


class Plotter(object):
    def __init__(self):
        (figure, plotter) = pp.subplots()
        self.plotter = plotter
        self.route = ([], [])

        self.plots = Plots(
            route=plotter.plot([], [], 'b-')[0],
            next_light=plotter.plot([], [], 'mo', ms=14)[0],
            red_lights=plotter.plot([], [], 'ro', ms=10)[0],
            yellow_lights=plotter.plot([], [], 'yo', ms=10)[0],
            green_lights=plotter.plot([], [], 'go', ms=10)[0],
            path=plotter.plot([], [], 'k-', lw=3)[0],
            pose=plotter.plot([0], [0], 'ko', ms=6)[0]
        )

        self.subscribers = [
            rospy.Subscriber('/base_waypoints', Lane, self.route_callback, queue_size=1),
            rospy.Subscriber('/final_waypoints', Lane, self.path_callback, queue_size=1),
            rospy.Subscriber('/current_pose', PoseStamped, self.pose_callback, queue_size=1),
            rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.lights_callback, queue_size=1),
            rospy.Subscriber('/traffic_waypoint', Int32, self.next_light_callback, queue_size=1)
        ]

        def init():
            for plot in self.plots:
                plot.set_data([], [])

            return self.plots

        def animate(i):
            return self.plots

        self.animation = FuncAnimation(figure, animate, init_func=init, interval=100, repeat=False, blit=True)

    def route_callback(self, message):
        x = []
        y = []
        for waypoint in message.waypoints:
            position = waypoint.pose.pose.position
            x.append(position.x)
            y.append(position.y)

        x0 = np.min(x) - 100
        xn = np.max(x) + 100
        y0 = np.min(y) - 100
        yn = np.max(y) + 100
        self.plotter.axis([x0, xn, y0, yn])

        self.plots.route.set_data(x, y)
        self.route = (x, y)

    def path_callback(self, message):
        x = []
        y = []
        for waypoint in message.waypoints:
            position = waypoint.pose.pose.position
            x.append(position.x)
            y.append(position.y)

        self.plots.path.set_data(x, y)

    def pose_callback(self, message):
        x = message.pose.position.x
        y = message.pose.position.y
        self.plots.pose.set_data([x], [y])

    def lights_callback(self, message):
        locations = [([], []) for i in range(3)]
        for light in message.lights:
            data = locations[light.state]
            data[0].append(light.pose.pose.position.x)
            data[1].append(light.pose.pose.position.y)

        lights = ['red_lights', 'yellow_lights', 'green_lights']
        for (light, data) in zip(lights, locations):
            getattr(self.plots, light).set_data(*data)

    def next_light_callback(self, message):
        (x, y) = self.route
        a = message.data
        b = a + 1
        self.plots.next_light.set_data(x[a:b], y[a:b])

    def start(self):
        pp.show()


def main():
    rospy.init_node('plotter')

    plotter = Plotter()
    plotter.start()
