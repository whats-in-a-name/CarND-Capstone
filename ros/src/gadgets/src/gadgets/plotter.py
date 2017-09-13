#! /usr/bin/env python

import matplotlib.pyplot as pp
from matplotlib.animation import FuncAnimation

import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped

from styx_msgs.msg import Lane, Waypoint


def main():
    rospy.init_node('plotter')

    (figure, plotter) = pp.subplots()

    route, = plotter.plot([], [], 'b-')
    def base_waypoints_callback(lane):
        x = []
        y = []
        for waypoint in lane.waypoints:
            position = waypoint.pose.pose.position
            x.append(position.x)
            y.append(position.y)

        x.append(lane.waypoints[0].pose.pose.position.x)
        y.append(lane.waypoints[0].pose.pose.position.y)

        x0 = np.min(x) - 100
        xn = np.max(x) + 100
        y0 = np.min(y) - 100
        yn = np.max(y) + 100
        plotter.axis([x0, xn, y0, yn])

        route.set_data(x, y)

    base_waypoints = rospy.Subscriber('/base_waypoints', Lane, base_waypoints_callback, queue_size=1)

    path, = plotter.plot([], [], 'r-', lw=4)
    def final_waypoints_callback(lane):
        x = []
        y = []
        for waypoint in lane.waypoints:
            position = waypoint.pose.pose.position
            x.append(position.x)
            y.append(position.y)

        path.set_data(x, y)


    final_waypoints = rospy.Subscriber('final_waypoints', Lane, final_waypoints_callback, queue_size=1)

    location, = plotter.plot([0], [0], 'ro', ms=8)
    def pose_callback(pose):
        x = pose.pose.position.x
        y = pose.pose.position.y
        location.set_data([x], [y])

    pose = rospy.Subscriber('/current_pose', PoseStamped, pose_callback, queue_size=1)

    def init():
        location.set_data([], [])
        route.set_data([], [])
        path.set_data([], [])
        return (location, route, path)

    def animate(i):
        return (location, route, path)

    animation = FuncAnimation(figure, animate, init_func=init, interval=100, repeat=False, blit=True)

    pp.show()
