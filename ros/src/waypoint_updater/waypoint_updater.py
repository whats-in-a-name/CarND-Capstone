#!/usr/bin/env python

import math
from copy import deepcopy

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLight, TrafficLightArray
from std_msgs.msg import Int32


class WaypointUpdater(object):
    r'''The Waypoint Updater extracts a sequence of waypoints from the route,
        annotates it with target speeds, and publishes the result for downstream
        nodes to use as a path for the car to follow.
    '''
    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.subscribers = [
            rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1),
            rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb, queue_size=1),
            rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1),
            rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)
        ]

        self.final_waypoints = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.v_min = rospy.get_param('~v_min', 1.0)
        self.n = rospy.get_param('~lookahead', 200)

        self.current_velocity = 0

        # List of route waypoints
        self.waypoints = []

        # List of velocities up to the stop point ahead.
        self.v_stopping = []

        # Index of the waypoint closest to the car.
        self.i_car = 0

    def spin(self):
        rospy.spin()

    def pose_cb(self, msg):
        i_car = self.find_nearest_wp(msg.pose.position.x, msg.pose.position.y)
        if i_car < 0:
            return

        self.i_car = i_car

        a = i_car
        b = min(i_car + self.n, len(self.waypoints))

        # Pub data
        lane = Lane()
        lane.header.frame_id = msg.header.frame_id
        lane.header.stamp = rospy.get_rostime()
        lane.waypoints = deepcopy(self.waypoints[a:b])

        v_stopping = self.v_stopping
        if len(v_stopping) > 0:
            n_stopping = len(v_stopping)
            for i in range(b - a):
                v = v_stopping[i] if i < n_stopping else 0.0
                self.set_waypoint_velocity(lane.waypoints, i, v)

        self.final_waypoints.publish(lane)

    def current_velocity_cb(self, msg):
        self.current_velocity = msg.twist.linear.x

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        self.v_stopping = []

        i_light = msg.data
        if i_light < 0 or i_light < self.i_car:
            return

        n = 1 + i_light - self.i_car
        self.v_stopping = [0.0] * n
        self.v_stopping[:-3] = [self.v_min] * (n - 3)

        if n < 7:
            return

        v_max = max(self.current_velocity, self.v_min)
        dv = v_max ** (1.0 / n)

        for i in range(n - 6):
            self.v_stopping[i] = dv ** (n - 1 - i)

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def distance_sqr(self, x0, y0, x1, y1):
        return (x0 - x1) ** 2.0 + (y0 - y1) ** 2.0

    def find_nearest_wp(self, x, y):
        waypoints = self.waypoints[self.i_car:]
        if len(waypoints) < 1:
            return -1

        position = waypoints[0].pose.pose.position
        d_min = self.distance_sqr(x, y, position.x, position.y)
        i_min = 0;

        for (i, waypoint) in enumerate(waypoints):
            position = waypoint.pose.pose.position
            d = self.distance_sqr(x, y, position.x, position.y)
            if d < d_min:
                d_min = d
                i_min = i

        return self.i_car + i_min


if __name__ == '__main__':
    node = WaypointUpdater()
    node.spin()
