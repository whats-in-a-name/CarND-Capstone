#!/usr/bin/env python

from copy import deepcopy

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLight, TrafficLightArray
from std_msgs.msg import Int32
import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MAX_SPEED = 10.0
MIN_SPEED = 1.0


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.subscriber = [
            rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1),
            rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb, queue_size=1),
            rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1),
            rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)
        ]

        # TODO: Add a subscriber for /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.current_velocity = 0

        # List of route waypoints
        self.map_wp = []

        # velocity map
        self.velocity_map = []

        # If see a red traffic light ahead
        self.stop_wp_active = False

        # next wp index
        self.waypoint_idx = 0

        rospy.spin()

    def pose_cb(self, msg):
        nearest_wp = self.find_nearest_wp(msg.pose.position.x, msg.pose.position.y)
        if nearest_wp < 0:
            return

        if len(self.velocity_map) is 0:
            return

        self.waypoint_idx = nearest_wp

        a = nearest_wp
        b = min(nearest_wp + LOOKAHEAD_WPS, len(self.map_wp))

        # Pub data
        lane = Lane()
        lane.header.frame_id = msg.header.frame_id
        lane.header.stamp = rospy.get_rostime()
        lane.waypoints = deepcopy(self.map_wp[a:b])

        if self.stop_wp_active:
            n_stop_wp = len(self.velocity_map)
            for i in range(b - a):
                v = self.velocity_map[i] if i < n_stop_wp else 0.0
                self.set_waypoint_velocity(lane.waypoints, i, v)

        self.final_waypoints_pub.publish(lane)

    def current_velocity_cb(self, msg):
        self.current_velocity = msg.twist.linear.x

    def waypoints_cb(self, waypoints):
        self.map_wp = waypoints.waypoints

    def calc_stop_wp_map(self, tl_wp_idx): # set the velocities when we see a red light ahead
        if (tl_wp_idx < 0) or (tl_wp_idx <= self.waypoint_idx):
            return False

        n_wp = tl_wp_idx - self.waypoint_idx
        vel = self.current_velocity
        dv = 1.5 * vel ** (1.0 / n_wp)
        self.velocity_map = []
        for i in range(n_wp):
            vel /= dv
            if (vel < MIN_SPEED):
                vel = 0.0
            self.velocity_map.append(vel)

        # Ensures the car is stopped at the end of the sequence.
        self.velocity_map[-1] = 0.0

        return True

    def traffic_cb(self, msg):
        self.stop_wp_active = self.calc_stop_wp_map(msg.data)

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

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
        waypoints = self.map_wp[self.waypoint_idx:]
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

        return self.waypoint_idx + i_min


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
