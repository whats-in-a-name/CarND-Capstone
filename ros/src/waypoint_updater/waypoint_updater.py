#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

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


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        # map_wp is the base_waypoints we get
        self.map_wp = None

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        if self.map_wp is None:
            return
        nearest_wp = self.find_nearest_wp(msg.pose.position.x, msg.pose.position.y, self.map_wp)

        # Pub data
        lane = Lane()
        lane.header.frame_id = msg.header.frame_id
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = self.map_wp[nearest_wp:nearest_wp+LOOKAHEAD_WPS]
        self.final_waypoints_pub.publish(lane)
        pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.map_wp = waypoints.waypoints;
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

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
        return ((x0-x1)*(x0-x1) + (y0-y1)*(y0-y1))

    def find_nearest_wp(self, x, y, map_xyz):
        arg_min = 0;
        val_min = self.distance_sqr(x, y, map_xyz[0].pose.pose.position.x, map_xyz[0].pose.pose.position.y)
        for i in range(len(self.map_wp)):
            val_tmp = self.distance_sqr(x, y, map_xyz[i].pose.pose.position.x, map_xyz[i].pose.pose.position.y)
            if (val_tmp < val_min):
                val_min = val_tmp
                arg_min = i
        return i


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
