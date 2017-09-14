#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLight, TrafficLightArray

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

        self.current_pose = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        self.base_waypoints = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)
        self.traffic_waypoint = rospy.Subscriber('/traffic_waypoint', TrafficLightArray, self.traffic_cb, queue_size=1)

        # TODO: Add a subscriber for /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # List of route waypoints
        self.map_wp = []
        
        # velocity map
        self.velocity_map = []
        
        # If see a red traffic light ahead 
        self.stop_wp_active = False
        
        # current car position
        self.car_position = None
        
        # next wp index
        self.next_wp_idx = None

        rospy.spin()

    def pose_cb(self, msg):
        self.car_position = msg
        
        nearest_wp = self.find_nearest_wp(msg.pose.position.x, msg.pose.position.y)
        if nearest_wp < 0:
            return
        self.next_wp_idx = nearest_wp+1
            
        # Pub data
        lane = Lane()
        lane.header.frame_id = msg.header.frame_id
        lane.header.stamp = rospy.get_rostime()
        lane.waypoints = self.map_wp[nearest_wp:nearest_wp+LOOKAHEAD_WPS]
        
        n_stop_wp = len(self.velocity_map)
        for i in range(LOOKAHEAD_WPS):
            if (self.stop_wp_active==False):
                vel_2_use = MAX_SPEED
            else:
                if (i<n_stop_wp):
                    vel_2_use = self.velocity_map
                else:
                    vel_2_use = 0.0
            set_waypoint_velocity(lane.waypoints, lane.waypoints[i], vel_2_use)
        
        self.final_waypoints_pub.publish(lane)
        

    def waypoints_cb(self, waypoints):
        self.map_wp = waypoints.waypoints

        
        
    def calc_stop_wp_map(self, tl): # set the velocities when we see a red light ahead
        tl_wp_idx = self.find_nearest_wp(tl.pose.position.x, tl.pose.position.y)
        if (tl_wp_idx < 0) or (tl_wp_idx < self.next_wp_idx):
            return False
        
        n_wp = tl_wp_idx - self.next_wp_idx                                     # number of wp 
        vel = self.get_waypoint_velocity(self.map_wp[self.next_wp_idx]);
        if (vel < MIN_SPEED):
            vel = MIN_SPEED
        
        dv = vel / n_wp   # by how much to reduce the speed
        
        self.velocity_map = []
        for i in range(n_wp):
            v_next = v_next - dv
            if (v_next < 0.0):
                v_next = 0.0
            self.velocity_map.append(v_next)
            
        return True   
        
        
        
    def traffic_cb(self, msg):
        tl_vec = msg.tl_vec   # traffic lights that we identified

        tl_ahead = None
        d_min = 1000.0
        for tl in tl_vec:     # find the traffic light ahead of us
            d_curr = self.distance_sqr(self.car_position.pose.position.x, self.car_position.pose.position.y, tl.pose.pose.position.x, tl.pose.pose.position.y)
            next_car_position = self.map_wp[self.next_wp_idx]
            d_next = self.distance_sqr(self.next_car_position.pose.position.x, self.next_car_position.pose.position.y, tl.pose.pose.position.x, tl.pose.pose.position.y)
            if (d_curr < d_min) and (d_next < d_curr):   # we make sure the traffic light is getting closer
                d_min = d_curr
                tl_ahead = tl
        
        if (tl_ahead and (tl_ahead.state == 0)):   # red light ahead
            succ = self.calc_stop_wp_map(tl_ahead)
            self.stop_wp_active = succ
        else:
            self.stop_wp_active = False
        

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
        if len(self.map_wp) < 1:
            return -1

        position = self.map_wp[0].pose.pose.position
        d_min = self.distance_sqr(x, y, position.x, position.y)
        i_min = 0;

        for (i, waypoint) in enumerate(self.map_wp):
            position = waypoint.pose.pose.position
            d = self.distance_sqr(x, y, position.x, position.y)
            if d < d_min:
                d_min = d
                i_min = i

        return i_min


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
