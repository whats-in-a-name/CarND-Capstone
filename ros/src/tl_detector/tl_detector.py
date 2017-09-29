#!/usr/bin/env python

import cv2
import numpy as np
import rospy
import tf # not tensorflow - local library
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, Pose, PoseStamped
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int32
from tf.transformations import euler_from_quaternion

from light_classification.tl_classifier import TLClassifier
from styx_msgs.msg import TrafficLight, TrafficLightArray
from styx_msgs.msg import Lane

import os

STATE_COUNT_THRESHOLD = 3


def distance2(a, b):
    r'''Return the squared distance between `a` and `b`.

        If `a` is a `n X d` matrix and `b` a `d`-dimensional array, return a
        1-D array of distances between each `a[i]` and `b`.
    '''
    d = (a - b) ** 2.0
    if len(d.shape) > 1:
        return np.sum(d, axis=1).flat
    else:
        return np.sum(d)


def distance(a, b):
    r'''Return the Euclidean distance between `a` and `b`.

        If `a` is a `n X d` matrix and `b` a `d`-dimensional array, return a
        1-D array of distances between each `a[i]` and `b`.
    '''
    return np.sqrt(distance2(a, b))


def closest(P, p):
    r'''Given a `n X d` matrix `P` and a d-dimensional point `p`, return
        the index `i` such that `P[i]` is the point closest to `p` in `P`.
    '''
    return np.argmin(distance2(P, p))


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.light_classifier = TLClassifier()

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.stop_indexes = None
        self.traffic_lights = None

        self.subscribers = [
            rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1),
            rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1),
            rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1),
            rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb, queue_size=1)
        ]

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.stop_lines = np.array(self.config['stop_line_positions'])

        # TODO: Move camera info generation to a separate node.
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']
        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']
        cx = 0.465 * image_width
        cy = 1.1 * image_height
        camera_info = CameraInfo()
        camera_info.header.frame_id = '/base_link'
        camera_info.width = image_width
        camera_info.height = image_height
        camera_info.distortion_model = 'plumb_bob'
        camera_info.D = [0, 0, 0, 0, 0]
        camera_info.K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        camera_info.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        camera_info.P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]

        self.camera = PinholeCameraModel()
        self.camera.fromCameraInfo(camera_info)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        self.image_zoomed = rospy.Publisher('/image_zoomed', Image, queue_size=1)

        self.bridge = CvBridge()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        # keeps python from exiting until node is stopped
        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        if self.waypoints is not None:
            return

        self.waypoints = np.array([[w.pose.pose.position.x, w.pose.pose.position.y] for w in waypoints.waypoints])
        self.stop_indexes = np.array([closest(self.waypoints, light) for light in self.stop_lines])

    def traffic_cb(self, msg):
        if self.traffic_lights is not None:
            return

        def pose(light):
            position = light.pose.pose.position
            return [position.x, position.y, position.z]

        self.traffic_lights = np.array([pose(light) for light in msg.lights])

    def image_cb(self, msg):
        r'''Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        '''
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        r'''Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        '''
        p = np.array([pose.position.x, pose.position.y])
        return closest(self.waypoints, p)

    def project_to_image_plane(self, point3d):
        r'''Project point from 3D world coordinates to 2D camera image location

        Args:
            point3d (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image
        '''
        stamp = self.camera_image.header.stamp

        p_world = PointStamped()
        p_world.header.seq = self.camera_image.header.seq
        p_world.header.stamp = stamp
        p_world.header.frame_id = '/world'
        p_world.point.x = point3d[0]
        p_world.point.y = point3d[1]
        p_world.point.z = point3d[2]

        # Transform point from world to camera frame.
        self.listener.waitForTransform('/base_link', '/world', stamp, rospy.Duration(1.0))
        p_camera = self.listener.transformPoint('/base_link', p_world)

        # The navigation frame has X pointing forward, Y left and Z up, whereas the
        # vision frame has X pointing right, Y down and Z forward; hence the need to
        # reassign axes here.
        x = -p_camera.point.y
        y = -p_camera.point.z
        z = p_camera.point.x

        return self.camera.project3dToPixel((x, y, z))

    def get_light_state(self, light):
        r'''Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        '''
        if not self.has_image:
            self.prev_light_loc = None
            return False

        (x_tl, y_tl) = self.project_to_image_plane(light + np.array([-0.7, -0.7, 1.0]))
        (x_br, y_br) = self.project_to_image_plane(light + np.array([0.5, 0.5, -1.0]))

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        shape = cv_image.shape

        a = int(max(0, y_tl)+5)
        b = int(min(y_br, shape[0]))
        c = int(max(0, min(x_br, x_tl)))
        d = int(min(max(x_br, x_tl)-20, shape[1]))

        if (b - a) < 50 or (d - c) < 50:
            return TrafficLight.UNKNOWN

        # original 100, 140
        zoomed = cv2.resize(cv_image[a:b, c:d], (40, 90))

        state = self.light_classifier.get_classification(zoomed)
        self.image_zoomed.publish(self.bridge.cv2_to_imgmsg(zoomed, 'bgr8'))
        return state

    def process_traffic_lights(self):
        r'''Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        '''

        # Entire function has changed? what exacty is going on here?
        # is it failing to classify all topics, or is it not listening at all?

        # Don't process if there are no waypoints or current position.
        if any(v is None for v in [self.waypoints, self.stop_indexes, self.pose]):
            return (-1, TrafficLight.UNKNOWN)

        # Get car's position.
        i_car = self.get_closest_waypoint(self.pose.pose)
        p_car = self.waypoints[i_car]

        # Get the closest stop line's index.
        j_stop = closest(self.stop_lines, p_car)
        i_stop = self.stop_indexes[j_stop]

        # If the car is ahead of the closest stop line, get the next one.
        if i_car > i_stop:
            j_stop = (j_stop + 1) % len(self.stop_indexes)
            i_stop = self.stop_indexes[j_stop]

        # Don't process if the closest stop line is too far.
        if distance(p_car, self.stop_lines[j_stop]) > 50.0:
            return (-1, TrafficLight.UNKNOWN)

        # Return the index and state of the traffic light.
        state = self.get_light_state(self.traffic_lights[j_stop])
        return (i_stop, state)


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
