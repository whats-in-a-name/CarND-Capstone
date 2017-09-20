#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf # not tensorflow - local library
import cv2
from traffic_light_config import config


STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.light_classifier = TLClassifier()

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        '''
        /vehicle/traffic_lights helps you acquire an accurate ground truth data source for the traffic light
        classifier, providing the location and current color state of all traffic lights in the
        simulator. This state can be used to generate classified images or subbed into your solution to
        help you work on another single component of the node. This topic won't be available when
        testing your solution in real life so don't rely on it in the final submission.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb, queue_size=1)
        sub6 = rospy.Subscriber('/camera/image_raw', Image, self.image_cb, queue_size=1)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

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
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
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

    def get_distance(self, x1, y1, x2, y2):
        " Euclidean distance "

        return ((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)) ** .5

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement

        min_distance = float('inf')
        closest_waypoint = None
        # print('waypoints: ', self.waypoints)
        # import pdb;pdb.set_trace()
        for idx, wp in enumerate(self.waypoints.waypoints):
            dist =  self.get_distance(
                wp.pose.pose.position.x, wp.pose.pose.position.y,
                pose.position.x, pose.position.y)
            if dist < min_distance:
                min_distance = dist
                closest_waypoint = idx

        return closest_waypoint

    def get_light_state(self):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        light_positions = config.light_positions

        # don't process if there are no waypoints or current position
        if self.waypoints and self.pose:

            # get car position
            car_position = self.get_closest_waypoint(self.pose.pose)
            car_x = self.waypoints.waypoints[car_position].pose.pose.position.x
            car_y = self.waypoints.waypoints[car_position].pose.pose.position.y

            #check if there is a traffic light close to the car
            min_distance = float('inf')
            light_pose = Pose()
            for light_x,light_y in light_positions:
                dist =  self.get_distance(light_x, light_y, car_x, car_y)
                if dist < min_distance:
                    min_distance = dist
                    light_pose.position.x = light_x
                    light_pose.position.y = light_y

            # check if it is close enough. set threshold experimentally
            light_wp = None
            threshold = 50
            if min_distance < threshold:
                # if close enough, get the closest waypoint to traffic light
                light_wp = self.get_closest_waypoint(light_pose)

            # if car is close to light, check if it is red
            if light_wp:
                state = self.get_light_state()
                return light_wp, state
                # return light_wp, TrafficLight.RED # for testing

        # else ignore it
        # self.waypoints = None # why set waypoints to None?
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
