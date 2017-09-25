import numpy as np
import os
import pickle
import cv2
from math import ceil, floor

# ROS
import rospy
from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    r'''Approach based on CV to reduce computational load.
    '''
    def __init__(self):
        r'''Initialize model and load weights.
        '''
        root_lib = os.path.dirname(os.path.realpath(__file__))
        data_path = root_lib + '/model_data/'
        print("Data path: {}".format(data_path))

        self.model = pickle.load(open(data_path + 'svc_model.p', 'rb'))

        self.encoder = {
            0: TrafficLight.RED,
            1: TrafficLight.YELLOW,
            2: TrafficLight.GREEN,
            3: TrafficLight.UNKNOWN
        }

    def get_classification(self, image):
        # Set all pixels not in the (red, yellow, green) range to zero.
        hls = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
        hue = hls[..., 0]
        thresholds = (hue > 85) & (hue < 150)
        image[thresholds] = [0, 0, 0]

        # define box size
        h, w = image.shape[:2]
        box_h = int(ceil(h/3))
        # get image mean
        img_mean = np.mean(image)
        features = []
        for i in range(3):
            # get box
            box = image[i*box_h:(i+1)*box_h, :]
            box_mean = np.mean(box)
            # calculate box features
            mean = box_mean / img_mean
            # avoid division by zero
            if box_mean == 0:
                box_mean = 0.1
            red = np.mean(box[:, :, 0]) / box_mean
            green = np.mean(box[:, :, 1]) / box_mean
            blue = np.mean(box[:, :, 2]) / box_mean
            features.extend([mean, red, green, blue])

        # predict (array as input)
        features = np.array(features).reshape(1, -1)

        if sum(np.isnan(features).any(axis=1)) == 0:
            c = self.model.predict(features)[0]
            return self.encoder[c]

        return TrafficLight.UNKNOWN
