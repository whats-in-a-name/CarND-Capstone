import numpy as np
import os
import pickle
import cv2
from math import ceil, floor

# ROS
from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    """ Approach based on CV to reduce computational load """

    def __init__(self):
        """ Initialize model and load weights """

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
        labels = []
        windows = self.select_windows(image)
        for w in windows: 
            box_img = image[w[0][1]:w[1][1], w[0][0]:w[1][0], :]
            label = self.classify(box_img.copy())
            labels.append(label)

        labels = np.array(labels)
        dist = np.bincount(labels)
        # number of true identifications based on number of sliding windows
        threshold = ceil(len(windows)/10)
        if np.sum(dist[:3]) >= threshold:
            klass = np.argmax(dist[:3])
        else:
            klass = 3

        return self.encoder[klass]

    def classify(self,image):  

        # define thresholds to remove sky
        rgb_threshold = [200, 200, 200]

        #retain pixels meet by color criterion, and blacken out the remaining
        thresholds = (image[:, :, 0] < rgb_threshold[0]) & \
                     (image[:, :, 1] < rgb_threshold[1]) & \
                     (image[:, :, 2] > rgb_threshold[2])
        image[thresholds] = [0,0,0]    
        
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
            mean = box_mean/img_mean
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
            return self.model.predict(features)[0]
        return 3


    def slide_window(self, img, x_start_stop=[None, None], y_start_stop=[None, None], 
                       xy_window=[(64, 64)], xy_overlap=(0.5, 0.5), debug=False):
        """ Define a function that takes an image, start and stop positions in both x and y, 
            window size (x and y dimensions), and overlap fraction (for both x and y)
            
            Improved version of slide_window:
            - establish a list of different windows sizes (accept multiple xy_windows)
            - restrict search to area of images the vehicles should appear (y_stop defaults to 350)
            - vehicles near the horizon will be smaller - pending (pending)
            
        """
        
        # If x and/or y start/stop positions not defined, set to image size
        if x_start_stop[0] == None:
            x_start_stop[0] = 0
        if x_start_stop[1] == None:
            x_start_stop[1] = img.shape[1]
        if y_start_stop[0] == None:
            y_start_stop[0] = 0
        if y_start_stop[1] == None:
            y_start_stop[1] = img.shape[0]

        # Compute the span of the region to be searched    
        xspan = x_start_stop[1] - x_start_stop[0]
        yspan = y_start_stop[1] - y_start_stop[0]

        # Initialize a list to append window positions to
        window_list = []

        # Compute the number of pixels per step in x/y
        nx_pix_per_step = np.int(xy_window[0]*(1 - xy_overlap[0]))
        ny_pix_per_step = np.int(xy_window[1]*(1 - xy_overlap[1]))
        # Compute the number of windows in x/y
        nx_windows = np.int(xspan/nx_pix_per_step) - 1
        ny_windows = np.int(yspan/ny_pix_per_step) - 1

        # Loop through finding x and y window positions
        for ys in range(ny_windows):
            for xs in range(nx_windows):
                # Calculate window position
                startx = xs*nx_pix_per_step + x_start_stop[0]
                endx = startx + xy_window[0]
                starty = ys*ny_pix_per_step + y_start_stop[0]
                endy = starty + xy_window[1]
                # Append window position to list
                window_list.append(((startx, starty), (endx, endy)))
                    
        # Return the list of windows
        return window_list


    def select_windows(self,image):

        h, w = (132, 62)
        y_start_stop = [0, 600]
        x_start_stop = [0, 800]
        xy_overlap=(0.2, .3)
        xy_windows = (w,h)

        windows = self.slide_window(image, x_start_stop=x_start_stop, 
            y_start_stop=y_start_stop, xy_window=xy_windows, xy_overlap=xy_overlap)
                        
        filtered_windows = []
        for w in windows:
            box_img = image[w[0][1]:w[1][1], w[0][0]:w[1][0], :]
            gray = cv2.cvtColor(box_img, cv2.COLOR_RGB2GRAY)
            canny = cv2.Canny(gray, 100, 120)
            if not np.mean(canny) < 10:
                filtered_windows.append(w)

        return filtered_windows

