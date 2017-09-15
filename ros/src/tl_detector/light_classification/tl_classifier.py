import numpy as np
import os

# classifier
import cv2
from keras.models import load_model
from keras import backend as K


# ROS
from styx_msgs.msg import TrafficLight


class TLClassifier(object):

    def __init__(self):
        """ Initialize model and load weights """

        # set backgroun as theano
        # tensorflow conflicting with webservice
        backend = "theano"
        if K.backend() != backend:
            os.environ['KERAS_BACKEND'] = backend
            reload(K)
            assert K.backend() == backend

        root_lib = os.path.dirname(os.path.realpath(__file__))
        data_path = root_lib + '/model_data/'
        print("Data path: {}".format(data_path))

        self.model = load_model(data_path + 'tl_classifier_py2.h5')
        self.model._make_predict_function()

        self.encoder = {
            0: TrafficLight.RED,
            1: TrafficLight.YELLOW,
            2: TrafficLight.GREEN
        }


    def get_classification(self,image):  
        """ Predict using pre-trained model """

        # crop - remove car
        image = image[0:770, :]

        # resize
        height = 50
        width = 88
        image = cv2.resize(image, (width,height))

        # scale and center
        image = (image/255) -.5

        # reshape as array
        image = image.reshape((1,height,width,3))

        # predict (array as input)
        pred = self.model.predict(image)
        klass = int(round((pred[0][0])))

        return self.encoder[klass]

