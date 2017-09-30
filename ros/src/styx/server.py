#!/usr/bin/env python

import eventlet

# "Greens" the module environment for eventlet.
# See: http://eventlet.net/doc/patching.html
eventlet.monkey_patch()

import rospy
import socketio
import eventlet.wsgi
import time
from flask import Flask, render_template

from bridge import Bridge
from conf import conf


class Server(object):
    r'''Interface to the simulator.
    '''
    def __init__(self):
        r'''Create a new simulator interface.
        '''
        # Simulator ID.
        self.sid = None

        # DBW activation status.
        self.dbw_enable = False

        # Message buffer.
        self.msgs = {}

        # Bridge object between ROS and simulator.
        self.bridge = Bridge(conf, self.send)

        # Enforce use of eventlet for asynchronous operations.
        self.sio = socketio.Server(async_mode='eventlet')

        # Register event handlers
        self.sio.on('connect', self.connect)
        self.sio.on('telemetry', self.telemetry)
        self.sio.on('control', self.control)
        self.sio.on('obstacle', self.obstacle)
        self.sio.on('lidar', self.lidar)
        self.sio.on('trafficlights', self.trafficlights)
        self.sio.on('image', self.image)

    def send(self, topic, data):
        r'''Queue a message for submission to the simulator.
        '''
        self.msgs[topic] = data

    def spin(self):
        r'''Start the interface.
        '''
        # Wrap Flask application with engineio's middleware
        app = socketio.Middleware(self.sio, Flask(__name__))

        # deploy as an eventlet WSGI server
        eventlet.wsgi.server(eventlet.listen(('', 4567)), app)

    def connect(self, sid, environment):
        r'''Connect to the simulator.
        '''
        rospy.loginfo('Connected to simulator at ID "%s"' % sid)
        self.sid = sid

    def telemetry(self, sid, data):
        r'''Collect telemetry data from the simulator and send queued messages in return.
        '''
        if data["dbw_enable"] != self.dbw_enable:
            self.dbw_enable = data["dbw_enable"]
            self.bridge.publish_dbw_status(self.dbw_enable)

        # Copy messages before sending to avoid race conditions.
        # Clean up message buffer afterwards.
        sending = list(self.msgs.items())
        self.msgs.clear()

        # Send all messages to simulator.
        for (topic, data) in sending:
            self.sio.emit(topic, data=data, room=self.sid, skip_sid=True)

        self.bridge.publish_odometry(data)

    def control(self, sid, data):
        r'''Publish control data received from the simulator.
        '''
        self.bridge.publish_controls(data)

    def obstacle(self, sid, data):
        r'''Publish obstacle data received from the simulator.
        '''
        self.bridge.publish_obstacles(data)

    def lidar(self, sid, data):
        r'''Publish LiDAR data received from the simulator.
        '''
        self.bridge.publish_lidar(data)

    def trafficlights(self, sid, data):
        r'''Publish traffic light data received from the simulator.
        '''
        self.bridge.publish_traffic(data)

    def image(self, sid, data):
        r'''Publish video camera data received from the simulator.
        '''
        self.bridge.publish_camera(data)


def main():
    server = Server()
    server.spin()

@sio.on('downlink'):
def downlink(sid, data):
    bridge.publish_lpf_status(data['lpf'])
    bridge.publish_lc_status(data['lc'])


if __name__ == '__main__':
    main()
