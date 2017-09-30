#!/usr/bin/env python

import eventlet

# "Greens" the module environment for eventlet.
# See: http://eventlet.net/doc/patching.html
eventlet.monkey_patch()

import socketio
import eventlet.wsgi
import time
from flask import Flask, render_template

from bridge import Bridge
from conf import conf

# Enforce use of eventlet for asynchronous operations.
sio = socketio.Server(async_mode='eventlet')

app = Flask(__name__)
dbw_enable = False
msgs = {}


@sio.on('connect')
def connect(sid, environ):
    print("connect ", sid)

def send(topic, data):
    msgs[topic] = data

bridge = Bridge(conf, send)

@sio.on('telemetry')
def telemetry(sid, data):
    global dbw_enable
    if data["dbw_enable"] != dbw_enable:
        dbw_enable = data["dbw_enable"]
        bridge.publish_dbw_status(dbw_enable)

    bridge.publish_odometry(data)

    # Copy messages before sending to avoid race conditions.
    # Clean up message buffer afterwards.
    sending = list(msgs.items())
    msgs.clear()

    # Send all messages to simulator.
    for (topic, data) in sending:
        sio.emit(topic, data=data, skip_sid=True)


@sio.on('control')
def control(sid, data):
    bridge.publish_controls(data)

@sio.on('obstacle')
def obstacle(sid, data):
    bridge.publish_obstacles(data)

@sio.on('lidar')
def obstacle(sid, data):
    bridge.publish_lidar(data)

@sio.on('trafficlights')
def trafficlights(sid, data):
    bridge.publish_traffic(data)

@sio.on('image')
def image(sid, data):
    bridge.publish_camera(data)

@sio.on('downlink'):
def downlink(sid, data):
    bridge.publish_lpf_status(data['lpf'])
    bridge.publish_lc_status(data['lc'])


if __name__ == '__main__':
    # wrap Flask application with engineio's middleware
    app = socketio.Middleware(sio, app)

    # deploy as an eventlet WSGI server
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app)
