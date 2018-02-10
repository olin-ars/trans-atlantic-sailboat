#!/usr/bin/env python

from socketIO_client_nexus import SocketIO, BaseNamespace
from datetime import datetime
from time import sleep
import json
# import math
import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32


class TelemetryReporter:

    def __init__(self):
        self.reporter = None

        # Register as a ROS node
        rospy.init_node('TelemetryReporter', anonymous=True)

        # Listen for ROS boat position messages TODO Reference name from another file
        rospy.Subscriber("/boat/position", Pose2D, self.received_position_msg, queue_size=5)
        rospy.Subscriber("/boat/heading", Float32, self.received_heading_msg, queue_size=5)

    def connect(self, server_address, port):
        with SocketIO(server_address, port) as socketIO:
            self.reporter = socketIO.define(ReportingNamespace, '/reporting')
            # socketIO.wait()

        # spin() simply keeps Python from exiting until this node is stopped
        rospy.spin()

    def report(self, message_type, data):
        if not self.reporter:
            print('Not connected to /reporter')
            return

        payload = {
            'type': message_type,
            'data': data,
            'timestamp': str(datetime.now())
        }
        print('Reporting', payload)
        # payload = json.dumps(payload)
        self.reporter.emit('report', payload)

    def received_position_msg(self, msg):
        # Build up the message data
        data = {
            'x': msg.x,
            'y': msg.y
        }
        # Transmit the data to the server
        self.report('position', data)

    def received_heading_msg(self, msg):
        # Build up the message data
        data = msg.data
        # Transmit the data to the server
        self.report('heading', data)


class ReportingNamespace(BaseNamespace):

    def on_connect(self):
        print('Connected to reporting')

    def on_aaa_response(self, *args):
        print('Received aaa:', args)


if __name__ == '__main__':
    tr = TelemetryReporter()
    tr.connect('127.0.0.1', 1234)
