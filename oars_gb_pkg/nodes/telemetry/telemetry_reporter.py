#!/usr/bin/env python

from socketIO_client_nexus import SocketIO, BaseNamespace
from datetime import datetime
import rospy
import os
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32


class TelemetryReporter:
    """
    This is used to report boat and environment statistics (e.g. position, heading, wind speed & direction) to a
    telemetry server for remote monitoring.
    """

    def __init__(self):
        self.reporter = None

        # Register as a ROS node
        rospy.init_node('telemetry_reporter', anonymous=True)

        # Listen for ROS boat position messages TODO Reference name from another file
        rospy.Subscriber("/boat/position", Pose2D, self.received_position_msg, queue_size=5)
        rospy.Subscriber("/boat/heading", Float32, self.received_heading_msg, queue_size=5)

    def connect(self, server_address, port, use_ssl=False):
        print('Connecting to {} on port {} {}using SSL'.format(server_address, port, '' if use_ssl else 'without '))

        with SocketIO(server_address, port, verify=(not use_ssl)) as socketIO:
            self.reporter = socketIO.define(ReportingNamespace, '/reporting')

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


if __name__ == '__main__':
    # Get server URI and port from environment variables
    server = os.environ.get('OARS_SERVER_URI', 'localhost')
    port = os.environ.get('OARS_SERVER_PORT', 1234)
    ssl = os.environ.get('OARS_SERVER_USE_SSL', False)

    tr = TelemetryReporter()
    # tr.connect(server, port)
    tr.connect(server, port, ssl)
