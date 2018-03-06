#!/usr/bin/env python

from socketIO_client_nexus import SocketIO, SocketIONamespace, LoggingNamespace
from datetime import datetime
import rospy
import os
import signal
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32, Float32MultiArray


class TelemetryReporter:
    """
    This is used to report boat and environment statistics (e.g. position, heading, wind speed & direction) to a
    telemetry server for remote monitoring.
    """

    def __init__(self):
        self.reporter = None
        self.socketIO = None

        # Register as a ROS node
        rospy.init_node('telemetry_reporter', anonymous=True)

        # Listen for ROS boat position messages TODO Reference name from another file
        rospy.Subscriber("/boat/position", Pose2D, self.received_position_msg, queue_size=1)
        rospy.Subscriber("/boat/heading", Float32, self.received_heading_msg, queue_size=1)
        rospy.Subscriber('/control/heading/error_desired_rudder_pos', Float32MultiArray, self.received_heading_controller_error, queue_size=1)
        self.kpPub = rospy.Publisher("/control/p", Float32, queue_size=1)
        self.kiPub = rospy.Publisher("/control/i", Float32, queue_size=1)
        self.targetHeadingPub = rospy.Publisher('/control/target_heading', Float32, queue_size=1)

        # Listen for SIGINT (process termination requests)
        signal.signal(signal.SIGINT, self.terminate)

    def connect(self, server_address, port, use_ssl=False):
        print('Connecting to {} on port {} {}using SSL'.format(server_address, port, '' if use_ssl else 'without '))

        self.socketIO = SocketIO(server_address, port, LoggingNamespace, verify=(not use_ssl))
        self.reporter = self.socketIO.define(ReportingNamespace, '/reporting')
        self.reporter.on('set:control/heading/kp', self.on_kp_received)
        self.reporter.on('set:control/heading/ki', self.on_ki_received)
        self.reporter.on('set:control/target_heading', self.on_target_heading_received)
        self.socketIO.wait()  # Don't finish execution

    def terminate(self, *args):
        if self.socketIO:
            print('Disconnecting...')
            self.socketIO.disconnect()

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

    def received_heading_controller_error(self, msg):
        data = {
            'error': msg.data[0],
            'desiredRudderPos': msg.data[1]
        }
        self.report('control/heading/error_desired_rudder_pos', data)

    def received_wind_rel_msg(self, msg):
        self.report('weather/wind/rel', msg.data)

    def on_kp_received(self, (data)):
        try:
            self.kpPub.publish(Float32(data=float(data)))
        except ValueError:
            pass  # Invalid input

    def on_ki_received(self, (data)):
        try:
            self.kiPub.publish(Float32(data=float(data)))
        except ValueError:
            pass  # Invalid input

    def on_target_heading_received(self, (data)):
        try:
            self.targetHeadingPub.publish(Float32(data=float(data)))
        except ValueError:
            pass  # Invalid input


class ReportingNamespace(SocketIONamespace):

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
