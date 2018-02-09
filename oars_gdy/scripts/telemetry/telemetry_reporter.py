#!/usr/bin/env python

from socketIO_client_nexus import SocketIO, BaseNamespace
from datetime import datetime
from time import sleep
import json
# import math
# import rospy
# from geometry_msgs.msg import Twist
# from std_msgs.msg import Int8MultiArray


class TelemetryReporter:

    def __init__(self):
        self.reporter = None

    def connect(self, server_address, port):
        with SocketIO(server_address, port) as socketIO:
            self.reporter = socketIO.define(ReportingNamespace, '/reporting')
            # socketIO.wait()

    def report(self, messageType, data):
        if not self.reporter:
            print('Not connected to /reporter')
            return

        payload = {
            'dataType': messageType,
            'dataValue': data,
            'timestamp': str(datetime.now())
        }
        print('Reporting', payload)
        # payload = json.dumps(payload)
        self.reporter.emit('report', payload)


class ReportingNamespace(BaseNamespace):

    def on_connect(self):
        print('Connected to reporting')

    def on_aaa_response(self, *args):
        print('Received aaa:', args)


if __name__ == '__main__':
    tr = TelemetryReporter()
    tr.connect('127.0.0.1', 1234)
    heading = 1
    while True:
        tr.report(messageType='heading', data=heading)
        heading += 1
        if heading > 360:
            heading = 0
        sleep(0.3)
