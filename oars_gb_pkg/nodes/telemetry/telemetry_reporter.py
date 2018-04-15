#!/usr/bin/env python

from socketIO_client_nexus import SocketIO, SocketIONamespace
from datetime import datetime
import rospy
import os
import signal
import subprocess
from geometry_msgs.msg import Pose2D
from genpy import Message
from std_msgs.msg import Float32, Float64, Float32MultiArray, Float64MultiArray, String, UInt8, UInt16, UInt32, UInt64,\
                        UInt8MultiArray, UInt16MultiArray, UInt32MultiArray, UInt64MultiArray
from sensor_msgs.msg import Image
from oars_gb.msg import GridMap, WaypointList

ROS_MSG_TYPES = {
    'Float32': Float32,
    'Float64': Float64,
    'Float32MultiArray': Float32MultiArray,
    'Float64MultiArray': Float64MultiArray,
    'GridMap': GridMap,
    'Image': Image,
    'Pose2D': Pose2D,
    'String': String,
    'UInt8': UInt8,
    'UInt16': UInt16,
    'UInt32': UInt32,
    'UInt64': UInt64,
    'UInt8MultiArray': UInt8MultiArray,
    'UInt16MultiArray': UInt16MultiArray,
    'UInt32MultiArray': UInt32MultiArray,
    'UInt64MultiArray': UInt64MultiArray,
}
ERROR_TOPIC_NAME = '/logging/errors'


class TelemetryReporter:
    """
    This is used to report boat and environment statistics (e.g. position, heading, wind speed & direction) to a
    telemetry server for remote monitoring.
    """

    def __init__(self):
        self.reporter = None
        self.socketIO = None
        self.rosbag_process = None

        # Register as a ROS node
        rospy.init_node('telemetry_reporter')

        # Listen for ROS boat position messages TODO Reference name from another file
        self.subscribers = {}
        self.publishers = {}

        # Listen for SIGINT (process termination requests)
        signal.signal(signal.SIGINT, self.terminate)

    def connect(self, server_address, port, use_ssl=False):
        print('Connecting to {} on port {} {}using SSL'.format(server_address, port, '' if use_ssl else 'without '))

        self.socketIO = SocketIO(server_address, port, verify=(not use_ssl))
        self.reporter = self.socketIO.define(ReportingNamespace, '/reporting')
        self.reporter.on('publishROSMessage', self._handle_server_publish_msg)
        self.reporter.on('getTopics', self._handle_get_published_topics_request)
        self.reporter.on('startStopRosbag', self.start_stop_rosbag)
        self.socketIO.wait()  # Don't finish execution

    def terminate(self, *args):
        if self.socketIO:
            print('Disconnecting...')
            self.socketIO.disconnect()

    def listen_to_topic(self, topic_name, msg_type, save_to_db=True, max_transmit_rate=0, queue_size=1):
        """
        Sets up a subscriber on the given ROS topic.
        :param {string} topic_name: the name of the ROS topic
        :param {string|genpy.Message} msg_type: the type of ROS message
        :param {boolean} save_to_db: whether or not messages sent to the server should be stored in the database
            or just forwarded on to connected observers
        :param {number} max_transmit_rate: the maximum rate at which to broadcast updates to the server (Hz)
        :param {number} queue_size: the maximum number of ROS messages to hold in the buffer
        :return: True if a subscriber was successfully registered, False otherwise
        """
        # Try to resolve the message type if it is not already a ROS message class
        if not issubclass(msg_type, Message):
            if type(msg_type) == str:
                # Try to look up in our dictionary
                msg_type = ROS_MSG_TYPES.get(msg_type)
                if not msg_type:  # Couldn't find given message type in dictionary
                    return False
            else:  # Not a subclass of genpy.Message and not a string
                return False

        # Define a handler to serialize the ROS message and send it to the server
        def msg_handler(msg):
            self.transmit_message(topic_name, msg, msg_type, save_to_db=save_to_db)

        self.subscribers[topic_name] = rospy.Subscriber(topic_name, msg_type, msg_handler, queue_size=queue_size)
        print('Registered listener for ' + topic_name)
        return True

    def transmit_message(self, topic_name, msg, msg_type, save_to_db=True):
        """
        Transmits a message to the server.
        :param {string} topic_name: the name of the ROS topic
        :param {genpy.Message} msg: the actual ROS message
        :param {type} msg_type: the type of ROS message
        :param {boolean} save_to_db: whether or not the message should be saved to the database
        """
        if not self.reporter:
            print('Not connected to /reporter')
            return

        payload = {
            'topicName': topic_name,
            'type': msg_type.__name__,
            'data': self._serialize_ros_message(msg),
            'timestamp': str(datetime.now()),
            'saveToDb': save_to_db
        }
        self.reporter.emit('message', payload)

    @staticmethod
    def _serialize_ros_message(msg):
        """
        Converts a ROS message into a dictionary that can be serialized and sent to the server.
        :param {genpy.Message} msg: the raw ROS message
        :return: a dictionary with the important values from the message
        """
        msg_type = type(msg)
        if msg_type == Pose2D:
            return {
                'x': msg.x,
                'y': msg.y,
                'theta': msg.theta
            }

        elif msg_type == Float32 \
                or msg_type == Float64 \
                or msg_type == UInt8 \
                or msg_type == UInt16 \
                or msg_type == UInt32 \
                or msg_type == UInt64:
            return msg.data

        elif msg_type == Float32MultiArray \
                or msg_type == Float64MultiArray:
            return list(msg.data)

        elif msg_type == GridMap:
            return {
                'grid': TelemetryReporter._serialize_ros_image_message(msg.grid),
                'minLatitude': msg.minLatitude.data,
                'maxLatitude': msg.maxLatitude.data,
                'minLongitude': msg.minLongitude.data,
                'maxLongitude': msg.maxLongitude.data
            }

        elif msg_type == WaypointList:
            points = []
            for lat, long in zip(msg.latitudes.data, msg.longitudes.data):
                points.append({'lat': lat, 'long': long})
            return points

        return None

    @staticmethod
    def _serialize_ros_image_message(msg):
        """
        Converts a ROS Image message into a dictionary.
        :param msg: the ROS Image message (from sensor_msgs.msg)
        :return: a dictionary with the image width, height, encoding, and pixel data
        """
        return {
            'height': msg.height,
            'width': msg.width,
            'encoding': msg.encoding,
            'data': msg.data
        }

    def publish_message(self, topic_name, msg_type, data):
        """
        Publishes a message to the given ROS topic.
        :param {string} topic_name: the name of the ROS topic to publish to
        :param {type} msg_type: the ROS message type
        :param data: the message payload
        """
        # Turn the data into a ROS message
        msg = self._build_ros_msg(data, msg_type)
        
        if msg is not None:
            # Register a publisher if one is not already registered for this topic
            if topic_name not in self.publishers:
                self.configure_publisher(topic_name, msg_type)
                # Note: Messages sent shortly after configuring publisher will likely be lost.
                # See https://github.com/ros/ros_comm/issues/176 for more info.
    
            self.publishers[topic_name].publish(msg)

    def _convert_unicode(self, data):
        """
        Recursively converts attributes unicode string attributes or elements on an object to UTF-8 strings.
        :param data: a dict, list, or string to convert to UTF-8.
        :return: the converted object
        """
        if isinstance(data, dict):
            return {self._convert_unicode(key): self._convert_unicode(value)
                    for key, value in data.iteritems()}
        elif isinstance(data, list):
            return [self._convert_unicode(element) for element in data]
        elif isinstance(data, unicode):
            return data.encode('utf-8')
        else:
            return data

    def configure_publisher(self, topic_name, msg_type, queue_size=1):
        """
        Sets up a ROS message publisher.
        :param topic_name: the name of the ROS topic to publish to
        :param msg_type: the type of messages to publish
        :param queue_size: the number of messages to queue up for sending before dropping old ones
        """
        if topic_name not in self.publishers:
            self.publishers[topic_name] = rospy.Publisher(topic_name, msg_type, queue_size=queue_size)

    def _build_ros_msg(self, data, msg_type):
        """
        Constructs a ROS message to transmit the given data.
        :param data: the data to hold in the message
        :param {genpy.Message} msg_type: the type of message to construct
        :return: a ROS message containing the data
        """
        try:
            # TODO Ints and arrays of ints

            if msg_type == UInt8 \
                    or msg_type == UInt16:  # Integers
                return msg_type(data=int(data))
            if msg_type == Float32 \
                    or msg_type == Float64:  # Floating-point numbers
                return msg_type(data=float(data))

            if msg_type == Float32MultiArray or msg_type == Float64MultiArray:  # Array of floating-point numbers
                return msg_type(data=[float(x) for x in data])

            if msg_type == String:
                return msg_type(data=data)

            if msg_type == Pose2D and 'x' in data and 'y' in data:
                return Pose2D(x=float(data['x']), y=float(data['y']))

        except ValueError:
            error_msg = 'Problem parsing data: ' + data + '. Supposed to be of type ' + str(msg_type)
            self.publish_message(ERROR_TOPIC_NAME, String, error_msg)

        return None

    def _handle_server_publish_msg(self, msg):
        """
        Handles WebSockets "publish" events from the server by publishing their info to ROS.
        :param msg: the WebSockets message payload
        """
        # Convert Unicode strings to UTF-8
        msg = self._convert_unicode(msg)

        # Make sure message type is specified and try to resolve it to ROS message class
        if 'type' in msg:
            msg_type = ROS_MSG_TYPES.get(msg['type'])
            if not msg_type:
                return self._publish_error_msg('Could not understand message type "' + msg['type'] + '".')
        else:
            return self._publish_error_msg('Attribute "type" must be specified.')
        # Check for the topic name
        if 'topicName' not in msg:
            return self._publish_error_msg('Attribute "topicName" must be specified.')
        # Check for the message payload
        elif 'data' not in msg:
            self._publish_error_msg('Attribute "data" must be specified.')

        self.publish_message(msg['topicName'], msg_type, msg['data'])

    def _publish_error_msg(self, error):
        """
        Logs an error by publishing it to the error log topic.
        :param {string} error: the content of the message
        """
        print(error)
        self.publish_message(ERROR_TOPIC_NAME, String, String(data=error))

    def _handle_get_published_topics_request(self, data):
        """
        Gets a list of the published ROS topics and their associated message types
        :param
        :return: A list of dictionaries of the form {name: topicName, type: topicType}
        :rtype list
        """
        res = []
        # Reformat the list items as dictionaries rather than arrays
        for topic in rospy.get_published_topics():
            res.append({
                'name': topic[0],
                'type': topic[1]
            })
        # Send the topics to the server
        self.reporter.emit('topicList', res)

    def start_stop_rosbag(self, data):
        """
        Handles a request from an observer to start or stop rosbag on the boat.
        This currently only supports one instance of rosbag. Attempts to start
        a second instance without first ending the first will be ignored.
        :param data: the WebSockets message from the observer
        """
        if 'action' not in data:
            return

        action = data['action']
        if self.rosbag_process and action == 'stop':  # Stop rosbag
            print('Stopping rosbag...')
            os.killpg(os.getpgid(self.rosbag_process.pid), signal.SIGINT)
            self.rosbag_process = None

        elif action == 'start':  # Start rosbag
            print('Starting rosbag...')
            cmd = 'rosbag record --all'
            if 'args' in data:
                cmd += ' ' + data['args']
            self.rosbag_process = subprocess.Popen(cmd, cwd=os.environ.get('HOME'), shell=True, preexec_fn=os.setsid)


class ReportingNamespace(SocketIONamespace):

    def on_connect(self):
        print('Connected to reporting')


if __name__ == '__main__':
    # Get server URI and port from environment variables
    server = os.environ.get('OARS_SERVER_URI', 'localhost')
    port = os.environ.get('OARS_SERVER_PORT', 1234)
    ssl = os.environ.get('OARS_SERVER_USE_SSL', False)

    tr = TelemetryReporter()

    # Register topic listeners
    tr.listen_to_topic('/boat/heading', Float32)
    tr.listen_to_topic('/boat/position', Pose2D)
    tr.listen_to_topic('/rudder_pos', Float32)
    tr.listen_to_topic('/weather/wind/rel', Pose2D)
    tr.listen_to_topic('/control/heading/error_desired_rudder_pos', Pose2D)
    tr.listen_to_topic('/control/mode', UInt8)
    tr.listen_to_topic('/planning/goal_pos', Pose2D)
    tr.listen_to_topic('/planning/waypoints', WaypointList)
    tr.listen_to_topic('/planning/waypoint_radius', UInt16)

    tr.connect(server, port, ssl)
