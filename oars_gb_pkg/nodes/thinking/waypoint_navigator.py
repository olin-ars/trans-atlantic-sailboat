import math
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D

class WaypointNavigator:

    def __init__(self, radius):
        self.longitude = None
        self.latitude = None
        self.radius = radius
        self.wp_list = []
        self.next_wp = None
        # rospy.init_node('waypoint_navigator',anonymous=True)
        # self.heading_pub = rospy.Publisher('/control/desired_heading', Float32, queue_size=0)
        # rospy.Subscriber('/boat/position', Pose2D, self.update_location, queue_size=1)

    def update_location(self, msg, do_publish=True):
        self.longitude = msg.x
        self.latitude = msg.y
        # tell if GPS location is within waypoint radius
        self.check_distance_to_wp()
        if do_publish == True:
            self.publish_desired_heading()

    def check_distance_to_wp(self):
        #self.next_wp = self.wp_list[0]
        if self.next_wp == None:
            return False
        dist_to_wp = math.sqrt(((self.next_wp[0] - self.longitude)**2) + ((self.next_wp[1] - self.latitude)**2))
        if dist_to_wp <= self.radius:
            self.wp_list = self.wp_list[1:]
            self.update_wp_list(self.wp_list)
            return True

    def update_wp_list(self, wp_list, do_publish=True):
        self.wp_list = wp_list
        self.next_wp = wp_list[0]
        if do_publish == True:
            self.publish_desired_heading()

    def calculate_desired_heading(self):
        if self.longitude == None:
            return
        delta_x = self.next_wp[0] - self.longitude
        delta_y = self.next_wp[1] - self.latitude
        return math.degrees(math.atan2(delta_y, delta_x))

    def publish_desired_heading(self):
        theta = self.calculate_desired_heading()
        #self.heading_pub.publish(theta)


if __name__ == '__main__':
    WaypointNavigator()
