import math
# import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D

class WaypointNavigator:

    def __init__(self):
        self.longitude = None
        self.latitude = None
        # rospy.init_node('waypoint_navigator',anonymous=True)
        # self.heading_pub = rospy.Publisher('/control/desired_heading', Float32, queue_size=0)
        # rospy.Subscriber('/boat/position', Pose2D, self.update_location, queue_size=1)

    def update_location(self, msg, do_publish=True):
        self.longitude = msg.x
        self.latitude = msg.y
        if do_publish == True:
            publish_desired_heading()

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
        theta = calculate_desired_heading()
        self.heading_pub.publish(theta)


if __name__ == '__main__':
    WaypointNavigator()
