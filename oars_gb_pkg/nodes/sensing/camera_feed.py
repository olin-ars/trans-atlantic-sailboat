#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import cv2

# creating ros node for the camera data
pub_camera = rospy.Publisher('/sensors/camera1', Image, queue_size=10)

def talker():
    rospy.init_node("camera_video_feed", anonymous=True) # Initialize nodes
    rate = rospy.Rate(10) #10 Hz
    # capturing video from camera, storing in frame
    cap = cv2.VideoCapture(0)

    while not rospy.is_shutdown():
        # capture frame by frame
        ret, frame = cap.read()

        # operations on the frame
        # luv = cv2.cvtColor(frame, cv2.RGB2RGBA)
        '''
        # display the resulting frame
        cv2.imshow('frame', luv)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        '''
        image_publisher(frame)

        rate.sleep() #controls the speed at which the while loop runs

# to display the camera feed in terminal (not through ros), use this code
'''
# when this is done, release the capture
cap.release()
cv2.destroyAllWindows()
'''

def image_publisher(frame):
    # create list that stores the image data
    ros_img_list = list()

    # defining the image object
    
    header = Header()
    height = frame.shape[0]
    width = frame.shape[1]
    encoding = 'bgr8'
    step = 3 * width

    # turn the opencv data into ROS-readable data
    # selecting the row
    for row in frame:
        # selecting the column - we'll call it a pixel
        for pixel in row:
            # selecting the rgb values - we'll call them components, or comp for short
            for comp in pixel:
                # add these values to the list
                ros_img_list.append(comp)

    #publishing the data to the image object

    ros_img = Image(header=header, height=height, width=width, encoding=encoding,
                    is_bigendian=False, step=step, data=ros_img_list)
    pub_camera.publish(ros_img)

if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass