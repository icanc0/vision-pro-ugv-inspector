#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge
from PIL import Image
from sensor_msgs.msg import Image as ROSImage

bridge = CvBridge()

def callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    Image.fromarray(cv_image).show()
    
def receiver():
    rospy.init_node('img_receiver', anonymous=True)
    rospy.Subscriber("img_topic", ROSImage, callback)
    rospy.spin()

if __name__ == '__main__':
    receiver()