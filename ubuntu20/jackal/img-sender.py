#!/usr/bin/env python
import numpy as np
from cv_bridge import CvBridge
from PIL import Image
import rospy
from sensor_msgs.msg import Image as ROSImage
import requests

bridge = CvBridge()

#!/usr/bin/env python
def sender():
    rospy.init_node('img_sender', anonymous=True)
    pub = rospy.Publisher('img_topic', ROSImage, queue_size=10)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        url = "https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcQAJ8gZmML3-V-ztaF8bYk1fkF16MXiSfHhMQ&s"
        img = np.asarray(Image.open(requests.get(url, stream=True).raw))
        image_message = bridge.cv2_to_imgmsg(img, encoding="passthrough")
        
        pub.publish(image_message)
        rate.sleep()

if __name__ == '__main__':
    try:
        sender()
    except rospy.ROSInterruptException:
        pass
