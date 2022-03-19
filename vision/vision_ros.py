import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from vision import process_image

cvBridge = CvBridge()

def callback(msg):

    # Convert rosmsg to opencv image
    try:
        cv_image = cvBridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    # Call process_image function
    process_image(cv_image)

if __name__ == '__main__':
    # Initialize ros node
    rospy.init_node('vision_ros')

    # Image subscriber and spin
    rospy.Subscriber('/head_camera/rgb/image_raw', Image, callback, queue_size=1)
    rospy.spin()