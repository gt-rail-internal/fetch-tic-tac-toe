import rospy

from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge, CvBridgeError

# from vision import process_image

cvBridge = CvBridge()

# ADD PUBLISHER TO game_state
marker_pub = rospy.Publisher('game_state', Int32MultiArray, queue_size=1)

def callback(msg):

    # Convert rosmsg to opencv image
    try:
        cv_image = cvBridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
        return

    # Call process_image function
    markers = [0,0,0,1,1,1,2,2,2] #process_image(cv_image)

    # ROS message for markers
    msg = Int32MultiArray()
    msg.data = markers

    # PUBLISH MARKERS to game_state
    marker_pub.publish(msg)

if __name__ == '__main__':
    # Initialize ros node
    rospy.init_node('vision_ros')

    # Image subscriber and spin
    rospy.Subscriber('/head_camera/rgb/image_raw', Image, callback, queue_size=1)
    rospy.spin()