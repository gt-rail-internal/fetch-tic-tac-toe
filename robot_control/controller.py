import rospy
from std_msgs.msg import Int32MultiArray, String

GAME_STATE_PUBLISHER = None  # publishes to the /game_state topic

# read the game_info
def read_info(data):
    info = data.data
    print("Game Info", info)
    return


# read the action
def read_action(data):
    action = data.data
    print("Game Action", action)
    return


# sets up the node and waits for program to exit
def ros_reader():
    rospy.init_node('test_ros', anonymous=True)  # initialize node 

    global GAME_STATE_PUBLISHER  # set up the publisher to /game_info
    GAME_STATE_PUBLISHER = rospy.Publisher('game_state', Int32MultiArray, queue_size=1)

    rospy.Subscriber('game_info', String, read_info)  # set up the subscriber to /game_info
    rospy.Subscriber('game_action', String, read_action)  # set up the subscriber to /game_action
    
    input()
    GAME_STATE_PUBLISHER.publish(data=[0, 0, 1, 0, 2, 0, 0, 0, 1])
    print("sent state")

    rospy.spin()  # wait for program to exit


if __name__ == "__main__":
    ros_reader()