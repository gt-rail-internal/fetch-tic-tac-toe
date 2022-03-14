import rospy
from sensor_msgs.msg import JointState

JIDXS = [2, 6, 7, 8, 9, 10, 11, 12]


def return_joints():
    msg = rospy.wait_for_message('joint_states', JointState)

    while len(msg.position) < 13:
        msg = rospy.wait_for_message('joint_states', JointState)

    joints = [msg.position[i] for i in JIDXS]

    return joints

if __name__ == '__main__':
        rospy.init_node('collect_joint_state')

        print(return_joints())