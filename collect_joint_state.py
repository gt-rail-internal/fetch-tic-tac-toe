import argparse

import rospy
from sensor_msgs.msg import JointState

JIDXS = [2, 6, 7, 8, 9, 10, 11, 12]

def write_to_file(data, line, filename):
    with open(filename, 'a') as file:
        file.write(data+'\n')

def return_joints(tile, type):
    msg = rospy.wait_for_message('joint_states', JointState)

    while len(msg.position) < 13:
        msg = rospy.wait_for_message('joint_states', JointState)

    joints = [msg.position[i] for i in JIDXS]

    data = '{}: {},'.format(tile, joints)

    write_to_file(data, tile, '{}_joints.txt'.format(type))

    return 

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='')
    parser.add_argument('-tile', type=int, help='Tile Integer')
    parser.add_argument('-type', type=str, help='Type of position')

    args = parser.parse_args()

    rospy.init_node('collect_joint_state')

    return_joints(args.tile, args.type)
