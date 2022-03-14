from math import pi

import numpy as np

from sensor_msgs.msg import JointState
from motion import *

def init_arm():
    #joints = [2.9803242683410645, -1.5093436805688476, -0.4041502734541504, -1.129718886820984,-1.4648575595581055, 1.9692782062088012, 2.017528761517334, 0.6389203225610351]
    
    # init forward right near 0,1
    joints = [2.9, -.6, -pi/4, 0, 4*pi/8, 0, pi/4, 0]
    
    move_arm_joints(joints)
    print("Arm location initialized")

def return_joints():
    msg = rospy.wait_for_message('joint_states', JointState)

    joints = msg.position[2:10]

    while len(joints) < 8:
        msg = rospy.wait_for_message('joint_states', JointState)

        joints = msg.position[2:10]

    return joints

def main():
    rospy.init_node('test_motion')

    # init_arm()

    # pos, ort = get_pose(link="wrist_roll_link")
    # print('Position-> ', pos)
    # print('Orientation [deg] --> ', [np.degrees(i) for i in ort])

    joints = return_joints()
    print(joints)

if __name__ == '__main__':
        main()