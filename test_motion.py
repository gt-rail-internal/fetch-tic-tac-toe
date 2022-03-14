from math import pi

import numpy as np

from sensor_msgs.msg import JointState
from motion import *

JIDXS = [2, 6, 7, 8, 9, 10, 11, 12]

def init_arm():
    #joints = [2.9803242683410645, -1.5093436805688476, -0.4041502734541504, -1.129718886820984,-1.4648575595581055, 1.9692782062088012, 2.017528761517334, 0.6389203225610351]
    
    # init forward right near 0,1
    joints = [2.9, -.6, -pi/4, 0, 4*pi/8, 0, pi/4, 0]
    
    move_arm_joints(joints)
    print("Arm location initialized")

def return_joints():
    msg = rospy.wait_for_message('joint_states', JointState)

    joints = [msg.position[i] for i in JIDXS]

    while len(joints) < 8:
        msg = rospy.wait_for_message('joint_states', JointState)

        joints = [msg.position[i] for i in JIDXS]

    return joints

def main():
    rospy.init_node('test_motion')

    init_arm()

    # joint_1 = [0.3835856318473816, 0.184, 0.13475960493087769, 0.28363562943725584, 0.364336195758667, -0.9540825148939698, -0.005694436042022705, 1.6165266224182129]

    # joint_2 = [0.3831583857536316, 0.184, 0.13456189632415771, 0.28325201394348143, 0.1330885322607422, -0.2722280284285156, -0.005694436042022705, 0.4610552974975586]

    # move_arm_joints(joint_1)

    # raw_input('Next?')

    # move_arm_joints(joint_2)

    joints = return_joints()
    print(joints)

if __name__ == '__main__':
        main()