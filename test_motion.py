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

def main():
    rospy.init_node('test_motion')

    joint_1 = [0.38392895460128784, 0.07403034286865234, -0.5414417048811524, 0.19870850244312285, 1.0232591816223144, -0.2703337919916153, 1.093688715588379, 0.0069198371259069445]

    joint_2 = [0.38385266065597534, 0.1434429319418335, -0.13225224221940918, 0.19947549501209258, 1.0244102665222168, -0.2703337919916153, 0.5211303125732422, 0.006536717095742226]

    move_arm_joints(joint_1)

    # raw_input('Next?')

    move_arm_joints(joint_2)

    joints = return_joints()
    print(joints)

if __name__ == '__main__':
        main()