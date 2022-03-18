from math import pi

import numpy as np

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from motion import *

#TODO: Hover joints for 8, 7, 5, 4 need to be updated

pickup = [0.3568596839904785, -0.799571570583496, -0.8731649180769532, 0.10935412892847061, 1.760337371472168, -0.07973667504444122, 0.3976447473876953, 0.06521148284650803]

home = [0.35653162002563477, -0.8847075073205566, -0.8026018401503174, 0.10935412892847061, 0.977240104321289, -0.07973667504444122, 1.0956060778015138, 0.06597810288644791]

hover_joints = {
    '0': [0.3523354232311249, 0.10701078491577148, -0.4651260634779541, 0.2351405600622177, 0.19644405329589842, -0.14493086459293364, 1.4411351572387696, 0.06521148284650803],
    '1': [0.35231253504753113, -0.06709581298461914, -0.6403832694410889, 0.23475706377773284, 0.6309437938964844, -0.14569784971132277, 1.10634397376709, 0.06444411774850846],
    '2': [0.35509729385375977, -0.1579843131982422, -0.6522717734694092, 0.10973761776237488, 0.6623907276428223, -0.07935318248524666, 1.1305043588989259, -0.12078406611703872],
    '3': [0.35232779383659363, 0.07786554413208008, -0.7696211596846192, 0.23552405634670256, 0.7935457416809082, -0.14569784971132277, 1.2620432268493653, 0.06444411774850846],
    '4': [0.355410099029541, -0.03181415481201172, -0.8371165057539551, 0.10935412892847061, 0.9373564907348633, -0.07973667504444122, 1.1492955576293946, -0.12078406611703872],
    '5': [0.355532169342041, -0.17409091872802734, -0.7861116191267579, 0.10935412892847061, 0.936973113659668, -0.07973667504444122, 1.150829542767334, -0.11925008842967987],
    '6': [0.35582971572875977, 0.14612764435180664, -0.9556165000319092, 0.11012111404685974, 1.2510557361877441, -0.07973667504444122, 1.082183588635254, 0.0640606214640236],
    '7': [0.35624170303344727, -0.03910070342651367, -1.0534076949476807, 0.10973761776237488, 1.3749246784484863, -0.07973667504444122, 0.9843923937194824, 0.06482798656202317],
    '8': [0.3564934730529785, -0.2158918945275879, -1.0564756652235596, 0.10935412892847061, 1.3430948444641113, -0.07973667504444122, 1.0288779627197266, 0.06559460660196305],
}

goal_joints = {

    '0': [0.34677356481552124, 0.18025869446166992, -0.24461629594560547, 0.11702403226642609, 0.3525262066162109, -0.21741145374431609, 1.0373146425598145, 0.07479886015630723],
    '1': [0.3550591468811035, -0.013790187069091797, -0.38037350381608886, 0.10935412892847061, 0.6620068737304687, -0.08012017132892608, 0.8919703852050781, -0.08741998622440338],
    '2': [0.35511255264282227, -0.15299898070922852, -0.3673348685621826, 0.10858713635950089, 0.6623907276428223, -0.07935318248524666, 0.847484816204834, -0.08741998622440338],
    '3': [0.35509729385375977, 0.1219672592199707, -0.5402910968184083, 0.10973761776237488, 0.9362058826721191, -0.08012017132892608, 0.9291694055908203, -0.120017073548069],
    '4': [0.35518884658813477, -0.031047400661621094, -0.5391407271742432, 0.10897063264398575, 0.9365897365844726, -0.0789696862007618, 0.9544799219482422, -0.120017073548069],
    '5': [0.355471134185791, -0.17409091872802734, -0.5257182380079835, 0.10897063264398575, 0.9365897365844726, -0.0789696862007618, 0.922649611126709, -0.12078406611703872],
    '6': [0.35553979873657227, 0.1188992889440918, -0.8332815429091065, 0.10935412892847061, 1.5079975315368652, -0.0789696862007618, 0.6542031656616211, 0.06559460660196305],
    '7': [0.356081485748291, -0.04025131148925781, -0.7960825225233643, 0.11088810661582947, 1.4128904529846191, -0.07973667504444122, 0.7934119593017578, 0.06444411774850846],
    '8': [0.3564629554748535, -0.22394519729248047, -0.8183250686049073, 0.10973761776237488, 1.4730997272766113, -0.08050366761341095, 0.7178633104675293, 0.06444411774850846],
}


def pick_and_place(x, y):

    move_gripper(1)     # Gripper: open

    go_to_joint(pickup) # Go to X

    # Gripper: closed (picking up X)
    print('Picking up X')
    move_gripper(0)

    go_to_joint(home)   # Back to home position

    go_to_joint(x)      # Hover above goal position

    go_to_joint(y)      # Go to goal position

    # Gripper: open (drop X)
    print('Releasing X')
    move_gripper(1)

    go_to_joint(x)      # Hover above goal position

    go_to_joint(home)   # Take robot to home position 

    return

def callback(msg):
    tile = msg.data

    if tile in hover_joints.keys():
        print('Going to {}'.format(tile))
        pick_and_place(hover_joints[tile], goal_joints[tile])

    else:
        raise KeyError

def main():
    rospy.init_node('run_game')

    print('Node initialized, going home...')
    go_to_joint(home)   # Take robot to home position 


    rospy.Subscriber('game_action', String, callback, queue_size=1)
    rospy.spin()

    # tile = str(0)
    # pick_and_place(hover_joints[tile], goal_joints[tile])

if __name__ == '__main__':
    main()
