from math import pi

import numpy as np

from sensor_msgs.msg import JointState
from motion import *

'''
pickup- [0.3707147538661957, -0.6427222458802795, -0.2488348742842285, 0.19985899129657744, 0.3433222957885742, -0.24732408942832945, 1.2769992243164063, 0.18256101062274932]
stall- [0.37060031294822693, -0.6419552533113098, -1.0280969401716797, 0.20062598386554717, 1.016356486920166, -0.13457649471416472, 1.4967419992797852, 0.18141053667045592]
0, 0 overhead - [0.3704706132411957, -0.19364934844604492, -0.9889805575728028, 0.2002424875810623, 1.456992644909668, -0.13457649471416472, 1.099057425152588, 0.18256101062274932]
0, 0 place- [0.36982211470603943, -0.20208626670471191, -0.9893639346479981, 0.20062598386554717, 1.722371120098877, -0.13534347983255385, 0.7623488794677734, 0.18102704038597106]
2, 1 overhead - [0.36181119084358215, 0.09205454903015137, -0.27874734605546875, 0.19372306564598082, -0.03250263249511719, -0.14071241291418074, 1.779761541973877, 0.18831343998886108]
2,1 place - [0.3617730438709259, 0.09128755646118164, -0.08469870294328613, 0.1929560879781723, -0.033269863482666015, -0.1410959091986656, 1.6351840387695313, 0.18831343998886108]
'''

pickup = [0.3568596839904785, -0.799571570583496, -0.8731649180769532, 0.10935412892847061, 1.760337371472168, -0.07973667504444122, 0.3976447473876953, 0.06521148284650803]

home = [0.35653162002563477, -0.8847075073205566, -0.8026018401503174, 0.10935412892847061, 0.977240104321289, -0.07973667504444122, 1.0956060778015138, 0.06597810288644791]

hover_joints = {
    '0': [0.3548531234264374, 0.11928266601928711, -0.4421162864088623, 0.10935412892847061, 0.30382253611450194, -0.08012017132892608, 1.3483292948120118, -0.0881869713427925],
    '1': [0.3550591468811035, -0.013406333156738282, -0.5786407252668946, 0.10935412892847061, 0.6623907276428223, -0.07973667504444122, 1.0361645113342286, -0.0881869713427925],
    '2': [0.35509729385375977, -0.1579843131982422, -0.6522717734694092, 0.10973761776237488, 0.6623907276428223, -0.07935318248524666, 1.1305043588989259, -0.12078406611703872],
    '3': [0.355135440826416, 0.13232177811035156, -0.8589756747603028, 0.11050461033134461, 0.9350557514465332, -0.0789696862007618, 1.1324217211120606, -0.120017073548069],
    '4': [0.355410099029541, -0.03181415481201172, -0.8371165057539551, 0.10935412892847061, 0.9373564907348633, -0.07973667504444122, 1.1492955576293946, -0.12078406611703872],
    '5': [0.355532169342041, -0.17409091872802734, -0.7861116191267579, 0.10935412892847061, 0.936973113659668, -0.07973667504444122, 1.150829542767334, -0.11925008842967987],
    '6': [0.35582971572875977, 0.14612764435180664, -0.9556165000319092, 0.11012111404685974, 1.2510557361877441, -0.07973667504444122, 1.082183588635254, 0.0640606214640236],
    '7': [0.35624170303344727, -0.03910070342651367, -1.0534076949476807, 0.10973761776237488, 1.3749246784484863, -0.07973667504444122, 0.9843923937194824, 0.06482798656202317],
    '8': [0.3564934730529785, -0.2158918945275879, -1.0564756652235596, 0.10935412892847061, 1.3430948444641113, -0.07973667504444122, 1.0288779627197266, 0.06559460660196305],
}

goal_joints = {
    '0': [0.3548378646373749, 0.13385576324829102, -0.20434930528398437, 0.10935412892847061, 0.3030553051269531, -0.07973667504444122, 1.1209165941589356, -0.0881869713427925],
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

    go_to_joint(home)   # Take robot to home position 

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

    return

def main():
    rospy.init_node('test_motion')

    tile = 0

    pick_and_place(hover_joints[tile], goal_joints[tile])

if __name__ == '__main__':
        main()