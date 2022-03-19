from math import pi

import numpy as np

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from motion import *

#TODO: Hover joints for 8, 7, 5, 4 need to be updated

'''
# Old

pickup = [0.3568596839904785, -0.799571570583496, -0.8731649180769532, 0.10935412892847061, 1.760337371472168, -0.07973667504444122, 0.3976447473876953, 0.06521148284650803]

home = [0.35653162002563477, -0.8847075073205566, -0.8026018401503174, 0.10935412892847061, 0.977240104321289, -0.07973667504444122, 1.0956060778015138, 0.06597810288644791]

# Old hover joints    
    '0': [0.3523354232311249, 0.10701078491577148, -0.4651260634779541, 0.2351405600622177, 0.19644405329589842, -0.14493086459293364, 1.4411351572387696, 0.06521148284650803],
    '1': [0.35231253504753113, -0.06709581298461914, -0.6403832694410889, 0.23475706377773284, 0.6309437938964844, -0.14569784971132277, 1.10634397376709, 0.06444411774850846],
    '2': [0.35509729385375977, -0.1579843131982422, -0.6522717734694092, 0.10973761776237488, 0.6623907276428223, -0.07935318248524666, 1.1305043588989259, -0.12078406611703872],
    '3': [0.35232779383659363, 0.07786554413208008, -0.7696211596846192, 0.23552405634670256, 0.7935457416809082, -0.14569784971132277, 1.2620432268493653, 0.06444411774850846],
    '4': [0.355410099029541, -0.03181415481201172, -0.8371165057539551, 0.10935412892847061, 0.9373564907348633, -0.07973667504444122, 1.1492955576293946, -0.12078406611703872],
    '5': [0.355532169342041, -0.17409091872802734, -0.7861116191267579, 0.10935412892847061, 0.936973113659668, -0.07973667504444122, 1.150829542767334, -0.11925008842967987],
    '6': [0.35582971572875977, 0.14612764435180664, -0.9556165000319092, 0.11012111404685974, 1.2510557361877441, -0.07973667504444122, 1.082183588635254, 0.0640606214640236],
    '7': [0.35624170303344727, -0.03910070342651367, -1.0534076949476807, 0.10973761776237488, 1.3749246784484863, -0.07973667504444122, 0.9843923937194824, 0.06482798656202317],
    '8': [0.3564934730529785, -0.2158918945275879, -1.0564756652235596, 0.10935412892847061, 1.3430948444641113, -0.07973667504444122, 1.0288779627197266, 0.06559460660196305],

# Old goal joints

    '0': [0.34677356481552124, 0.18025869446166992, -0.24461629594560547, 0.11702403226642609, 0.3525262066162109, -0.21741145374431609, 1.0373146425598145, 0.07479886015630723],
    '1': [0.3550591468811035, -0.013790187069091797, -0.38037350381608886, 0.10935412892847061, 0.6620068737304687, -0.08012017132892608, 0.8919703852050781, -0.08741998622440338],
    '2': [0.35511255264282227, -0.15299898070922852, -0.3673348685621826, 0.10858713635950089, 0.6623907276428223, -0.07935318248524666, 0.847484816204834, -0.08741998622440338],
    '3': [0.35509729385375977, 0.1219672592199707, -0.5402910968184083, 0.10973761776237488, 0.9362058826721191, -0.08012017132892608, 0.9291694055908203, -0.120017073548069],
    '4': [0.35518884658813477, -0.031047400661621094, -0.5391407271742432, 0.10897063264398575, 0.9365897365844726, -0.0789696862007618, 0.9544799219482422, -0.120017073548069],
    '5': [0.355471134185791, -0.17409091872802734, -0.5257182380079835, 0.10897063264398575, 0.9365897365844726, -0.0789696862007618, 0.922649611126709, -0.12078406611703872],
    '6': [0.35553979873657227, 0.1188992889440918, -0.8332815429091065, 0.10935412892847061, 1.5079975315368652, -0.0789696862007618, 0.6542031656616211, 0.06559460660196305],
    '7': [0.356081485748291, -0.04025131148925781, -0.7960825225233643, 0.11088810661582947, 1.4128904529846191, -0.07973667504444122, 0.7934119593017578, 0.06444411774850846],
    '8': [0.3564629554748535, -0.22394519729248047, -0.8183250686049073, 0.10973761776237488, 1.4730997272766113, -0.08050366761341095, 0.7178633104675293, 0.06444411774850846],


'''

home = [0.3544258773326874, -0.9288096992456054, -0.7155485412001221, 0.10513567724971772, 0.7713031955993652, -0.02489686166114509, 1.2478535066955567, 0.1131483842966652]
pickup =  [0.3537850081920624, -0.6611300079309081, -0.783043648850879, 0.103218203277874, 1.64222051585083, -0.025280356781476734, 0.27262519706420896, 0.1131483842966652]


hover_joints = {
    '0': [0.35344168543815613, 0.10240894871124268, -0.5855436583876221, 0.10436868468074799, 0.383205909375, -0.025280356781476734, 1.2716305147521973, 0.11391499688602448],
    '1': [0.35342642664909363, -0.03679984492889404, -0.5855436583876221, 0.10245121070890427, 0.383205909375, -0.026430842375302313, 1.2727811228149415, 0.11391499688602448],
    '2': [0.3530068099498749, -0.10237747115722656, -0.5176649352430909, 0.10360169956235886, 0.27851201975708006, -0.14799881996765135, 1.367504347454834, 0.11391499688602448],
    '3': [0.3446907103061676, 0.09512251930603027, -0.9145825168013184, 0.11318907687215805, 1.0351476856506348, -0.02911530891611576, 1.086402166973877, 0.1104635452268219],
    '4': [0.3447059690952301, -0.04446977061859131, -0.9763250609755127, 0.11357257315664292, 1.0347643085754394, -0.06593084978117943, 1.1151645075195313, 0.10931305637336731],
    '5': [0.3444160521030426, -0.18099397105804443, -0.8087377807020752, 0.11357257315664292, 0.7647838779724121, -0.06554735349669456, 1.3000090013854981, 0.1104635452268219],
    '6': [0.34512558579444885, 0.1330885322607422, -1.1841795703291504, 0.10782014379053116, 1.4849877544677734, -0.09392600031747818, 0.8689606081359863, 0.12350276162601471],
    '7': [0.3451332151889801, -0.05060571117034912, -1.1837961932539551, 0.10666966238765717, 1.4849877544677734, -0.09277551146402359, 0.873945940625, 0.12081828763462067],
    '8': [0.3451332151889801, -0.19364934844604492, -1.1837961932539551, 0.10666966238765717, 1.48460390055542, -0.09354250403299331, 0.8724119554870605, 0.12196877648807526],
}

goal_joints = {
    '0': [0.35365530848503113, 0.10164195614227295, -0.22927644456621094, 0.10360169956235886, 0.3828225322998047, -0.0268143377284646, 1.0327126871459962, 0.11429887315011979],
    '1': [0.35302969813346863, 0.033379736713256836, -0.22735908235307617, 0.10360169956235886, 0.3728513904846191, -0.14723182739868163, 1.067227591168213, 0.11429887315011979],
    '2': [0.35275503993034363, -0.11158138198486328, -0.3178639670729248, 0.10360169956235886, 0.5596137234008789, -0.14876581253662108, 0.9356887232177734, 0.11429887315011979],
    '3': [0.3446907103061676, 0.10240894871124268, -0.557931925904712, 0.11395606944112778, 1.0343809315002441, -0.0651638572122097, 0.8275430094116211, 0.10969692518688202],
    '4': [0.3446449339389801, -0.045236763187561035, -0.5993492862104981, 0.11280558058767319, 1.1283369251525879, -0.06593084978117943, 0.7565963159912109, 0.11084704151130677],
    '5': [0.3439735472202301, -0.16795521659484863, -0.6591747065901368, 0.11242208430318833, 1.305895347241211, -0.17829495193614958, 0.4992711435668945, 0.15609984149909972],
    '6': [0.3446907103061676, 0.1668360860861206, -0.8639610072493165, 0.10782014379053116, 1.721220988873291, -0.2657318514789581, 0.3543097864501953, 0.12235227277256013],
    '7': [0.34440842270851135, -0.03756683749786377, -0.8827524443983643, 0.1074366475060463, 1.743463534954834, -0.17407649280681609, 0.3489410768859863, 0.12235227277256013],
    '8': [0.3443855345249176, -0.1947998372994995, -0.8819854518293946, 0.1074366475060463, 1.8055896946228027, -0.2404211712087631, 0.23044084418945313, 0.12158490767456055],

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


    # rospy.Subscriber('game_action', String, callback, queue_size=1)
    # rospy.spin()

    tile = str(3)
    pick_and_place(hover_joints[tile], goal_joints[tile])

if __name__ == '__main__':
    main()
