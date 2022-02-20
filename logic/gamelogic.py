import rospy
from std_msgs.msg import String, Int32MultiArray
import random


GAME_INFO_PUBLISHER = None  # publishes to the /game_info topic

# sends info to the /game_info topic
def send_info(info):
    print(info)  # print the info to the terminal
    if GAME_INFO_PUBLISHER is not None:  # if possible, publish the info
        GAME_INFO_PUBLISHER.publish(info)


# read the game state int32[] and process what that means
def read_state(data):
    state = data.data  # get states

    # check if states has correct dimensions
    if len(state) != 9:
        send_info('game_state malformed, has ' + str(len(state)) + ' states, needs 9.')
        return
    
    # check if values of all states are valid
    if len([x for x in state if x not in [0, 1, 2]]) != 0:
        send_info('game_state malformed, has items that are not 0, 1, or 2: ' + str(state))
        return

    # determine turn
    num_blank = len([x for x in state if x == 0])
    num_x = len([x for x in state if x == 1])
    num_o = len([x for x in state if x == 2])

    # if x <= o, human's turn (human always goes first), return
    if num_x <= num_o:
        send_info('human turn')
        return
         
    # it's the robot's turn -- decide where to go
    move = select_move(state)


# choose the next move given a game state
def select_move(state):
    # for now, choose random move
    open_slots = [i for i in range(len(state)) if state[i] == 0]
    move = random.sample(open_slots)
    print(move)

print(select_move([0, 0, 0, 1, 2, 3]))


# sets up the node and waits for program to exit
def state_reader():
    rospy.init_node('listener', anonymous=True)  # initialize node 

    global GAME_INFO_PUBLISHER  # set up the publisher to /game_info
    GAME_INFO_PUBLISHER = rospy.Publisher('game_info', String)

    subscriber = rospy.Subscriber('game_state', Int32MultiArray, read_state)  # set up the subscriber to /game_state
    rospy.spin()  # wait for program to exit


if __name__ == "__main__":
    state_reader()