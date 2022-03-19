import rospy
from std_msgs.msg import String, Int32MultiArray
import training.game_env

env = training.game_env.GameEnv()
GAME_INFO_PUBLISHER = None  # publishes to the /game_info topic
GAME_ACTION_PUBLISHER = None  # publishes to the /game_action topic

is_moving = False
game_state = [0, 0, 0, 0, 0, 0, 0, 0]

# sends info to the /game_info topic
def send_info(info):
    print(info)  # print the info to the terminal
    if GAME_INFO_PUBLISHER is not None:  # if possible, publish the info
        GAME_INFO_PUBLISHER.publish(info)
    else:
        print("send_info(): could not send info, publisher not initialized.")


# sends action to the /game_info topic
def send_action(action):
    if action == -1:
        print("game is won, ignoring")
        return

    global is_moving
    print(action)  # print the info to the terminal
    if GAME_ACTION_PUBLISHER is not None:  # if possible, publish the info
        GAME_ACTION_PUBLISHER.publish(action)
        is_moving = True
    else:
        print("send_action(): could not send action, publisher not initialized.")


# read the game state int32[] and process what that means
def read_state(data):
    state = list(data.data)  # get states
    print("Received State", state)

    # check if states has correct dimensions
    if len(state) != 9:
        send_info('game_state malformed, has ' + str(len(state)) + ' states, needs 9.')
        return
    
    # check if values of all states are valid
    if len([x for x in state if x not in [0, 1, 2]]) != 0:
        send_info('game_state malformed, has items that are not 0, 1, or 2: ' + str(state))
        return

    global game_state
    game_state = state

def read_info(data):
    info = str(data.data)
    if info == "done moving":
        global is_moving
        is_moving = False
        print("Now able to move game logic")

# choose the next move given a game state
def select_move(state):
    # set the board state
    env.board_state = state
    move = env.get_best_robot_move()
    print("Robot chooses to go", move)
    return move


# sets up the node and waits for program to exit
def state_reader():
    rospy.init_node('game_logic', anonymous=True)  # initialize node 

    global GAME_INFO_PUBLISHER  # set up the publisher to /game_info
    GAME_INFO_PUBLISHER = rospy.Publisher('game_info', String, queue_size=1)

    global GAME_ACTION_PUBLISHER  # set up the publisher to /game_action
    GAME_ACTION_PUBLISHER = rospy.Publisher('game_action', String, queue_size=1)

    rospy.Subscriber('game_state', Int32MultiArray, read_state)  # set up the subscriber to /game_state
    rospy.Subscriber('game_info', String, read_info)  # set up the subscriber to /game_info
    
    while True:
        raw_input("Press ENTER for Robot's Turn")

        # determine turn
        num_blank = len([x for x in game_state if x == 0])
        num_x = len([x for x in game_state if x == 1])
        num_o = len([x for x in game_state if x == 2])

        # if x <= o, human's turn (human always goes first), return
        if num_x <= num_o:
            send_info('human turn')
            continue

        # check if currently moving
        if is_moving:
            print("Cannot run, currently is_moving")
            continue
            
        # it's the robot's turn -- decide where to go
        move = select_move(game_state)
        send_action(str(move))  # send the action to the /game_action topic
    
    rospy.spin()  # wait for program to exit


if __name__ == "__main__":
    state_reader()