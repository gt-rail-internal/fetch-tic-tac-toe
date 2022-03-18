from operator import truediv
import numpy as np
import reward
import random
import forwardPass
import pickle

class GameEnv:
    def __init__(self):
        self.board_state = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.h_score = 0
        self.r_score = 0
        self.run_num = 0
        self.last_feedback = 0
        self.last_r_move = -1
        self.network = pickle.load(open('training/trained_agent_saved.pkl', 'rb'))
        # canvas declaration
        
    def reset(self):
        self.run_num += 1
        self.board_state = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        # self.l_score = 0.0
        # self.r_score = 0.0
        
        return self.board_state

    def next_board_state(self, action):
        # get the next board state, but don't actually transit to it!
        backup_states = [[x for x in self.board_state], self.h_score, self.r_score]
        next_board_state, _, _ = self.step(action)
        self.board_state, self.h_score, self.r_score = backup_states
        return next_board_state

    # select a random network
    def random_action(self):
        open_slots = [i for i in range(9) if self.board_state[i] == 0]
        action = random.sample(open_slots, 1)[0]
        return action

    # move the human randomly
    def move_human_randomly(self, state):
        # move the opponent's marker
        human_action = self.random_action()
        new_state = state.copy()
        new_state[human_action] = 1  # set to X
        #print("moved human")
        return new_state

    # choose the robot's best action
    def get_best_robot_move(self):
        best_value = -1e10
        best_action = -1
        # for each action, choose the best
        for action in range(9):
            # if the action isn't valid for the board state, ignore
            if not self.move_is_valid(action):
                continue
            
            feature_future = self.state_to_feature(self.next_board_state(action))
            # TODO: Insert code here: For each action, test with network and remember the best one
            X = np.array([feature_future]).T  # construct the input
            theoretical_value = forwardPass.forward_pass(self.network, X)  # get the reward value
            
            if theoretical_value > best_value:  # keep track of the best action
                best_value = theoretical_value
                best_action = action  

        # return the best action
        return best_action

    # moves a token on the board state
    def move_token(self, token=0, position=0):
        if self.board_state[position] != 0:
            print("error in move_token(): trying to move a token to a filled spot! Token", token, "Position", position, "Contains", self.board_state[position])
            return False
        if token not in [1, 2]:
            print("error in move_token(): invalid token,", token, "not in [1, 2]")
            return False
        self.board_state[position] = token
        return token

    def step(self, action):
        done = False
        
        new_state = [x for x in self.board_state]

        # move the robot's marker to their action
        if new_state[action] != 0:  # check that the action is 0, so safe to move there
            print("Error in game_env.step(), action", action, "not 0!")

        new_state[action] = 2  # set to O

        rew = reward.eval_reward(new_state)  # get the reward of the new state
        
        if rew == 1 or rew == -1:
            done = True

        # set the board state to the new state
        self.board_state = new_state

        #print("set new board state")
        
        return self.board_state, rew, done

    def get_current_game_state(self):
        return reward.eval_reward(self.board_state)

    @property
    def state(self):
        return self.board_state

    def state_to_feature(self, board_state):
        # TODO: insert code here (Optional) Modify feature space
        # return state / np.array([WIDTH, HEIGHT, WIDTH, HEIGHT, HEIGHT, HEIGHT, HEIGHT, HEIGHT])
        #return np.array((state[1] - state[-2]) / HEIGHT)  # y position of the ball
        return np.array(board_state)


    # prints out the game board
    def print_board(self):
        print(self.board_state)  # print empty space at the top
        # print each row
        for r in range(3):
            # print above     
            print("   |   |   ")
            # print row
            rp = ""
            # add each column
            for c in range(3):
                tile = self.board_state[r*3 + c]
                tile = " " if tile == 0 else "X" if tile == 1 else "O"
                rp += " " + tile + " "
                # add the right side border
                if c != 2:
                    rp += "|"
            print(rp)
            # add the bottom border
            if r != 2:
                print("---------------")
        print()  # print empty space at the end
        return


    # checks if a move is valid (empty spot)
    def move_is_valid(self, move):
        if self.board_state[move] == 0:
            return True
        return False

    # auto-feedback for common cases
    def get_auto_feedback(self):
        #print("auto review")
        good_reward = 10
        ok_reward = 3
        bad_reward = -10

        # case when first turn and robot does not take middle
        if len([x for x in self.board_state if x > 0]) == 2 and self.board_state.index(2) != 4:
            #print("Did not initially take middle, BAD")
            return bad_reward
        elif len([x for x in self.board_state if x > 0]) == 2 and self.board_state.index(2) == 4:
            #print("took middle, GOOD")
            return good_reward

        # case when two links and there is an open third
        # check across
        for i in range(0, len(self.board_state), 3):
            row = [self.board_state[i], self.board_state[i+1], self.board_state[i+2]]
            # if robot two X and one O and last move blocked, means robot blocked, rate good
            two_and_empty, idx = self.check_two_and_one(row, two=1, one=2)
            if two_and_empty and self.last_r_move == [i, i+1, i+2][idx]:
                #print("Blocked opponent from winning, GOOD")
                return good_reward
            # if opponent two X and one -, means robot did not fill spot, rate bad 
            two_and_empty, _ = self.check_two_and_one(row, two=1, one=0)
            if two_and_empty:
                #print("Did not fill opponent 2/3 slot row, BAD, row", row, "tande", two_and_empty)
                return bad_reward
            # if robot two X and one - and last move not the filled, means robot did not fill spot, rate bad 
            two_and_empty, idx = self.check_two_and_one(row, two=2, one=0)
            if two_and_empty and self.last_r_move not in [i, i+1, i+2]:
                #print("Did not fill personal 2/3 slot row, BAD, should have went", [i, i+1, i+2][idx], "instead went", self.last_r_move)
                return bad_reward

        # check down
        for i in range(3):
            col = [self.board_state[i], self.board_state[i+3], self.board_state[i+6]]
            # if robot two X and one O and last move blocked, means robot blocked, rate good
            two_and_empty, idx = self.check_two_and_one(col, two=1, one=2)
            if two_and_empty and self.last_r_move == [i, i+3, i+6][idx]:
                #print("Blocked opponent from winning, GOOD")
                return good_reward
            # if two X and one -, means robot did not fill spot, rate bad 
            two_and_empty, _ = self.check_two_and_one(col, two=1, one=0)
            if two_and_empty:
                #print("Did not fill opponent 2/3 slot col, BAD, col", col)
                return bad_reward
            # if robot two X and one - and last move not the filled, means robot did not fill spot to win, rate bad 
            two_and_empty, idx = self.check_two_and_one(col, two=2, one=0)
            if two_and_empty and self.last_r_move not in [i, i+3, i+6]:
                #print("Did not fill personal 2/3 slot col, BAD, should have went", [i, i+3, i+6][idx], "instead went", self.last_r_move)
                return bad_reward
        
        # check diag
        # if human two X and robot one O and robot just moved to block, rate good
        two_and_empty_diag_r, idx_r = self.check_two_and_one([self.board_state[0], self.board_state[4], self.board_state[8]], two=1, one=2)
        two_and_empty_diag_l, idx_l = self.check_two_and_one([self.board_state[2], self.board_state[4], self.board_state[6]], two=1, one=2)
        if (two_and_empty_diag_l and self.last_r_move == [self.board_state[0], self.board_state[4], self.board_state[8]][idx_r]) or (two_and_empty_diag_r and self.last_r_move == [self.board_state[2], self.board_state[4], self.board_state[6]][idx_l]):
            #print("Blocked off potential opponent win, GOOD")
            return bad_reward

        two_and_empty_diag_r, idx_r = self.check_two_and_one([self.board_state[0], self.board_state[4], self.board_state[8]], two=1, one=0)
        two_and_empty_diag_l, idx_l = self.check_two_and_one([self.board_state[2], self.board_state[4], self.board_state[6]], two=1, one=0)
        if two_and_empty_diag_r or two_and_empty_diag_l:
            #print("Did not fill opponent 2/3 slot diag, BAD")
            return bad_reward

        # if robot two O and one - and last move not the filled, means robot did not fill spot to win, rate bad 
        two_and_empty_diag_r, idx_r = self.check_two_and_one([self.board_state[0], self.board_state[4], self.board_state[8]], two=2, one=0)
        two_and_empty_diag_l, idx_l = self.check_two_and_one([self.board_state[2], self.board_state[4], self.board_state[6]], two=2, one=0)
        if (two_and_empty_diag_l and self.last_r_move not in [self.board_state[0], self.board_state[4], self.board_state[8]]) or (two_and_empty_diag_r and self.last_r_move not in [self.board_state[2], self.board_state[4], self.board_state[6]]):
            #print("Did not fill personal 2/3 slot row, BAD")
            return bad_reward   
        
        return ok_reward

    # bool check if two items are filled and one is empty
    def check_two_and_one(self, items, two=1, one=0):
        if len([x for x in items if x == two]) == 2 and len([x for x in items if x == one]) == 1:
            return True, items.index(one)
        return False, -1

    # get feedback from the human
    def get_human_feedback(self):
        #self.render(waiting=True)
        # get a user input
        while True:
            #move = input("Give X placement: ")
            feedback = input("Give feedback on the placement (y/n): ").lower()

            # end if the move is a valid placement
            #if self.move_is_valid(move):
            if feedback.lower() == "y":
                return 10
            if feedback.lower() == "n":
                return -10
            if feedback.lower() == "s":
                return 0
            if feedback.lower() == "":
                return 0.0001

            print("Invalid feedback! Try again.")
            print()
        return

    def set_feedback(self, feedback):
        self.last_feedback = feedback

    def check_done(self):
        if len([x for x in self.state if x == 0]) == 0:
            return True
        return False
    

if __name__ == "__main__":
    env = GameEnv()
    env.reset()
    for _ in range(1500):
        env.step(0)