import pickle

from numpy.core.fromnumeric import argmax
import game_env
import sys
from createNeuralNet import create_neural_net
from forwardPass import forward_pass  # Hint: You'll need these two!
from backprop import backprop
import numpy as np
import reward
import copy
from os.path import exists


if __name__ == "__main__":
    FEEDBACK = 0.0
    save = True
    env = GameEnv()

    # network hyperparameters
    # TODO: insert code here (Optional) Modify hyperparameters
    f_t_1 = None
    f_t_2 = None

    hidden_dims = [360]
    input_dim = 9
    output_dim = 1
    alpha = 1e-3

    if exists("trained_agent.pkl"):
        nn = pickle.load(open("trained_agent.pkl", "rb"))
    else:
        nn = create_neural_net(numNodesPerLayer=hidden_dims, numInputDims=input_dim, numOutputDims=output_dim)

    last_1000_games = [0.5] * 1000

    count = 0  # training count
    old_nn = None

    while True:
        # update every 10000 iterations
        if count % 1000 == 0:
            catless_games = [x for x in last_1000_games if x != 0.5]
            if catless_games == []:
                catless_games = [0, 1]
            print("Count:", count, "Win Rate (inc. cats):", sum(last_1000_games) / len(last_1000_games), "Win Rate (exc. cats):", sum(catless_games) / len(catless_games))
            
            if old_nn is not None:
                #print("nn diff", nn[0][0] - old_nn[0][0])
                #print("nn diff2", nn[0][1] - old_nn[0][1])
                pass
            old_nn = copy.deepcopy(nn)
            #input()
        count += 1

        state = env.reset()

        done = False
        feature = env.state_to_feature(state)
        step_index = 0
        #print(">>>", nn[0][0])
        while not done:
            step_index += 1

            # move the opponent's marker
            #print("moved human random")
            env.board_state = env.move_human_randomly(env.board_state)  # set to X

            # if three in a row loss, reset
            if reward.eval_reward(env.board_state) == -1:
                env.h_score += 1
                #print("Robot LOSS! Wins", env.r_score, ", Loss", env.h_score)
                last_1000_games.append(0)
                last_1000_games.remove(last_1000_games[0])
                break

            # move the robot's marker
            best_value = -1e10
            best_action = -1
            # for each action, choose the best
            for action in range(9):
                # if the action isn't valid for the board state, ignore
                if not env.move_is_valid(action):
                    continue
                
                feature_future = env.state_to_feature(env.next_board_state(action))
                # TODO: Insert code here: For each action, test with network and remember the best one
                X = np.array([feature_future]).T  # construct the input
                theoretical_value = forward_pass(nn, X)  # get the reward value
                
                if theoretical_value > best_value:  # keep track of the best action
                    best_value = theoretical_value
                    best_action = action            

            # if best action is -1, cat game
            if best_action == -1:
                #print("cat game")
                last_1000_games.append(0.5)
                last_1000_games.remove(last_1000_games[0])
                break

            # Take the best action
            env.last_r_move = best_action
            state, rew, done = env.step(best_action)
            feature = env.state_to_feature(state)
        
            f_t_2 = f_t_1
            f_t_1 = feature

            # if three in a row victory, reset
            if reward.eval_reward(env.board_state) == 1:
                env.r_score += 1
                #print("Robot victory! Wins", env.r_score, ", Loss", env.h_score)
                last_1000_games.append(1)
                last_1000_games.remove(last_1000_games[0])
                done = True

            #env.print_board()

            # check auto feedback
            FEEDBACK = env.get_auto_feedback()
            #print("FEED", FEEDBACK, done)

            # get human feedback if FEEDBACK is not 10 or -10
            if False:  # FEEDBACK not in [-10, 10]:
                # TODO: Optional, insert code here to get change update frequency
                if not done:
                    FEEDBACK = env.get_human_feedback()
                else:
                    FEEDBACK = 10

            # save the agent        
            pickle.dump(nn, open('trained_agent.pkl', 'wb'))

            env.set_feedback(FEEDBACK)

            if FEEDBACK != 0:
                # TODO: Insert code here to learn to predict the value indicated by the human
                # Obtain gradients and then update parameters
                X = (feature).reshape((9,1))
                g = backprop(nn, X, FEEDBACK, "MSE")  # get the gradients
                #print("g", g)
                #input()
                for j in range(len(nn)):  # update the parameters
                    nn[j][0] = nn[j][0] - alpha * g[j][0] + 1e-6 #/ num_epochs
                    nn[j][1] = nn[j][1] - alpha * g[j][1] + 1e-6 #/ num_epochs
                FEEDBACK = 0
            #print("!", g)

            env.board_state = state



           