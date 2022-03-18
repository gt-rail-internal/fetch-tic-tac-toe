import pickle

from numpy.core.fromnumeric import argmax
from pong_env import PongEnv
import pygame
from pygame.locals import *
import sys
from createNeuralNet import create_neural_net
from forwardPass import forward_pass  # Hint: You'll need these two!
from backprop import backprop
import numpy as np

KEY_LABEL = 0.0


def keydown(event, trained_agent):
    global KEY_LABEL

    if event.key == K_0:
        KEY_LABEL = 4
    elif event.key == K_9:
        KEY_LABEL = 3
    elif event.key == K_8:
        KEY_LABEL = 2
    elif event.key == K_7:
        KEY_LABEL = 1
    elif event.key == K_6:
        KEY_LABEL = 0
    elif event.key == K_5:
        KEY_LABEL = -1
    elif event.key == K_4:
        KEY_LABEL = -2
    elif event.key == K_3:
        KEY_LABEL = -3
    elif event.key == K_2:
        KEY_LABEL = -4
    elif event.key == K_1:
        KEY_LABEL = -5
    elif event.key == K_DOWN:
        pickle.dump(trained_agent, open('trained_agent.pkl', 'wb'))
        pygame.quit()
        sys.exit()


def keyup(event):
    global KEY_LABEL
    KEY_LABEL = 0


if __name__ == "__main__":
    save = True
    env = PongEnv()

    # network hyperparameters
    # TODO: insert code here (Optional) Modify hyperparameters
    f_t_1 = None
    f_t_2 = None

    hidden_dims = [8]
    input_dim = 1
    output_dim = 1
    alpha = 1e-2
    nn = create_neural_net(numNodesPerLayer=hidden_dims, numInputDims=input_dim, numOutputDims=output_dim)

    while True:
        state = env.reset()

        done = False
        feature = env.state_to_feature(state)
        step_index = 0
        while not done:
            step_index += 1
            # TODO: Optional, insert code here to get change update frequency
            if step_index % 5 == 0 and env.state[2] > 0 and env.state[0] > 200:
                key_press_event = env.get_human_feedback()
                keydown(key_press_event, nn)
            env.set_feedback(KEY_LABEL)
            if KEY_LABEL != 0:
                # TODO: Insert code here to learn to predict the value indicated by the human
                # Obtain gradients and then update parameters
                X = (feature).reshape((1,1))
                g = backprop(nn, X, KEY_LABEL, "MSE")  # get the gradients
                for j in range(len(nn)):  # update the parameters
                    nn[j][0] = nn[j][0] - alpha * g[j][0] #/ num_epochs
                    nn[j][1] = nn[j][1] - alpha * g[j][1] #/ num_epochs
                KEY_LABEL = 0
                print("!", g)

            best_value = -1e10
            best_action = 0
            for action in [-8, 0, 8]:
                feature_future = env.state_to_feature(env.next_state(action))
                # TODO: Insert code here: For each action, test with network and remember the best one
                X = np.array([np.array([feature_future])])  # construct the input
                theoretical_value = forward_pass(nn, X)  # get the reward value
                
                if theoretical_value > best_value:  # keep track of the best action
                    best_value = theoretical_value
                    best_action = action            

            # Take the best action
            state, rew, done = env.step(best_action)
            feature = env.state_to_feature(state)
            env.fps.tick(60)
            env.render()
        
            f_t_2 = f_t_1
            f_t_1 = feature
