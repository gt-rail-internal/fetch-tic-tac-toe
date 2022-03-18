import pickle
from pong_env import PongEnv
import pygame
from pygame.locals import *
import sys
from forwardPass import forward_pass  # Hint: you'll need this!
import numpy as np

if __name__ == "__main__":
    save = True
    env = PongEnv()

    nn = pickle.load(open('trained_agent.pkl', 'rb'))

    while True:
        state = env.reset()

        done = False
        feature = env.state_to_feature(state)
        step_index = 0
        while not done:
            step_index += 1
            for event in pygame.event.get():
                if event.type == QUIT:
                    pygame.quit()
                    sys.exit()
            best_value = -1e10
            best_action = 0
            for action in [-8, 0, 8]:
                feature = env.state_to_feature(env.next_state(action))
                # TODO: Insert code here: For each action, test with network and remember the best one
                theoretical_value = forward_pass(nn, np.array([[feature]]))  # use the network to get the theoretical rewards from the future state
                if theoretical_value > best_value:  # update the best value/action, if applicable
                    best_value = theoretical_value
                    best_action = action

            state, rew, done = env.step(best_action)
            feature = env.state_to_feature(state)
            env.fps.tick(60)
            env.render()
