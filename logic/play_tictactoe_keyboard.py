import pickle
from training.game_env import GameEnv
import pygame
from pygame.locals import *
import sys
from training.forwardPass import forward_pass  # Hint: you'll need this!
import numpy as np

def check_game_over(env):
    state = env.get_current_game_state()
    if state == 1:
        print("Robot wins!")
        return True
    if state == -1:
        print("Human wins!")
        return True
    if state == 0.5:
        print("Cat game!")  
        return True
    return False

if __name__ == "__main__":
    save = True
    env = GameEnv()

    nn = pickle.load(open('training/trained_agent_saved.pkl', 'rb'))
    env.network = nn

    numpad = True

    while True:
        env.reset()
        step_index = 0
        while not env.check_done():
            step_index += 1
            
            # get human move
            print("Human's turn!")
            env.print_board()
            human_move = None
            while True:
                human_move = input("Give X placement: ").lower()  # get a move as input

                if not human_move.isnumeric():  # check if integer
                    print("Must be an integer! Choose again.")
                    continue
                human_move = int(human_move)

                # if using a numpad, convert move to 0-9
                if numpad:
                    if 7 <= human_move <= 9:
                        human_move = human_move - 7
                    elif 4 <= human_move <= 6:
                        human_move = human_move - 1
                    elif 1 <= human_move <= 3:
                        human_move = human_move + 5

                if not 0 <= int(human_move) <= 8:  # check if the move is int and valid range
                    print("Must be a valid move index (0-9)! Choose again.")
                    continue
                
                if env.board_state[human_move] != 0:  # check if move already played
                    print("Move already has a token on it! Choose again.")
                    continue
                break

            # move the token
            print("Human is moving", human_move)
            env.move_token(token=1, position=human_move)

            # check if game is done
            if check_game_over(env):
                break

            # choose the robot's move
            print("Robot's turn!")
            env.print_board()
            robot_move = env.get_best_robot_move()
            print("Robot is moving ", robot_move)
            env.step(robot_move)

            
            # check if game is done
            if check_game_over(env):
                break
            