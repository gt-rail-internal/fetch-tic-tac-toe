import numpy as np
import pygame
from pygame.locals import *
import sys

# colors
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLACK = (0, 0, 0)

# globals
WIDTH = 600
HEIGHT = 400
BALL_RADIUS = 20
PAD_WIDTH = 8
PAD_HEIGHT = 80
HALF_PAD_WIDTH = PAD_WIDTH / 2
HALF_PAD_HEIGHT = PAD_HEIGHT / 2


class PongEnv:
    def __init__(self):
        pygame.init()
        self.fps = pygame.time.Clock()
        self.ball_pos = [0.0, 0.0]
        self.ball_vel = [0.0, 0.0]
        self.paddle1_pos = [HALF_PAD_WIDTH - 1, HEIGHT / 2]
        self.paddle1_vel = 0.0
        self.paddle2_pos = [WIDTH + 1 - HALF_PAD_WIDTH, HEIGHT / 2]
        self.paddle2_vel = 0.0
        self.l_score = 0
        self.r_score = 0
        self.run_num = 0
        self.last_feedback = 0
        # canvas declaration
        self.window = pygame.display.set_mode((WIDTH, HEIGHT), 0, 32)
        pygame.display.set_caption('Pong Game')

    def reset(self):
        self.run_num += 1
        self.paddle1_pos = [HALF_PAD_WIDTH - 1, HEIGHT / 2]
        self.paddle2_pos = [WIDTH + 1 - HALF_PAD_WIDTH, HEIGHT / 2]
        # self.l_score = 0.0
        # self.r_score = 0.0
        self.ball_pos = [WIDTH / 2, HEIGHT / 2]
        self.ball_vel = [3, -3]
        return self.state

    def next_state(self, action):
        # get the next state, but don't actually transit to it!
        backup_states = [self.ball_pos.copy(),
                         self.ball_vel.copy(),
                         self.paddle1_pos.copy(),
                         self.paddle1_vel,
                         self.paddle2_pos.copy(),
                         self.paddle2_vel,
                         self.l_score,
                         self.r_score]
        next_state, _, _ = self.step(action)
        self.ball_pos, self.ball_vel, self.paddle1_pos, self.paddle1_vel, self.paddle2_pos, self.paddle2_vel, self.l_score, self.r_score = backup_states
        return next_state

    def step(self, action):
        done = False
        reward = 0.0
        # update paddle's vertical position, keep paddle on the screen
        if self.paddle1_pos[1] < self.ball_pos[1]:
            self.paddle1_vel = 8
        elif self.paddle1_pos[1] > self.ball_pos[1]:
            self.paddle1_vel = -8
        if HALF_PAD_HEIGHT < self.paddle1_pos[1] < HEIGHT - HALF_PAD_HEIGHT:
            self.paddle1_pos[1] += self.paddle1_vel
        elif self.paddle1_pos[1] == HALF_PAD_HEIGHT and self.paddle1_vel > 0:
            self.paddle1_pos[1] += self.paddle1_vel
        elif self.paddle1_pos[1] == HEIGHT - HALF_PAD_HEIGHT and self.paddle1_vel < 0:
            self.paddle1_pos[1] += self.paddle1_vel

        self.paddle2_vel = action
        if HALF_PAD_HEIGHT < self.paddle2_pos[1] < HEIGHT - HALF_PAD_HEIGHT:
            self.paddle2_pos[1] += self.paddle2_vel
        elif self.paddle2_pos[1] == HALF_PAD_HEIGHT and self.paddle2_vel > 0:
            self.paddle2_pos[1] += self.paddle2_vel
        elif self.paddle2_pos[1] == HEIGHT - HALF_PAD_HEIGHT and self.paddle2_vel < 0:
            self.paddle2_pos[1] += self.paddle2_vel

        # update ball
        self.ball_pos[0] += int(self.ball_vel[0])
        self.ball_pos[1] += int(self.ball_vel[1])

        # ball collision check on top and bottom walls
        if int(self.ball_pos[1]) <= BALL_RADIUS:
            self.ball_vel[1] = - self.ball_vel[1]
        if int(self.ball_pos[1]) >= HEIGHT + 1 - BALL_RADIUS:
            self.ball_vel[1] = -self.ball_vel[1]

        # ball collison check on gutters or paddles
        if int(self.ball_pos[0]) <= BALL_RADIUS + PAD_WIDTH and \
                int(self.ball_pos[1]) in range(int(self.paddle1_pos[1] - HALF_PAD_HEIGHT),
                                               int(self.paddle1_pos[1] + HALF_PAD_HEIGHT), 1):
            self.ball_vel[0] = -self.ball_vel[0]
            self.ball_vel[0] *= 1.1
            self.ball_vel[1] *= 1.1
        elif int(self.ball_pos[0]) <= BALL_RADIUS + PAD_WIDTH:
            self.r_score += 1
            reward = 1.0
            done = True

        if int(self.ball_pos[0]) >= WIDTH + 1 - BALL_RADIUS - PAD_WIDTH \
                and int(self.ball_pos[1]) in range(int(self.paddle2_pos[1] - HALF_PAD_HEIGHT),
                                                   int(self.paddle2_pos[1] + HALF_PAD_HEIGHT), 1):
            self.ball_vel[0] = -self.ball_vel[0]
            self.ball_vel[0] *= 1.1
            self.ball_vel[1] *= 1.1
        elif int(self.ball_pos[0]) >= WIDTH + 1 - BALL_RADIUS - PAD_WIDTH:
            self.l_score += 1
            reward = -1.0
            done = True

        return self.state, reward, done

    @property
    def state(self):
        # 2 + 2 + 4 = 8-dimensional
        return np.array([self.ball_pos[0], self.ball_pos[1],
                         self.ball_vel[0], self.ball_vel[1],
                         self.paddle1_pos[1], self.paddle1_vel, self.paddle2_pos[1], self.paddle2_vel])

    def state_to_feature(self, state):
        # TODO: insert code here (Optional) Modify feature space
        # return state / np.array([WIDTH, HEIGHT, WIDTH, HEIGHT, HEIGHT, HEIGHT, HEIGHT, HEIGHT])
        return np.array((state[1] - state[-2]) / HEIGHT)

    def get_human_feedback(self):
        self.render(waiting=True)
        # pygame.event.wait(10*1000)  # Triggers on too many things...
        while True:
            for event in pygame.event.get():
                if event.type == QUIT:
                    pygame.quit()
                    sys.exit()
                if event.type == KEYDOWN and event.key in [K_0, K_9, K_1, K_2, K_3, K_4, K_5, K_6, K_7, K_8, K_DOWN]:
                    return event

    def set_feedback(self, feedback):
        self.last_feedback = feedback

    def render(self, waiting=False):
        canvas = self.window
        canvas.fill(BLACK)
        pygame.draw.line(canvas, WHITE, [WIDTH / 2, 0], [WIDTH / 2, HEIGHT], 1)
        pygame.draw.line(canvas, WHITE, [PAD_WIDTH, 0], [PAD_WIDTH, HEIGHT], 1)
        pygame.draw.line(canvas, WHITE, [WIDTH - PAD_WIDTH, 0], [WIDTH - PAD_WIDTH, HEIGHT], 1)
        pygame.draw.circle(canvas, WHITE, [WIDTH // 2, HEIGHT // 2], 70, 1)

        # draw paddles and ball
        pygame.draw.circle(canvas, RED, self.ball_pos, 20, 0)
        pygame.draw.polygon(canvas, GREEN, [[self.paddle1_pos[0] - HALF_PAD_WIDTH, self.paddle1_pos[1] - HALF_PAD_HEIGHT],
                                            [self.paddle1_pos[0] - HALF_PAD_WIDTH, self.paddle1_pos[1] + HALF_PAD_HEIGHT],
                                            [self.paddle1_pos[0] + HALF_PAD_WIDTH, self.paddle1_pos[1] + HALF_PAD_HEIGHT],
                                            [self.paddle1_pos[0] + HALF_PAD_WIDTH, self.paddle1_pos[1] - HALF_PAD_HEIGHT]], 0)
        pygame.draw.polygon(canvas, GREEN, [[self.paddle2_pos[0] - HALF_PAD_WIDTH, self.paddle2_pos[1] - HALF_PAD_HEIGHT],
                                            [self.paddle2_pos[0] - HALF_PAD_WIDTH, self.paddle2_pos[1] + HALF_PAD_HEIGHT],
                                            [self.paddle2_pos[0] + HALF_PAD_WIDTH, self.paddle2_pos[1] + HALF_PAD_HEIGHT],
                                            [self.paddle2_pos[0] + HALF_PAD_WIDTH, self.paddle2_pos[1] - HALF_PAD_HEIGHT]], 0)

        # update scores
        myfont1 = pygame.font.SysFont("Comic Sans MS", 20)
        label1 = myfont1.render("Score " + str(self.l_score), 1, (255, 255, 0))
        canvas.blit(label1, (50, 20))

        myfont2 = pygame.font.SysFont("Comic Sans MS", 20)
        label2 = myfont2.render("Score " + str(self.r_score), 1, (255, 255, 0))
        canvas.blit(label2, (470, 20))

        myfont3 = pygame.font.SysFont("Comic Sans MS", 20)
        if waiting:
            label3 = myfont3.render("Waiting for feedback... ", 1, (255, 255, 0))
        else:
            label3 = myfont3.render("Last Feedback: " + str(self.last_feedback), 1, (255, 255, 0))
        canvas.blit(label3, (200, 20))

        pygame.display.update()


if __name__ == "__main__":
    env = PongEnv()
    env.reset()
    for _ in range(1500):
        env.step(0)
        env.render()
