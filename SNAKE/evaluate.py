import environments_fully_observable 
#import environments_partially_observable
import numpy as np
import keras
from keras import layers
from tqdm import trange
import matplotlib.pyplot as plt
import random
import tensorflow as tf
tf.random.set_seed(0)
random.seed(0)
np.random.seed(0)

def get_env(n):
    # n is the number of boards that you want to simulate parallely
    # size is the size of each board, also considering the borders
    # mask for the partially observable, is the size of the local neighborhood
    size = 7
    e = environments_fully_observable.OriginalSnakeEnvironment(n, size)
    # or environments_partially_observable.OriginalSnakeEnvironment(n, size, 2)
    return e

def decide_move_direction(env_):
    state = env_.to_state()
    boards = env_.boards.astype(int)
    num_boards = state.shape[0]
    actions = []
    
    # Define movement constants
    UP = 0
    RIGHT = 1
    DOWN = 2
    LEFT = 3
    NONE = 4
    
    HEAD = 4
    BODY = 3
    FRUIT = 2
    EMPTY = 1
    WALL = 0
    
    for index in range(num_boards):
        curr_state = state[index]
        head_pos = np.where(curr_state[..., 3])  # Extract head positions
        fruit_pos = np.where(curr_state[..., 1])  # Extract fruit positions
        curr_board = boards[index]
        
        count = 0
        
        head_pos = (head_pos[0][0], head_pos[1][0])
        fruit_pos = (fruit_pos[0][0], fruit_pos[1][0])
        if abs(head_pos[0] - fruit_pos[0]) + abs(head_pos[1] - fruit_pos[1]) == 1:
            # If fruit is adjacent, calculate the direction towards the fruit
            if fruit_pos[0] < head_pos[0]:
                actions.append(DOWN)
            elif fruit_pos[0] > head_pos[0]:
                actions.append(UP)
            elif fruit_pos[1] < head_pos[1]:
                actions.append(LEFT)
            elif fruit_pos[1] > head_pos[1]:
                actions.append(RIGHT)
        else:
            # Else find all empty cells around the head
            walls_head = np.zeros(4)
            empty_cells_head = np.full((2, 4), 100, dtype = int)
            neighbor_offsets = [(0, -1), (-1, 0), (0, 1), (1, 0)]  # Left, Up, Right, Down

            # Check the cells around the head
            for offset in neighbor_offsets:
                neighbor_pos = (head_pos[0] + offset[0], head_pos[1] + offset[1])
                if curr_board[neighbor_pos[0], neighbor_pos[1]] == 1:
                    empty_cells_head[0][count] = neighbor_pos[0]
                    empty_cells_head[1][count] = neighbor_pos[1]
                    count = count + 1
                elif curr_board[neighbor_pos[0], neighbor_pos[1]] == 0:
                    walls_head[count] = 1
            #If there are no empty cells around the head identify the wall and choose that direction
            if np.all(empty_cells_head == 100) and np.any(walls_head):
                direction = np.nonzero(walls_head)[0]
                if direction == 0:
                    actions.append(LEFT)
                elif direction == 1:
                    actions.append(UP)
                elif direction == 2:
                    actions.append(RIGHT)
                elif direction == 3:
                    actions.append(DOWN)
            else:
                # Calculate distances from the empty cells to the fruit
                empty_cell_distances = np.abs(empty_cells_head[0] - fruit_pos[0]) + np.abs(empty_cells_head[1] - fruit_pos[1])

                # Find the empty cell nearest to the fruit
                min_distance_index = np.argmin(empty_cell_distances)

                # Select only the cells asjacent to the head of the snake

                nearest_empty_cell = (empty_cells_head[0][min_distance_index], empty_cells_head[1][min_distance_index])

                # Determine the direction to move based on the position of the nearest empty cell relative to the head
                if nearest_empty_cell[0] < head_pos[0]:
                    actions.append(DOWN)
                elif nearest_empty_cell[0] > head_pos[0]:
                    actions.append(UP)
                elif nearest_empty_cell[1] < head_pos[1]:
                    actions.append(LEFT)
                elif nearest_empty_cell[1] > head_pos[1]:
                    actions.append(RIGHT)

    actions = np.array(actions)      # Convert to Numpy array
    actions = actions.reshape(-1, 1)  # Reshape it
    return actions

q_values = np.zeros(4)
num_boards = 1000
ITERATIONS = 1000
random_env = get_env(num_boards)
random_rewards = []
learnt_env = get_env(num_boards)
learnt_rewards = []
fruit_env = get_env(num_boards) 
fruit_rewards = []  
trained_q_estimator = tf.saved_model.load('trained_q_estimator')
actions = np.zeros(num_boards)


for iteration in trange(ITERATIONS):
	# RANDOM
    probs = tf.convert_to_tensor([[.25]*4]*num_boards)
    #sample actions
    actions =  tf.random.categorical(tf.math.log(probs), 1, dtype=tf.int32)
    # MDP update
    rewards = random_env.move(actions)
    random_rewards.append(np.mean(rewards))

    # AGENT
    state = learnt_env.to_state()
    q_values = trained_q_estimator(state)
    actions = np.zeros(num_boards, dtype=np.int32)
    for i in range(num_boards):
        actions[i] = np.argmax(q_values[i])
    # Reshape actions values 
    actions = actions.reshape(-1,1)
    # Apply the selected action
    rewards = learnt_env.move(actions)
    learnt_rewards.append(np.mean(rewards))

    # BASELINE
    actions = decide_move_direction(fruit_env)
    rewards = fruit_env.move(actions)
    fruit_rewards.append(np.mean(rewards))


random_result = np.mean(random_rewards)
learnt_result = np.mean(learnt_rewards)
fruit_result = np.mean(fruit_rewards)
print("Results over 1000 iterations (1000 7x7 boards per iteration):")
print("Random Policy", random_result)
print("Baseline Policy", fruit_result)
print("Agent:", learnt_result)
