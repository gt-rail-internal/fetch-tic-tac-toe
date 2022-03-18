def eval_reward(state):
    # if three in a row, robot winning, max reward
    if three_in_row(state, token=2):
        return 1
    
    # if three in a row, game lost, min reward
    if three_in_row(state, token=1):
        return -1

    # if no empty spots, cat
    if len([x for x in state if x == 0]) == 0:
        return 0.5

    # default to 0 reward
    return 0


# check if this player has open two in a row
def two_in_row(state):
    return False


# check if either player has three in a row
def three_in_row(state, token=2):
    # check across
    for i in range(0, len(state), 3):
        row = [state[i], state[i+1], state[i+2]]
        if items_are_token(row, token=token):
            #print("Three in a row!")
            return True

    # check down
    for i in range(3):
        col = [state[i], state[i+3], state[i+6]]
        if items_are_token(col, token=token):
            return True

    # check diag
    if items_are_token([state[0], state[4], state[8]], token=token) or items_are_token([state[2], state[4], state[6]], token=token):
        return True
    
    return False


# check if all items in a set are the same token
def items_are_token(items, token=2):
    if len(set(items)) == 1 and set(items).pop() == token:
        return True
    return False