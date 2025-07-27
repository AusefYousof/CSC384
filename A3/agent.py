"""
An AI player for Othello. 
"""

import random
import sys
import time
import math

# You can use the functions from othello_shared to write your AI
from othello_shared import find_lines, get_possible_moves, get_score, play_move

cache = {} # Use this for state caching

def eprint(*args, **kwargs): #use this for debugging, to print to sterr
    print(*args, file=sys.stderr, **kwargs)

'''
Ausef Yousof - CSC384 A3 Summer 2025
'''    
    
def compute_utility(board, color):
    # IMPLEMENT!
    """
    Method to compute the utility value of board.
    INPUT: a game state and the player that is in control
    OUTPUT: an integer that represents utility
    """
    scores = get_score(board)

    return scores[0] - scores[1] if color == 1 else scores[1] - scores[0]

def compute_heuristic(board, color):
    # IMPLEMENT! 
    """
    Method to heuristic value of board, to be used if we are at a depth limit.
    INPUT: a game state and the player that is in control
    OUTPUT: an integer that represents heuristic value
    """
    raise RuntimeError("Method not implemented") # Replace this line!

############ MINIMAX ###############################
def minimax_min_node(board, color, limit, caching = 0):
    # IMPLEMENT!
    """
    A helper function for minimax that finds the lowest possible utility
    """
    # HINT:
    # 1. Get the allowed moves
    # 2. Check if we are at terminal state
    # 3. If not, for each possible move, get the max utiltiy
    # 4. After checking every move, you can find the minimum utility
    # ...
    
    min_utility = math.inf
    moves = get_possible_moves(board, 2 if color == 1 else 1)
    chosen_move = None


    if caching and (board, color) in cache:
        return cache[board, color]

    if len(moves) == 0 or limit == 0:
        return (None, compute_utility(board, color))

    for move in moves:
        new_board = play_move(board, 2 if color == 1 else 1, move[0], move[1])        
        utility = minimax_max_node(new_board, color, limit-1, caching)[1]

        if utility < min_utility:
            min_utility = utility
            chosen_move = move
    
    if caching:
        cache[(board, color)] = (chosen_move, min_utility)
    
    return (chosen_move, min_utility)


def minimax_max_node(board, color, limit, caching = 0):
    # IMPLEMENT!
    """
    A helper function for minimax that finds the highest possible utility
    """
    # HINT:
    # 1. Get the allowed moves
    # 2. Check if w are at terminal state
    # 3. If not, for each possible move, get the min utiltiy
    # 4. After checking every move, you can find the maximum utility
    # ...
    max_utility = -math.inf
    moves = get_possible_moves(board, color)
    chosen_move = None

    if caching and (board, color) in cache:
        return cache[board, color]

    if len(moves) == 0 or limit == 0:
        return (None, compute_utility(board, color))

    for move in moves:
        new_board = play_move(board, color, move[0], move[1])

        utility = minimax_min_node(new_board, color, limit-1, caching)[1]
        
        if utility > max_utility:
            max_utility = utility
            chosen_move = move

    if caching:
        cache[(board, color)] = (chosen_move, max_utility)
    
    return (chosen_move, max_utility)


def select_move_minimax(board, color, limit, caching = 0):
    # IMPLEMENT!
    """
    Given a board and a player color, decide on a move using Minimax algorithm. 
    Note that other parameters are accepted by this function:
    If limit is a positive integer, your code should enfoce a depth limit that is equal to the value of the parameter.
    Search only to nodes at a depth-limit equal to the limit.  If nodes at this level are non-terminal return a heuristic 
    value (see compute_utility)
    If caching is ON (i.e. 1), use state caching to reduce the number of state evaluations.
    If caching is OFF (i.e. 0), do NOT use state caching to reduce the number of state evaluations.
    INPUT: a game state, the player that is in control, the depth limit for the search, and a flag determining whether state caching is on or not
    OUTPUT: a tuple of integers (i,j) representing a move, where i is the column and j is the row on the board.
    """
    global cache
    cache.clear()

    return minimax_max_node(board, color, limit, caching)[0]


############ ALPHA-BETA PRUNING #####################
def alphabeta_min_node(board, color, alpha, beta, limit, caching = 0, ordering = 0):
    # IMPLEMENT!
    """
    A helper function for alpha-beta that finds the lowest possible utility (don't forget to utilize and update alpha and beta!)
    """
    if caching and (board, color) in cache:
        return cache[board, color]

    if ordering:
        moves = get_sorted_moves(board, 2 if color == 1 else 1)
    else:
        moves = get_possible_moves(board, 2 if color == 1 else 1)

    min_util = math.inf
    chosen_move = None

    if len(moves) == 0 or limit == 0:
        return None, compute_utility(board, color)
    
    for move in moves:
        new_board = play_move(board, 2 if color == 1 else 1, move[0], move[1])
    
        util = alphabeta_max_node(new_board, color, alpha, beta, limit-1, caching, ordering)[1]
        
        if util < min_util:
            min_util = util
            chosen_move = move

        if beta > min_util:
            beta = min_util
            if beta <= alpha:
                break

    if caching:
        cache[board, color] = (chosen_move, min_util)
    
    return (chosen_move, min_util)

def alphabeta_max_node(board, color, alpha, beta, limit, caching = 0, ordering = 0):
    # IMPLEMENT!
    """
    A helper function for alpha-beta that finds the highest possible utility (don't forget to utilize and update alpha and beta!)
    """
    #alpha at max node: highest value of children
    #beta at max node: lowest value of my sister nodes (our ancestor is a min node so min would just choose that)

    if caching and (board, color) in cache:
        return cache[board, color]

    if ordering:
        moves = get_sorted_moves(board, color)
    else:
        moves = get_possible_moves(board, color)

    max_util = -math.inf
    chosen_move = None

    if len(moves) == 0 or limit == 0:
        return None, compute_utility(board, color)
    
    for move in moves:
        new_board = play_move(board, color, move[0], move[1])

        util = alphabeta_min_node(new_board, color, alpha, beta, limit-1, caching, ordering)[1]
        
        if util > max_util:
            max_util = util
            chosen_move = move

        if alpha < max_util:
            alpha = max_util
            if beta <= alpha:
                break

    if caching:
        cache[board, color] = (chosen_move, max_util)

    return (chosen_move, max_util)

def select_move_alphabeta(board, color, limit = -1, caching = 0, ordering = 0):
    # IMPLEMENT!
    """
    Given a board and a player color, decide on a move using Alpha-Beta algorithm. 
    Note that other parameters are accepted by this function:
    If limit is a positive integer, your code should enfoce a depth limit that is equal to the value of the parameter.
    Search only to nodes at a depth-limit equal to the limit.  If nodes at this level are non-terminal return a heuristic 
    value (see compute_utility)
    If caching is ON (i.e. 1), use state caching to reduce the number of state evaluations.
    If caching is OFF (i.e. 0), do NOT use state caching to reduce the number of state evaluations.    
    If ordering is ON (i.e. 1), use node ordering to expedite pruning and reduce the number of state evaluations. 
    If ordering is OFF (i.e. 0), do NOT use node ordering to expedite pruning and reduce the number of state evaluations. 
    INPUT: a game state, the player that is in control, the depth limit for the search, a flag determining whether state caching is on or not, a flag determining whether node ordering is on or not
    OUTPUT: a tuple of integers (i,j) representing a move, where i is the column and j is the row on the board.
    """
    global cache
    cache.clear()

    return alphabeta_max_node(board, color, -math.inf, math.inf, limit, caching, ordering)[0]

####################################################
def run_ai():
    """
    This function establishes communication with the game manager.
    It first introduces itself and receives its color.
    Then it repeatedly receives the current score and current board state until the game is over.
    """
    print("Othello AI") # First line is the name of this AI
    arguments = input().split(",")
    
    color = int(arguments[0]) # Player color: 1 for dark (goes first), 2 for light. 
    limit = int(arguments[1]) # Depth limit
    minimax = int(arguments[2]) # Minimax or alpha beta
    caching = int(arguments[3]) # Caching 
    ordering = int(arguments[4]) # Node-ordering (for alpha-beta only)

    if (minimax == 1): eprint("Running MINIMAX")
    else: eprint("Running ALPHA-BETA")

    if (caching == 1): eprint("State Caching is ON")
    else: eprint("State Caching is OFF")

    if (ordering == 1): eprint("Node Ordering is ON")
    else: eprint("Node Ordering is OFF")

    if (limit == -1): eprint("Depth Limit is OFF")
    else: eprint("Depth Limit is ", limit)

    if (minimax == 1 and ordering == 1): eprint("Node Ordering should have no impact on Minimax")

    while True: # This is the main loop
        # Read in the current game status, for example:
        # "SCORE 2 2" or "FINAL 33 31" if the game is over.
        # The first number is the score for player 1 (dark), the second for player 2 (light)
        next_input = input()
        status, dark_score_s, light_score_s = next_input.strip().split()
        dark_score = int(dark_score_s)
        light_score = int(light_score_s)

        if status == "FINAL": # Game is over.
            print
        else:
            board = eval(input()) # Read in the input and turn it into a Python
                                  # object. The format is a list of rows. The
                                  # squares in each row are represented by
                                  # 0 : empty square
                                  # 1 : dark disk (player 1)
                                  # 2 : light disk (player 2)

            # Select the move and send it to the manager
            if (minimax == 1): # run this if the minimax flag is given
                movei, movej = select_move_minimax(board, color, limit, caching)
            else: # else run alphabeta
                movei, movej = select_move_alphabeta(board, color, limit, caching, ordering)
            
            print("{} {}".format(movei, movej))

if __name__ == "__main__":
    run_ai()


def get_sorted_moves(board, color):
    moves = get_possible_moves(board, color)
    return sorted(moves, key=lambda m: compute_utility(play_move(board, color, m[0], m[1]), color), reverse=True)
