#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os  # for time functions
import math  # for infinity
from search import *  # for search engines
from sokoban import sokoban_goal_state, SokobanState, Direction, PROBLEMS  # for Sokoban specific classes and problems

# SOKOBAN HEURISTICS
def heur_alternate(state):
    # IMPLEMENT
    '''a better heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    # heur_manhattan_distance has flaws.
    # Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    # Your function should return a numeric value for the estimate of the distance to the goal.
    # EXPLAIN YOUR HEURISTIC IN THE COMMENTS. Please leave this function (and your explanation) at the top of your solution file, to facilitate marking.

    #doing this later havent come up with a good heuristic
    #possible options: distance from robot to box considered, number of obstacles in direct path


    return 0 

# CONSTANTS

#factor to multiply (decrease) heuristic weight for iterative astar
_WEIGHT_FACTOR = 0.7


def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0

def heur_manhattan_distance(state):
    # IMPLEMENT
    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    # We want an admissible heuristic, which is an optimistic heuristic.
    # It must never overestimate the cost to get from the current state to the goal.
    # The sum of the Manhattan distances between each box that has yet to be stored and the storage point nearest to it is such a heuristic.
    # When calculating distances, assume there are no obstacles on the grid.
    # You should implement this heuristic function exactly, even if it is tempting to improve it.
    # Your function should return a numeric value; this is the estimate of the distance to the goal.

    total_dist = 0

    for box in state.boxes:
        total_dist += find_closest(box, state.storage)[0]

    return total_dist

def fval_function(sN, weight):
    # IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
    #          g   +    w   *    h
    return sN.gval + weight * sN.hval

# SEARCH ALGORITHMS
def weighted_astar(initial_state, heur_fn, weight, timebound):
    # IMPLEMENT    
    '''Provides an implementation of weighted a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of weighted astar algorithm'''

    SE, setup_time = init_weighted_astar_SE(initial_state, heur_fn, weight, cc_level="full")

    return SE.search(timebound=timebound - setup_time)

def iterative_astar(initial_state, heur_fn, weight=1, timebound=5):  # uses f(n), see how autograder initializes a search line 88
    # IMPLEMENT
    '''Provides an implementation of realtime a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of iterative astar algorithm'''

    goal_state, stats, time_up = None, None, False

    #track when we began searching and when we are to stop
    global_search_start_time = os.times[0]
    search_end_time = global_search_start_time + timebound    

    SE, setup_time = init_weighted_astar_SE(initial_state, heur_fn, weight, cc_level="full")
    
    goal_state, stats = SE.search(timebound=timebound - setup_time)

    #are we out of time?
    time_up = os.times[0] > search_end_time
    #timebound needs to be adjusted, remove time taken so far
    timebound -= os.times[0] - global_search_start_time

    #We introduce a costbound for any follow up searches we have given we have time to do them
    #Dont explore paths that have a cost greater than what we have found so far
    best_path_cost = goal_state.gval

    while not time_up:
        #subtract from timebound how long current search has taken so following searches
        #get an accurate lesser timebound
        current_search_start_time = os.times[0]

        #iteratively decrease weight of heuristic, the smaller w means we focus more on
        #immediate cost, if we have time to find a solution, we are guaranteed optimality
        #as we decrease weight value though time to solve takes longer.
        adjust_SE(SE, initial_state, heur_fn, weight * _WEIGHT_FACTOR)

        better_goal_state, better_stats = SE.search(timebound, costbound=best_path_cost)

        time_up = os.times[0] > search_end_time
        timebound -= os.times[0] - current_search_start_time
        best_path_cost = better_goal_state.gval if better_goal_state.gval <= best_path_cost else best_path_cost


    #if we managed to find something better, return that
    if better_goal_state:
        return better_goal_state, stats
    else:
        return goal_state, stats

def iterative_gbfs(initial_state, heur_fn, timebound=5):  # only use h(n)
    # IMPLEMENT
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of iterative gbfs algorithm'''
    return None, None #CHANGE THIS

#HELPERS
def find_closest(box, storage):
    """
    Find closest storage point to a given box (in terms of manhattan distance)

    @param tuple box: tuple in the form (x,y) for box position
    @param set storage: frozenset of tuples of form (x,y) for storage positions
    @rtype: tuple -> closest position
    @rtype: int -> distance to that position
    """

    if box in storage:
        return 0, box

    min = math.inf
    closest_point = None

    for storage_point in storage:
        distance = abs(storage_point[0] - box[0]) + abs(storage_point[1] - box[1])
        if distance < min:
            min = distance
            closest_point = storage_point
    
    return min, closest_point

def init_weighted_astar_SE(initial_state, heur_fn, weight, cc_level):
    """
    Initialize a search engine object for weighted astar search

    @param state initial_state: initial state 
    @param function heur_fn: heuristic
    @param float weight: heuristic weight for fval
    @param str cc_level: cycle checking level
    @rtype: SearchEngine -> search engine ready to run SE.search()
    @rtype: time_taken -> time taken to set up search engine
    """

    start_time = os.times[0]

    #initialize search engine
    SE = SearchEngine(strategy="custom", cc_level=cc_level)
    SE.trace_on(level=2)

    wrapped_fval_function = (lambda sN: fval_function(sN, weight))

    SE.init_search(initial_state, goal_fn=sokoban_goal_state, 
                   heur_fn=heur_fn, fval_function=wrapped_fval_function)
    
    return SE, os.times[0] - start_time

def adjust_SE(SE, initial_state, heur_fn, new_weight):
    """
    Re-init Search engine with new weight, dont need to init an entire new SE in memory,
    just need to adjust weight of fval function, cc and other stuff stays the same.
    This is for iterative astar when we progressively decrease weight to look for
    better solutions

    @param SearchEngine SE: current search engine
    @param SokobanState inital_state: initial state
    @param function heur_fn: heuristic
    @param new_weight float: new weight to use in fval
    @rtype: new_SE -> SE with adjusted weight for fval
    """

    wrapped_fval_function = (lambda sN: fval_function(sN, new_weight))

    SE.init_search(initial_state, goal_fn=sokoban_goal_state, 
                   heur_fn=heur_fn, fval_function=wrapped_fval_function)




