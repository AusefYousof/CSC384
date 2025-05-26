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

    """
    Discussion of alternate heuristic:
    Manhattan distance suffers from its lack of consideration for the distance a robot has to a given storage box.
    To illustrate, consider the following simplified configuration, where R denotes robot, S denotes storage, 
    B denotes box, X denotes obstacle

    XXXXXXXXXXXXXXXXXXXXXXXXXX
    X                        X
    X    R           B     S X
    X   B                  S X
    X                        X
    XXXXXXXXXXXXXXXXXXXXXXXXXX

    Certainly, it is easy to predict manhattan distance heuristic given that all moves are of equal cost would not have a preference
    between any moves the R could make in this state, and may end up making its away to the B closer to S for example. This
    means that we would have to move the robot all the way back to one side of the statespace and push it back due to our misguided heuristic
    Moving the farther from S box would get us closer to the box we were originally going to move overall (box closer to S), 
    making for a lower cost solution.

    We can adjust our heuristic to do h = wRB * manhattan_robot_to_box + wBS * manhattan_box_to_storage
    with wRB and wBS being weights used to give more or less importance to either factor, which will be experimentally determined,
    although I feel as though robot to box is more important since a move that isnt moving a box is more or less wasted in most cases
    that I can think of: To simplify lets just consider the x axis/direction consider to box b1 and b2, it is clear that either 
    b1 or b2 will have an x value less or more than (and therefore closer/farther to storage) its fellow box, meaning the case where 
    we must move in some direction along x to get a box closer to storage whilst also being able to move said other box is very common.

    Instantly something that comes to mind is the issue of a box getting push into a corner, again consider a simple example with 
    Manhattan heuristic 

    XXXXXXXXXXXXXXXXXXXXXXXXXX
    X                        X
    X    R     B  X        S X
    X            X           X
    X                        X
    XXXXXXXXXXXXXXXXXXXXXXXXXX

    Manhattan distance heuristic which doesnt consider obstacles would not consider the corner B would get trapped in if we were to follow
    what it believes is most optimal which is to push to the right. Therefore we can implement a function to perform corner/trapped detection
    and return math.inf as the h value for that state.

    After these two improvements and weight tweaking have only been able to beat benchmark/alternate once on astar
    so will add improvements.

    Manhattan distance doesnt take into account whether or not a storage will be available to house a given box,
    my heuristic will adjust the manhattan distance to be ran for each box, and will pair each box with a storage
    space, such that the total cost is minimized, and will return that cost. This provides a more
    accurate and realistic depiction of the problem, the following example should illustrate this: 

    XXXXXXXXXXXXXXXXXXXXXXXXXX
    X  R                     X
    X    B B               S X
    X       S                X
    X                        X
    XXXXXXXXXXXXXXXXXXXXXXXXXX

    With typical manhattan distance, we would calculate an extremely low cost because of the storage space 
    in close proximity to both boxes, but that would require that one space to house both boxes, and my 
    robot to box proximity consideration and deadlock check doesnt do anything to help either,
    we should calculate the shortest distance from box to storage and remove that from storage so boxes after
    do not consider a storage spot that will be taken up down the line. 
    """

    for box in state.boxes:
        if box not in state.storage:
            if (deadlock(box, state.obstacles, state.height, state.width)):
                return math.inf

    #made improvement on heur_manhattan_distance
    #return heur_manhattan_distance(state)

    return _wRB * manhattan_robot_to_box(state.robots, state.boxes) + _wBS * pairwise_manhattan(state.boxes, state.storage)


# CONSTANTS
#factor to multiply (decrease) heuristic weight for iterative astar
_WEIGHT_FACTOR = 0.5
#Custom heuristic weights, wRB = weight robot to box, wBS = weight box to storage
_wRB = 1
_wBS = 0.5

_LEFT_WALL = 1
_TOP_WALL = 2
_RIGHT_WALL = 3
_BOTTOM_WALL = 4

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
        total_dist += closest_box_to_storage(box, state.storage)[0]

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

    timebound -= 0.015

    SE, setup_time = init_weighted_astar_SE(initial_state, heur_fn, weight, cc_level="full")

    return SE.search(timebound=timebound - setup_time)

def iterative_astar(initial_state, heur_fn, weight=1, timebound=5):  # uses f(n), see how autograder initializes a search line 88
    # IMPLEMENT
    '''Provides an implementation of realtime a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of iterative astar algorithm'''

    #initial autograder test prints time search is taking, tend to take ~0.01 longer than timebound,
    #subtract 0.015 from given timebound so search accounts for this
    timebound -= 0.015

    goal_state, better_goal_state = None, None

    SE, setup_time = init_weighted_astar_SE(initial_state, heur_fn, weight, cc_level="full")
    
    timebound -= setup_time
    goal_state, stats = SE.search(timebound)

    #track when we began searching and when we are to stop
    global_search_start_time = os.times()[0]
    search_end_time = global_search_start_time + timebound  

    #are we out of time?
    time_up = os.times()[0] >= search_end_time
    #timebound needs to be adjusted, remove time taken so far
    timebound -= (os.times()[0] - global_search_start_time)

    #We introduce a costbound for any follow up searches we have given we have time to do them
    #Dont explore paths that have a cost greater than what we have found so far
    costbound = [math.inf, math.inf, math.inf]

    if goal_state:
        costbound[2] = goal_state.gval
    else:
        return goal_state, stats

    while not time_up:
        #subtract from timebound how long current search has taken so following searches
        #get an accurate lesser timebound
        current_search_start_time = os.times()[0]

        #iteratively decrease weight of heuristic, the smaller w means we focus more on
        #immediate cost, if we have time to find a solution, we are guaranteed optimality
        #as we decrease weight value though time to solve takes longer.
        adjust_SE(SE, initial_state, heur_fn, weight * _WEIGHT_FACTOR)

        better_goal_state, better_stats = SE.search(timebound, costbound=costbound)

        time_up = os.times()[0] >= search_end_time
        timebound -= (os.times()[0] - current_search_start_time)
        if better_goal_state:
            costbound[2] = better_goal_state.gval if better_goal_state.gval <= costbound[2] else costbound[2]


    #if we managed to find something better, return that
    if better_goal_state:
        return better_goal_state, better_stats
    else:
        return goal_state, stats

def iterative_gbfs(initial_state, heur_fn, timebound=5):  # only use h(n)
    # IMPLEMENT
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of iterative gbfs algorithm'''

    timebound -= 0.015

    #Follows general structure of iterative a star, refer to iterative_astar() for comments

    goal_state, better_goal_state = None, None

    SE, setup_time = init_iterative_gbfs_SE(initial_state, heur_fn, timebound, "full")
    timebound -= setup_time

    global_search_start_time = os.times()[0]
    global_search_end_time = global_search_start_time + timebound
    goal_state, stats = SE.search(timebound)

    time_up = os.times()[0] >= global_search_end_time
    timebound -= (os.times()[0] - global_search_start_time)

    costbound = [math.inf, math.inf, math.inf]
    
    if goal_state:
        costbound[0] = goal_state.gval
    else:
        return goal_state, stats

    while not time_up:
        current_search_start_time = os.times()[0]
        better_goal_state, better_stats = SE.search(timebound, costbound=costbound)

        time_up = os.times()[0] >= global_search_end_time
        timebound -= (os.times()[0] - current_search_start_time)

        if better_goal_state:
            costbound[0] = better_goal_state.gval if better_goal_state.gval <= costbound[0] else costbound[0]
    

    if better_goal_state:
        return better_goal_state, better_stats
    else:
        return goal_state, stats

#HELPERS
def closest_box_to_storage(box, storage):
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

def manhattan_robot_to_box(robots, boxes):
    """
    Find sum of all closest distances of each robot to a box

    @param set robots: robots positions
    @param set boxes: boxes positions
    @rtype: total: total distance
    """

    total = 0

    for robot in robots:
        min = math.inf
        for box in boxes:
            distance = abs(box[0] - robot[0]) + abs(box[1] - robot[1])
            if distance < min:
                min = distance
        total += min
    
    return total

def deadlock(box, obstacles, height, width):
    """
    Check if box in deadlock (two adjacent corners have obstacles)
    returns infinity as h value

    @param box: box to check for
    @param obstacles: set of obstacles on board
    @rtype: bool -> True if box in deadlock, False if not
    """
    
    #corner of board checking
    if ((box[0] - 1 < 0 and box[1] - 1 < 0)
        or (box[0] - 1 < 0 and box[1] + 1 >= height)
        or (box[0] + 1 >= width and box[1] - 1 < 0)
        or (box[0] + 1 >= width and box[1] + 1 >= height)):
        return True

    #obstacle plus side of board checking    
    wall = attached_to_wall(box, height, width)
    if wall == _LEFT_WALL:
        if ((box[0], box[1] - 1) in obstacles or (box[0], box[1] + 1) in obstacles):
            return True
    elif wall == _TOP_WALL:
        if ((box[0] - 1, box[1]) in obstacles or (box[0] + 1, box[1]) in obstacles):
            return True
    elif wall == _RIGHT_WALL:
        if ((box[0], box[1] - 1) in obstacles or (box[0], box[1] + 1) in obstacles):
            return True
    elif wall == _BOTTOM_WALL:
        if ((box[0] - 1, box[1]) in obstacles or (box[0] + 1, box[1]) in obstacles):
            return True

    possible_deadlocks = [[(box[0] + 1, box[1]), (box[0], box[1] + 1)], #right and down blocked
                          [(box[0] + 1, box[1]), (box[0], box[1] - 1)], #right and up blocked
                          [(box[0] - 1, box[1]), (box[0], box[1] - 1)], #left and up blocked
                          [(box[0] - 1, box[1]), (box[0], box[1] + 1)]] #left and down blocked
    
    for deadlock_pair in possible_deadlocks:
        for obst1, obst2 in [deadlock_pair]:
            if obst1 in obstacles and obst2 in obstacles:
                return True

    return False

def attached_to_wall(box, height, width):
    """
    Check if box is attached to wall

    @param box: box to check for
    @rtype: int -> True if box attached to wall, False if not
    """
    if (box[0] - 1 < 0):
        return _LEFT_WALL
    elif (box[0] + 1 >= width):
        return _RIGHT_WALL
    elif (box[1] - 1 < 0):
        return _TOP_WALL
    elif (box[1] + 1 >= height):
        return _BOTTOM_WALL
    else:
        return 0

def pairwise_manhattan(boxes, storage):
    """
    Pair a box with its closest storage point, remove that point for other boxes

    @param boxes: boxes avail
    @param storage: set of storage points on the board
    @rtype: int -> total cost for getting each box to storage point by manhattan
    """
    storage_c = list(storage)

    #[b1, b2, b3]
    #[s1, s2, s3]

    total_cost = 0

    for box in boxes:
        if box in storage_c:
            storage_c.remove(box)
            continue

        closest = math.inf
        index_of_closest = -1
        for j in range(len(storage_c)):
            dist = manhattan_helper(box, storage_c[j])
            
            if dist < closest:
                closest = dist
                index_of_closest = j
                
        total_cost += closest
        del storage_c[index_of_closest]

    return total_cost

def manhattan_helper(p1, p2):
    #small helper to get manhattan of two points used in pairwise_manhattan
    return abs(p2[0] - p1[0]) + abs(p2[1] - p1[1])

#Sub-section of helpers, search engine init/modification functions

def init_iterative_gbfs_SE(initial_state, heur_fn, timebound, cc_level="full"):
    """
    Initialize a search engine object for iterative gbfs

    @param state initial_state: initial state 
    @param function heur_fn: heuristic
    @param float timebound: timebound for search
    @rtype: SearchEngine -> search engine ready to run SE.search()
    @rtype: time_taken -> time taken to set up search engine
    """

    start_time = os.times()[0]

    SE = SearchEngine(strategy="best_first", cc_level="full")
    SE.trace_on(level=0)
    SE.init_search(initial_state, goal_fn=sokoban_goal_state,
                   heur_fn=heur_fn)
    
    return SE, os.times()[0] - start_time

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

    start_time = os.times()[0]

    #initialize search engine
    SE = SearchEngine(strategy="custom", cc_level=cc_level)
    SE.trace_on(level=0)

    wrapped_fval_function = (lambda sN: fval_function(sN, weight))

    SE.init_search(initial_state, goal_fn=sokoban_goal_state, 
                   heur_fn=heur_fn, fval_function=wrapped_fval_function)
    
    return SE, os.times()[0] - start_time

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
    

if __name__=='__main__':
    ss = SokobanState("START", 0, None, 5, 5,  # dimensions
                 ((4, 0), (4, 4)),  # robots
                 frozenset(((3, 1), (3, 2), (3, 3))),  # boxes
                 frozenset(((0, 0), (0, 2), (0, 4))),  # storage
                 frozenset(((2, 0), (2, 1), (2, 3), (2, 4)))  # obstacles
                 )
    
    print(ss.state_string())
    print("\n\n\n\n\n\n\n\nSTART\n\n\n\n\n\n\n")
    goal_state, stats = iterative_gbfs(ss, heur_alternate, timebound = 2)

    if not goal_state:
        print("No solution found")






