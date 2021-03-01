#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os
from search import * #for search engines
from snowman import SnowmanState, Direction, snowman_goal_state #for snowball specific classes
from test_problems import PROBLEMS #20 test problems

def heur_manhattan_distance(state):
    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a snowman state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''

    if state.width > 0 and state.height > 0:
      fullDist = 0
      for snowball in state.snowballs:
        currSnowball = state.snowballs[snowball]
        x0 = snowball[0]
        y0 = snowball[1]
        tempSum = abs(x0 - state.destination[0]) + abs(y0 - state.destination[1])

        # cases for stacked snowballs
        if (currSnowball == 3 or currSnowball == 4 or currSnowball == 5):
          tempSum = tempSum * 2
        elif (currSnowball == 6):
          tempSum = tempSum * 3
        fullDist += tempSum
      return fullDist
    else:
      return 0


#HEURISTICS
def trivial_heuristic(state):
  '''trivial admissible snowball heuristic'''
  '''INPUT: a snowball state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state (# of moves required to get) to the goal.'''   
  return len(state.snowballs)

def heur_alternate(state):
    '''a better heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    global stuckStates

    stuckStates = {}

    heurValue = 0

    # checks distance of robot from goal
    robotDist = abs(state.destination[0] - state.robot[0]) + abs(state.destination[1] - state.robot[1])
    heurValue += robotDist

    fullDist = 0
    for snowball in state.snowballs:
      currSnowball = state.snowballs[snowball]
      x0 = snowball[0]
      y0 = snowball[1]

      if snowball in stuckStates:
        return float("inf")

      # return max heurValue for stuck states
      stuckBox = stuck_box(state=state, snowball_x=x0, snowball_y=y0)
      if (stuckBox == True):
        stuckStates[snowball] = True
        return float("inf")

      # calculate manhattan distance as normal
      tempSum = abs(x0 - state.destination[0]) + abs(y0 - state.destination[1])

      if (currSnowball == 3 or currSnowball == 4 or currSnowball == 5):
        tempSum = tempSum * float(0.33)
      elif (currSnowball == 6):
        tempSum = tempSum * float(0.66)
      fullDist += tempSum
      
    heurValue += fullDist
    
    return heurValue

def stuck_box(state, snowball_x, snowball_y):
  '''helper function to check if a snowball is stuck in a corner'''
  '''INPUT: a snowball state, x and y position of the snowball'''
  '''OUTPUT: a boolean value that represents whether a snowball is stuck in a corner.'''

  # whether the current snowball is at the goal already
  notDest = (state.destination != (snowball_x, snowball_y))

  # cases for corner states
  if (snowball_x == 0 and snowball_y == 0) and notDest:
    return True
  elif (snowball_x == 0 and snowball_y == state.height - 1) and notDest:
    return True
  elif (snowball_x == state.width - 1 and snowball_y == 0) and notDest:
    return True
  elif (snowball_x == state.width - 1 and snowball_y == state.height - 1) and notDest:
    return True
        
  # cases for snowball being stuck on a wrong wall
  if((snowball_x == 0 or snowball_x == state.width - 1) and snowball_x != state.destination[0]): # x case for walls
    return True
  elif((snowball_y == 0 or snowball_y == state.height - 1) and snowball_y != state.destination[1]): # y case for walls
    return True

  # if passes all cases then return False
  return False

def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0

def fval_function(sN, weight):
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
    return sN.gval + (weight * sN.hval)

def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound = 5):
  '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
  '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  '''implementation of weighted astar algorithm'''
  # start the timers
  startTime = os.times()[0]
  finalTime = startTime + timebound
  optimalResult = False

  # initialize the search engines
  searcher = SearchEngine(strategy='custom', cc_level='full')
  wrappedFvalFunction = (lambda sN : fval_function(sN,weight))
  searcher.init_search(initState=initial_state, heur_fn=heur_fn, goal_fn=snowman_goal_state, fval_function=wrappedFvalFunction)

  # initialize cost to maximum since first iter
  costBound = (float("inf"), float("inf"), float("inf")) 
  result = searcher.search(timebound=timebound)

  while startTime < finalTime:
    if result == False: # occurs if search doesn't find anything
      return optimalResult

    timebound = timebound - (os.times()[0] - startTime) # check amount of time that went by
    startTime = os.times()[0] # grabs current time 

    if (result.gval < costBound[0]): # only need to grab first index because the cost bound by the gval
      costBound = (result.gval, result.gval, result.gval) # gval used for each, outlined in handout    
      optimalResult = result

    result = searcher.search(timebound, costBound) # continue searching until time runs out

  return optimalResult

def anytime_gbfs(initial_state, heur_fn, timebound = 5):
  '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
  '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  '''implementation of weighted astar algorithm'''

  # start the timers
  startTime = os.times()[0]
  finalTime = startTime + timebound
  optimalResult = False

  # initialize the search engines
  searcher = SearchEngine(strategy='best_first', cc_level='full')
  searcher.init_search(initState=initial_state, goal_fn=snowman_goal_state, heur_fn=heur_fn)

  # initialize cost to maximum since first iter
  costBound = (float("inf"), float("inf"), float("inf")) 
  result = searcher.search(timebound=timebound)

  while startTime < finalTime:
    if result == False: # occurs if search doesn't find anything
      return optimalResult

    timebound = timebound - (os.times()[0] - startTime) # check amount of time that went by
    startTime = os.times()[0] # grabs current time 

    if (result.gval < costBound[0]): # only need to grab first index because the cost bound by the gval
      costBound = (result.gval, result.gval, result.gval) # gval used for each, outlined in handout    
      optimalResult = result

    result = searcher.search(timebound, costBound) # continue searching until time runs out

  return optimalResult