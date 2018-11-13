# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util


class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    #print "Start's successors:", problem.getSuccessors(problem.getStartState())[1][1]
    #print "start state " , problem.getStartState()
    #print "type of the direction " , type(problem.getSuccessors(problem.getStartState())[1][1])
    #print  " start's successor ", problem.getSuccessors(problem.getStartState())

    visited = []  # to keep list of visited nodes
    notVisited = util.Stack()
    notVisited.push([(problem.getStartState(), "Start", 0)])

    while not notVisited.isEmpty():

        sPath = notVisited.pop()
        #print "visiting ", sPath
        #print "Spath type", type(sPath)
        s = sPath[-1][0]
        #print "visiting ", s

        if problem.isGoalState(s):
            #print "In goal Check for ",s
            ansPath = []
            for p in sPath:
                ansPath.append(p[1])
            return ansPath[1:]

        if s not in visited:
            visited.append(s)

            for child in problem.getSuccessors(s):
                if child[0] not in visited:
                    childPath = sPath + []
                    childPath.append(child)
                    notVisited.push(childPath)

    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    visited = []  # to keep list of visited nodes
    notVisited = util.Queue()
    notVisited.push([(problem.getStartState(), "Start", 0)])

    while not notVisited.isEmpty():

        sPath = notVisited.pop()
        # print "visiting ", sPath
        # print "Spath type", type(sPath)
        s = sPath[-1][0]
        # print "visiting ", s

        if problem.isGoalState(s):
            # print "In goal Check for ",s
            ansPath = []
            for p in sPath:
                ansPath.append(p[1])
            return ansPath[1:]

        if s not in visited:
            visited.append(s)

            for child in problem.getSuccessors(s):
                if child[0] not in visited:
                    childPath = sPath + []
                    childPath.append(child)
                    notVisited.push(childPath)



    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    """print "initial state : - ", problem.getStartState()
    print "Successors of initial ", problem.getSuccessors(problem.getStartState())
    """
    visited = []  # to keep list of visited nodes
    notVisited = util.PriorityQueue()
    notVisited.push([(problem.getStartState(), "Start", 0)], 0)

    while not notVisited.isEmpty():

        sPath = notVisited.pop()
        # print "visiting ", sPath
        # print "Spath type", type(sPath)
        s = sPath[-1]
        # print "visiting ", s

        if problem.isGoalState(s[0]):
            # print "In goal Check for ",s
            ansPath = []
            for p in sPath:
                ansPath.append(p[1])
            return ansPath[1:]

        if s[0] not in visited:
            visited.append(s[0])

            for child in problem.getSuccessors(s[0]):
                if child[0] not in visited or notVisited:
                    childPath = sPath + []
                    childNew = (child[0],child[1],child[2]+s[2])
                    childPath.append(childNew)
                    notVisited.push(childPath, childNew[2])



    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    U = util.PriorityQueue()
    rhs = {}
    g = {}
    INF = float('inf')
    goalState = problem.getGoalState()
    startState = problem.getStartState()
    def calculateKey(s):
        h = util.manhattanDistance(s,goalState)
        return (min(g[s],rhs[s])+h,min(g[s],rhs[s]))

    def initilize():
        for state in problem.getAllStates():
            rhs[state] = INF
            g[state] = INF
        rhs[startState] =0
        U.push(startState,calculateKey(startState))

    def updateVertx(u):
        if(u!=startState):
            temp = INF
            for s,c  in problem.getSuccessors(u):
                t1=g[s]+c
                if temp > t1:
                    temp = t1
            rhs[u] = temp
        U.remove(u)
        if(g[u]!=rhs[u]):
            U.push(u,calculateKey(u))
    def computeShortestPath():
        while (U.topKey() < calculateKey(goalState) or rhs[goalState] != g[goalState]):
            u  =U.pop()
            if g[u]>rhs[u]:
                g[u] = rhs[u]
                for s ,c in problem.getSuccessors(u):
                    updateVertx(s)
            else:
                g[u] = INF
                list = problem.getSuccessors(u)
                list.append(u)
                for s,c in list:
                    updateVertx(s)
    def createPath():
        path = []
        path.append(goalState)
        S = goalState

        while S != startState:
            temp = INF
            minState = None
            for s , c in problem.getSuccessors(S):
                if temp > g[s]:
                    temp = g[s]
                    minState =s
            path.append(s)
            S=minState

        path.append(S)
        return path[::-1]



    initilize()
    computeShortestPath()
    print "#####"
    print g[(2,1)]
    print g[(3,1)]
    print g[(4,1)]
    print createPath()
    print z




    visited = []  # to keep list of visited nodes
    notVisited = util.PriorityQueue()
    notVisited.push([(problem.getStartState(), "Start", 0)], (0,0))

    while not notVisited.isEmpty():

        sPath = notVisited.pop()
        # print "visiting ", sPath
        # print "Spath type", type(sPath)
        s = sPath[-1]
        # print "visiting ", s

        if problem.isGoalState(s[0]):
            # print "In goal Check for ",s
            ansPath = []
            for p in sPath:
                ansPath.append(p[1])
            return ansPath[1:]

        if s[0] not in visited:
            visited.append(s[0])

            for child in problem.getSuccessors(s[0]):
                if child[0] not in visited or notVisited:
                    childPath = sPath + []
                    childNew = (child[0], child[1], child[2] + s[2])
                    childPath.append(childNew)
                    tempval = childNew[2] + heuristic(child[0], problem)
                    notVisited.push(childPath,(tempval,tempval) )
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch 
ucs = uniformCostSearch
