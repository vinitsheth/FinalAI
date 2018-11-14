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
from game import Directions

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

    U = util.PriorityQueue()
    rhs = {}
    g = {}
    INF = float('inf')
    goalState = problem.getGoalState()
    startState = problem.getStartState()
    finalPath = []
    
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
                #print g[s],c
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

                list.append((u,1))
                for s,c in list:
                    updateVertx(s)
    
    def createPath():
        path = []

        S = goalState

        while S != startState:
            temp = INF
            minState = None
            path.append(S)
            for s , c in problem.getSuccessors(S):
                if temp > g[s]:
                    temp = g[s]
                    minState =s

            S=minState

        path.append(S)
        return path[::-1]

    initilize()
    #print g.items()
    #print g[(16,1)]





    while len(finalPath) == 0 or finalPath[-1] != goalState:
        computeShortestPath()
        path = createPath()
        print "Traversing path"
        for i in range(len(path)-1):
            #print path[i+1]
            print "next Element"+str(path[i+1])
            if problem.checkWall(path[i+1]):
                #print "################"
                print "Wall Found At "+str(path[i+1])
                print "Updating"
                problem.addWall(path[i+1])
                updateVertx(path[i+1])
                #for state,c in problem.getSuccessors(path[i+1]):
                #  updateVertx(state)
                #path = createPath()
                #for p in path:
                #    print g[p], rhs[p]

                break
            elif path[i+1] == goalState:
                finalPath = path
                break

    #print finalPath

    print createPath()
    return getDirections(createPath())

    #print z
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
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
                    childNew = (child[0], child[1], child[2] + s[2])
                    childPath.append(childNew)
                    notVisited.push(childPath, childNew[2] + + heuristic(child[0], problem))

    util.raiseNotDefined()
    """

def getDirections(path):
    ans = []
    x,y = path[0]
    for i in range(1,len(path)):
        xi,yi = path[i]
        if x > xi:
            ans.append(Directions.WEST)
        if x<xi:
            ans.append(Directions.EAST)
        if y<yi:
            ans.append(Directions.NORTH)
        if y>yi:
            ans.append(Directions.SOUTH)
        x = xi
        y = yi
    return ans



def dStarSearch(problem, heuristic=nullHeuristic):
    U = util.PriorityQueue()
    rhs = {}
    g = {}
    km = 0
    INF = float('inf')
    goalState = problem.getGoalState()
    startState = problem.getStartState()
    finalPath = []

    def calculateKey(s):
        h = util.manhattanDistance(s, startState)
        return (min(g[s], rhs[s]) + h+ km, min(g[s], rhs[s]))

    def initilize():
        for state in problem.getAllStates():
            rhs[state] = INF
            g[state] = INF
        rhs[goalState] = 0
        U.push(goalState, calculateKey(goalState))

    def updateVertx(u):
        if (u != goalState):
            temp = INF
            for s, c in problem.getSuccessors(u):
                # print g[s],c
                t1 = g[s] + c
                if temp > t1:
                    temp = t1
            rhs[u] = temp
        U.remove(u)
        if (g[u] != rhs[u]):
            U.push(u, calculateKey(u))

    def computeShortestPath():
        while (U.topKey() < calculateKey(startState) or rhs[startState] != g[startState]):
            kold = U.topKey()
            u = U.pop()
            if kold < calculateKey(u):
                U.push(u,calculateKey(u))
            elif g[u] > rhs[u]:
                g[u] = rhs[u]
                #print ("a",u,g[u])
                for s, c in problem.getSuccessors(u):
                    updateVertx(s)
            else:
                g[u] = INF
                #print ("b",u,g[u])
                list = problem.getSuccessors(u)

                list.append((u, 1))
                for s, c in list:
                    updateVertx(s)

    def createPath():
        path = []

        S = problem.getStartState()

        while S != goalState:
            temp = INF
            minState = None
            path.append(S)
            #print S
            #print problem.getSuccessors(S)
            #print path
            tempList = []
            for s1, c in problem.getSuccessors(S):
                #print "in 1"
                tempList.append(s1)
            tempList1 = []
            for s2 in tempList:
                if s2 not in path:
                    tempList1.append(s2)
            for s3 in tempList1:
                #print (s3,g[s3])
                if temp > g[s3]:
                    temp = g[s3]
                    minState = s3
            S = minState
        path.append(S)
        return path
    
    sstart = startState
    slast = startState
    initilize()
    computeShortestPath()
    #print problem.getSuccessors(startState)
    #print createPath()
    #print zx
    def printg():
        templ = []
        for key,val in g.items():
            if val !=INF:
                templ.append((key,val))
        def getKey(a):
            return a[1]
        templ = sorted(templ,key=getKey)
        for t in templ:
            print t
    
    
    finalPath = []
    finalPath.append(startState)
    
    while startState != goalState:
        minimum = INF
        minimumState = None
        #print 
        for s, c in problem.getSuccessors(startState):
            temp = g[s]+c 
            if temp<minimum:
                minimum = temp
                minimumState = s
        startState = minimumState
        if problem.checkWall(startState):
            print "wall found at"+str(startState)
            problem.addWall(startState)
            km += util.manhattanDistance(slast,startState)
            startState = slast
            updateVertx(minimumState)
            computeShortestPath()
       
            
    
    """
    printg()

    
    while startState != goalState:

        minimum = INF
        minimumState = None
        #print 
        for s, c in problem.getSuccessors(startState):
            temp = g[s]+c 
            if temp<minimum:
                minimum = temp
                minimumState = s
        nextState = minimumState
        if problem.checkWall(nextState):
            print "wall found at"+str(nextState)
            problem.addWall(nextState)
            km += util.manhattanDistance(slast,nextState)
            slast = startState
            updateVertx(nextState)
            #for st , ct in problem.getSuccessors(nextState):
            #   updateVertx(st)
            
            computeShortestPath()
            #printg()
            #print g[(5,1)]
        else:
            #print startState,nextState
            finalPath.append(nextState)
            startState = nextState
    """
    #print createPath()
    #print finalPath
    return getDirections(createPath())
    


    # print g.items()
    # print g[(16,1)]







# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
dstar = dStarSearch
