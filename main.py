from copy import deepcopy
from time import time as Time
import heapq

class Coordinate:
    def __init__(self, y, x):
        self.x = x
        self.y = y

    def __eq__(self, other):
        if self.x == other.x and self.y == other.y:
            return True
        return False

    def getH(self):
        return "x" + str(self.x) + "y" + str(self.y)


class State:
    def __init__(self):
        self.one = []
        self.two = []
        self.three = []
        self.p = None
        self.q = None
        self.father = None
        self.id = ""
        self.dfsDepth = -1
        self.heuristic = 0
        self.cost = 0

    def __gt__(self, other):
        if self.heuristic + self.cost >= other.heuristic + other.cost:
            return True
        return False

    def getHeuristic(self):
        return len(self.one) + len(self.two) + len(self.three)

    def setHeuristic(self):
        self.heuristic = self.getHeuristic()

    def getHash(self):
        hashS = "o"
        for i in range(len(self.one)):
            hashS += self.one[i].getH()
        hashS += "t"
        for i in range(len(self.two)):
            hashS += self.two[i].getH()
        hashS += "h"
        for i in range(len(self.three)):
            hashS += self.three[i].getH()
        hashS += "p"
        hashS += self.p.getH()
        hashS += "q"
        hashS += self.q.getH()
        return hashS

    def setHash(self):
        self.id = self.getHash()


start = Time()
file = open("Test Cases/test1")
table = file.readlines()
file.close()
statesHashMap = {}


def createInitialState():
    global statesHashMap, table
    head = State()
    for i in range(len(table)):
        for j in range(len(table[i])):
            if table[i][j] == 'P':
                head.p = Coordinate(i, j)
            elif table[i][j] == 'Q':
                head.q = Coordinate(i, j)
            elif table[i][j] == '1':
                head.one.append(Coordinate(i, j))
            elif table[i][j] == '2':
                head.two.append(Coordinate(i, j))
            elif table[i][j] == '3':
                head.three.append(Coordinate(i, j))
    head.setHash()
    head.setHeuristic()
    statesHashMap[head.id] = head
    return head


states = [createInitialState()]
undiscoveredStates = [states[0]]
frontier = [states[0]]


def copyState(state):
    newState = State()
    newState.one = deepcopy(state.one)
    newState.two = deepcopy(state.two)
    newState.three = deepcopy(state.three)
    newState.p = state.p
    newState.q = state.q
    return newState



def findState(newState):
    global states, undiscoveredStates, statesHashMap
    hashV = newState.getHash()
    if hashV in statesHashMap:
        return statesHashMap[hashV]

    states.append(newState)
    undiscoveredStates.append(newState)
    newState.id = hashV
    statesHashMap[hashV] = newState

    if len(newState.one) == len(newState.two) == len(newState.three) == 0:
        print(Time()-start)
        return 1
    else:
        return 0


def extendFrontier(newState, curState):
    global statesHashMap, frontier
    hashV = newState.getHash()
    if hashV in statesHashMap:
        oldState = statesHashMap[hashV]
        if curState.cost + 1 >= oldState.cost:
            return 0

    newState.id = hashV
    statesHashMap[hashV] = newState
    newState.setHeuristic()
    newState.cost = curState.cost + 1
    heapq.heappush(frontier, newState)
    newState.father = curState

    if len(newState.one) == len(newState.two) == len(newState.three) == 0:
        print(Time()-start)
        # printGoalDepthAStar(newState)
        return 1
    else:
        return 0

def printGoalDepthAStar(state):
    curState = state
    i = 1
    while curState.id != states[0].id:
        curState = curState.father
        i += 1
    print(i)


def findOrGenerateNewState(state, coordinate, letter):
    newState = copyState(state)
    index = find(coordinate, newState.three)
    if index == -1:
        if letter == "P":
            index = find(coordinate, newState.one)
            if index != -1:
                newState.one.pop(index)
        elif letter == "Q":
            index = find(coordinate, newState.two)
            if index != -1:
                newState.two.pop(index)
    else:
        newState.three.pop(index)

    if letter == 'P':
        newState.p = coordinate
    else:
        newState.q = coordinate
    if AStarRunning:
        return extendFrontier(newState, state)
    else:
        res = findState(newState)
        if res == 0 or res == 1:
            states[len(states)-1].father = state
        return res


def find(coord, li):
    for i in range(len(li)):
        if li[i] == coord:
            return i
    return -1

def findNextState(state, letter, direction):
    global table

    if letter == 'P':
        coord = Coordinate(state.p.y, state.p.x)
        if direction == "L":
            coord.x -= 1
        elif direction == "R":
            coord.x += 1
        elif direction == "U":
            coord.y -= 1
        else:
            coord.y += 1

        if table[coord.y][coord.x] != "%" and coord not in state.two and coord != state.q:
            return findOrGenerateNewState(state, coord, "P")

    elif letter == "Q":
        coord = Coordinate(state.q.y, state.q.x)
        if direction == "L":
            coord.x -= 1
        elif direction == "R":
            coord.x += 1
        elif direction == "U":
            coord.y -= 1
        else:
            coord.y += 1
        if table[coord.y][coord.x] != "%" and coord not in state.one and coord != state.p:
            return findOrGenerateNewState(state, coord, "Q")
    return -1

movements = [['Q', "L"], ['Q', "R"], ['Q', "U"], ['Q', "D"], ['P', "L"], ['P', "R"], ['P', "U"], ['P', "D"]]


def printGoalDepthBFS():
    global states
    cur = states[len(states)-1]
    goalHash = states[0].getHash()
    i = 1
    while cur.id != goalHash:
        cur = cur.father
        i += 1
    print(i)

# BFS
def BFS():
    global states, undiscoveredStates,visitedStates
    while len(undiscoveredStates) != 0:
        curState = undiscoveredStates[0]
        for i in range(len(movements)):
            if findNextState(curState, movements[i][0], movements[i][1]) == 1:
                return
        undiscoveredStates.pop(0)


# IDS
sameStates, visitedStates = 0, 0
def DFS(curState, maxDepth, depth):
    global states, sameStates, visitedStates
    # visitedStates += 1
    curState.dfsDepth = depth
    if depth == maxDepth:
        return 0
    for i in range(len(movements)):
        res = findNextState(curState, movements[i][0], movements[i][1])
        if res == 1:
            return 1
        elif res == 0:
            if DFS(states[len(states)-1], maxDepth, depth+1) == 1:
                return 1
        elif res != -1:
            if res.dfsDepth > depth + 1:
                if DFS(res, maxDepth, depth+1) == 1:
                    return 1
    return 0

def IDS():
    global states, statesHashMap
    i = 1
    while True:
        statesHashMap = {}
        states = [createInitialState()]
        if DFS(states[0], i, 1) == 1:
            return 1
        i += 1


# A* Search
AStarRunning, visitedStates = False, 0
def AStar():
    global visitedStates, AStarRunning, frontier, movements
    AStarRunning = True
    while len(frontier) != 0:
        curState = frontier[0]
        heapq.heappop(frontier)
        # visitedStates += 1
        for i in range(len(movements)):
            if findNextState(curState, movements[i][0], movements[i][1]) == 1:
                return

if __name__=="__main__":
    # BFS()
    # IDS()
    AStar()

