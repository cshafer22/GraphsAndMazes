import os, sys, math, random, time, numpy as np, matplotlib.pyplot as pl

class Node():
    # builds a basic node class. x represents input row. y represents input column.
    # z value sets weight at 0 or maxint where maxint represents a block space on graph.
    # designates whether this node is a start node or a goal node. the neighbors list
    # indicataes a list of adjacent nodes in an eventual graph structure.
    def __init__(self, x, y, z):
        self.name = "node"+"("+str(x)+","+str(y)+")"
        self.row = x
        self.column = y
        self.isGoal = False
        self.isStart = False
        self.cost = 0
        if z == 1:
            self.weight = sys.maxsize
        else:
            self.weight = 0
        self.neighbors = []
        self.parent = None
        self.parentAlt = None

    # function to set cost where moving along a row costs 1 or moving along a column costs 2.
    # can be changed arbitrarily for different purposes/needs.
    def setCost(self, node):
        if (self.row - node.row) == 0:
            self.cost = node.cost + 1
        else:
            self.cost = node.cost + 2

    # returns parents of node in a path. ParentAlt specifically used for secondary path finding
    # involved Bidirectional Search.
    def getParent(self):
        return self.parent
    def getParentAlt(self):
        return self.parentAlt

class Graph():
    # builds graph structure as a list of nodes
    def __init__(self):
        self.nodes = []

    # adds node to graph
    def addNode(self, node):
        self.nodes.append(node)

    # generates a random maze. importantly, once that is done, iteration through
    # each row connecting all the edges in the row to each other in a lattice.
    # then it moves accross the columns downward in the second iteration.
    def generateRandomMaze(self, x, p):
        for i in range(0,x):
            for j in range(0,x):
                val = 0
                if random.random() <= p:
                    val = 1
                name = Node(i, j, val)

                self.addNode(name)

        for i in range(-1, len(self.nodes) - 1):
            if i not in range(-1, len(self.nodes) - 1, x):
                self.nodes[i].neighbors.append(self.nodes[i + 1])
                self.nodes[i+1].neighbors.append(self.nodes[i])
        
        for i in range(0, len(self.nodes) - x):
            self.nodes[i].neighbors.append(self.nodes[i + x])
            self.nodes[x + i].neighbors.append(self.nodes[i])

    # generates a maze given certain inputs. 'a' dictates start node.
    # 'b' denotes start node column. 'c' and 'd' respectively represent
    # the same for the goal node. n is the number of rows and columns in
    # the lattice graph. then the iteration mimics the random process
    # above to deteriministically model the necessary graph.
    def generateGivenMaze(self, maze, a, b, c, d, n):
        for i in range(len(maze)):
            list = maze[i].split()
            x = int(list[0])
            y = int(list[1])
            z = int(list[2])
            name = Node(x, y, z)
            self.addNode(name)

        for i in range(-1, len(self.nodes) - 1):
            if i not in range(-1, len(self.nodes) - 1, n):
                self.nodes[i].neighbors.append(self.nodes[i + 1])
                self.nodes[i + 1].neighbors.append(self.nodes[i])

        for i in range(0, len(self.nodes) - n):
            self.nodes[i].neighbors.append(self.nodes[i + n])
            self.nodes[n + i].neighbors.append(self.nodes[i])

        # this sets value for start and goal nodes depending on input
        for i in range(len(self.nodes)):
            if self.nodes[i].name == 'node('+str(a)+','+str(b)+')':
                self.nodes[i].isStart = True
            if self.nodes[i].name == 'node('+str(c)+','+str(d)+')':
                self.nodes[i].isGoal = True
                
# various metrics by which A* algorithm can weigh distance. more can be added or manipulated to
# desired specifications. could be generalized more in the future if necessary for user input.
def EuclideanMetric(node, goal):
    return math.sqrt((goal.column - node.column)**2 + (goal.row - node.row)**2)

def TaxiCabMetric(node, goal):
    return math.fabs(goal.column - node.column) + math.fabs(goal.row - node.row)

def MinTaxiEuclidMetric(node, goal):
    return min(TaxiCabMetric(node, goal), EuclideanMetric(node, goal))

def InfinityMetric(node, goal):
    return max(math.fabs(goal.column - node.column), math.fabs(goal.row - node.row))

def MaxTaxiEuclidMetric(node, goal):
    return max(TaxiCabMetric(node, goal), EuclideanMetric(node, goal))

def AdjustedEuclideanMetric(node, goal):
    return math.sqrt((2*(goal.column - node.column))**2 + (goal.row - node.row)**2)

def AdjustedTaxiMetric(node, goal):
    return 2*math.fabs(goal.column - node.column) + math.fabs(goal.row - node.row)

def AdjustedMaxMetric(node, goal):
    return max(AdjustedEuclideanMetric(node, goal), AdjustedTaxiMetric(node, goal))

def AdjustedMinMetric(node, goal):
    return min(AdjustedEuclideanMetric(node, goal), AdjustedEuclideanMetric(node, goal))    

# priority queue object to operate operate as the fringe of A* algorithm
class PriorityQueue():
    def __init__(self):
            self.queue = []

    def put(self, node):
        self.queue.append(node)
    
    def isEmpty(self):
        return len(self.queue) == 0

    def printQueue(self):
        for i in range(0, len(self.queue)):
            print(str(self.queue[i].name)+'---'+str(self.queue[i].cost))
        print("END")

    # print out the queue given a certain metric
    def printQueueMetric(self, goal, metricFlag):
            for i in range(0, len(self.queue)):

                if metricFlag == 2:
                    print(str(self.queue[i].name)+'---'+str(self.queue[i].cost)+'+'
                        +str(EuclideanMetric(self.queue[i], goal)))
                elif metricFlag == 3:
                    print(str(self.queue[i].name)+'---'+str(self.queue[i].cost)+'+'
                        +str(TaxiCabMetric(self.queue[i], goal)))
                elif metricFlag == 4:
                    print(str(self.queue[i].name)+'---'+str(self.queue[i].cost)+'+'
                        +str(MinTaxiEuclidMetric(self.queue[i], goal)))
                elif metricFlag == 5:
                    print(str(self.queue[i].name)+'---'+str(self.queue[i].cost)+'+'
                        +str(MaxTaxiEuclidMetric(self.queue[i], goal)))
                elif metricFlag == 6:
                    print(str(self.queue[i].name)+'---'+str(self.queue[i].cost)+'+'
                        +str(AdjustedTaxiMetric(self.queue[i], goal)))
                elif metricFlag == 7:
                    print(str(self.queue[i].name)+'---'+str(self.queue[i].cost)+'+'
                        +str(AdjustedEuclideanMetric(self.queue[i], goal)))
                elif metricFlag == 8:
                    print(str(self.queue[i].name)+'---'+str(self.queue[i].cost)+'+'
                        +str(AdjustedMaxMetric(self.queue[i], goal)))
                elif metricFlag == 9:
                    print(str(self.queue[i].name)+'---'+str(self.queue[i].cost)+'+'
                        +str(InfinityMetric(self.queue[i], goal)))

            print("END")

    # standard uniform cost search fringe returning the featured value
    def get(self):
        if len(self.queue) == 1:
            next = self.queue[0]
            del self.queue[0]
            return next

        index = 0

        for i in range(len(self.queue)):
            node = self.queue[i]
            if node.cost < self.queue[index].cost:
                index = i

        next = self.queue[index]
        del self.queue[index]
        return next
    
    # fringe build for A* with metrics determining priority
    def getMetric(self, goal, metricFlag):
        if len(self.queue) == 1:
            next = self.queue[0]
            del self.queue[0]
            return next

        index = 0
        if metricFlag == 2:
            for i in range(len(self.queue)):
                node = self.queue[i]
                metricEst = EuclideanMetric(node, goal)
                metricPot = EuclideanMetric(self.queue[index], goal)
        if metricFlag == 3:
            for i in range(len(self.queue)):
                node = self.queue[i]
                metricEst = TaxiCabMetric(node, goal)
                metricPot = TaxiCabMetric(self.queue[index], goal)    
        if metricFlag == 4:
            for i in range(len(self.queue)):
                node = self.queue[i]
                metricEst = MinTaxiEuclidMetric(node, goal)
                metricPot = MinTaxiEuclidMetric(self.queue[index], goal)
        if metricFlag == 5:
            for i in range(len(self.queue)):
                node = self.queue[i]
                metricEst = MaxTaxiEuclidMetric(node, goal)
                metricPot = MaxTaxiEuclidMetric(self.queue[index], goal)
        if metricFlag == 6:
            for i in range(len(self.queue)):
                node = self.queue[i]
                metricEst = AdjustedTaxiMetric(node, goal)
                metricPot = AdjustedTaxiMetric(self.queue[index], goal)
        if metricFlag == 7:
            for i in range(len(self.queue)):
                node = self.queue[i]
                metricEst = AdjustedEuclideanMetric(node, goal)
                metricPot = AdjustedEuclideanMetric(self.queue[index], goal)
        if metricFlag == 8:
            for i in range(len(self.queue)):
                node = self.queue[i]
                metricEst = AdjustedMaxMetric(node, goal)
                metricPot = AdjustedMaxMetric(self.queue[index], goal)
        if metricFlag == 9:
            for i in range(len(self.queue)):
                node = self.queue[i]
                metricEst = InfinityMetric(node, goal)
                metricPot = InfinityMetric(node, goal)

        if (node.cost + metricEst) < (self.queue[index].cost + metricPot):
            index = i
            
        next = self.queue[index]
        del self.queue[index]
        return next

# A* algorithm implementation where standard priority queue without weighting metric
# produces uniform cost search
def AStar(graph, start, goal, path, metricFlag):
    # triggers invalid if start or goal is a bricked out input
    if start.weight > 9999:
        print("INVALID START")
        return 0
    if goal.weight > 9999:
        print("INVALID GOAL")
        return 0
    
    # intantiates a visted list
    Visited = []
    # places all the bricked out locations on the map in visited
    for i in range(0, len(graph.nodes)-1):
        if graph.nodes[i].weight > 9999:
            Visited.append(graph.nodes[i].name)

    # instantiates the fringe as a priortity queue and places start in the queue
    # and in the visited list
    Fringe = PriorityQueue()
    Fringe.put(start)
    Visited.append(start.name)

    # while the fringe is not empy we run an algorithm that calls the get method
    # to pull the updated fringe according to the designated metric function.
    # then we append the current important item from the priority queue decided
    # by the metric into the path and into visited. if it's the goal, we won.
    # otherwise we keep looking by updating the fringe to include the neighbors
    # of the current location pulled off the queue.
    while not Fringe.isEmpty():
        if metricFlag == 1:
            current = Fringe.get()
        else:
            current = Fringe.getMetric(goal, metricFlag)

        Visited.append(current.name)
        path.append(current)

        if current.isGoal:
            print("Cost: "+str(current.cost))
            return 1
        else:
            for i in range(len(current.neighbors)):
                if current.neighbors[i].name not in Visited:
                    current.neighbors[i].setCost(current)
                    current.neighbors[i].parent = current
                    Fringe.put(current.neighbors[i])
                    Visited.append(current.neighbors[i].name)

# bidirectional search implementation running backwards from start and goal to find
# joining path
def Bidirectional(graph, start, goal, path):
    num = 0
    if start.weight > 9999:
        print("INVALID START")
        return 0
    if goal.weight > 9999:
        print("INVALID GOAL")
        return 0

    Visited = []
    for i in range(0, len(graph.nodes) - 1):
        if graph.nodes[i].weight > 9999:
            Visited.append(graph.nodes[i].name)

    Fringe1 = []
    Fringe1.append(start)
    Fringe2 = []
    Fringe2.append(goal)
    Visited.append(start.name)
    Visited.append(goal.name)
    path.append(start)
    path.append(goal)

    # bidirectional search works in a similar way to A* but
    # instead of searching along one avenue, it searches from the
    # start and goal for the point at which their searched areas
    # intersect. because of this we track the fringe of both and
    # once there is an overlap we use PopulatePath2 and PopulatePath3
    # to grab the established path pull from the union of the
    # set of elements searched by both fringes.
    while not len(Fringe1) == 0 and not len(Fringe2) == 0:
        current1 = Fringe1.pop()
        Visited.append(current1.name)
        path.append(current1)

        current2 = Fringe2.pop()
        Visited.append(current2.name)
        path.append(current2)

        Names = []

        for i in range(0, len(current1.neighbors), 1):
            if current1.neighbors[i].name not in Visited:
                Fringe1.append(current1.neighbors[i])
                current1.neighbors[i].parent = current1

        for i in range(len(current2.neighbors) - 1, -1, -1):
            if current2.neighbors[i].name not in Visited:
                Fringe2.append(current2.neighbors[i])
                current2.neighbors[i].parentAlt = current2
                Names.append(current2.neighbors[i].name)

        for i in range(0, len(Fringe1)):
            if Fringe1[i].name in Names:
                path.append(Fringe1[i])
                for j in range(0, len(graph.nodes)):
                    if Fringe1[i].name == graph.nodes[j].name:
                        num = j
                return num

    return num

# function to read a text file that outlines the search problem to be solved in the form of
# mazeSize
# startRow startCol
# goalRow goalCol
# algorithm
# mazeName
def ReadProblemText():
    file = open('mazes/problem.txt', 'r')
    list = []
    f = file.readlines()
    for line in f:
        list.append(line.strip())
    file.close()
    return list

# a function to read a maze from the attributed file of mazes
def ReadMazeText(int):
    string = str(int)
    newString = 'mazes/'+string+'.txt'
    file = open(newString, 'r')
    list = []
    f = file.readlines()
    for line in f:
        list.append(line.strip())
    file.close()
    return list

# three PopulatePath functions where the 2nd and 3rd are mechanisms for Bidirectional Search
def PopulatePath(list, goal, start):
    list.append(goal)
    try:
        if goal.name == start.name:
            list.append(goal)
            return
        else:
            PopulatePath(list, goal.getParent(), start)
    except:
        return

def PopulatePath2(list, goal, start, mid):
    list.append(mid)
    try:
        if start.name == mid.name:
            list.append(start)
            return
        else:
            PopulatePath(list, mid.getParent(), start)
    except:
        return

def PopulatePath3(list, goal, start, mid):
    list.append(mid)
    try:
        if goal.name == mid.name:
            list.append(goal)
            return
        else:
            PopulatePath3(list, goal, start, mid.getParentAlt())
    except:
        return

# function that generates a random graph with a random start and finish
# based on a taken user input for the probability by which obstacles appear
# on the maze
def RandomGraph(graph, prob, metricFlag):
    graph.generateRandomMaze(101, prob)
    
    start = random.randrange(0, len(graph.nodes)-1, 1)
    goal = random.randrange(0, len(graph.nodes)-1, 1)

    graph.nodes[start].isStart = True
    graph.nodes[goal].isGoal = True
    
    Path = []
    if metricFlag == 0:
        PlaceHolder = Bidirectional(graph, graph.nodes[start], graph.nodes[goal], Path)
    else:
        PlaceHolder = AStar(graph, graph.nodes[start], graph.nodes[goal], Path, metricFlag)

    if PlaceHolder == 0:
        return "NO IMAGE"
    else:
        return PrintGraph(graph, Path, start, goal, metricFlag, PlaceHolder)

# function for the user to call a sample of input instructions as given by mazes/problem.txt
def GivenGraph(graph):
    newList = ReadProblemText()

    list1 = newList[1].split()
    list2 = newList[2].split()
    n = int(newList[0])
    a, b, c, d = str(list1[0]), str(list1[1]), str(list2[0]), str(list2[1])
    metricFlag, maze = int(newList[3]), ReadMazeText(newList[4])
    graph.generateGivenMaze(maze, a, b, c, d, n)
    
    start = 0
    goal = 0
    for i in range(len(graph.nodes)):
        if graph.nodes[i].isStart:
            start = i
        if graph.nodes[i].isGoal:
            goal = i
    
    Path = []
    if metricFlag == 0:
        PlaceHolder = Bidirectional(graph, graph.nodes[start], graph.nodes[goal], Path)
    else:
        PlaceHolder = AStar(graph, graph.nodes[start], graph.nodes[goal], Path, metricFlag)

    if PlaceHolder == False:
        return "NO IMAGE"
    else:
        return PrintGraph(graph, Path, start, goal, metricFlag, PlaceHolder)

# function for the user to define precisely the specifications
# they desire for the graph input
def ChosenGraph(graph, a, b, c, d, metricFlag, num):
    n = 101
    maze = ReadMazeText(num)
    graph.generateGivenMaze(maze, a, b, c, d, n)
    
    start = 0
    goal = 0
    for i in range(len(graph.nodes)):
        if graph.nodes[i].isStart:
            start = i
        if graph.nodes[i].isGoal:
            goal = i
    
    Path = []
    if metricFlag == 0:
        PlaceHolder = Bidirectional(graph, graph.nodes[start], graph.nodes[goal], Path)
    else:
        PlaceHolder = AStar(graph, graph.nodes[start], graph.nodes[goal], Path, metricFlag)

    if PlaceHolder == False:
        return "NO IMAGE"
    else:
        return PrintGraph(graph, Path, start, goal, metricFlag, PlaceHolder)

# maze and path print out utilizing PopulatePath functions 1, 2, and 3 as designed above
def PrintGraph(graph, Path, start, goal, metricFlag, PlaceHolder):
        X, Y, A, B, PathX, PathY, MAINPATH, MAINPATHX, MAINPATHY, SGX, SGY = [], [], [], [], [], [], [], [], [], [], []
        for i in range(0, len(graph.nodes)):
            if graph.nodes[i].weight > 9999:
                A.append(graph.nodes[i].row)
                B.append(graph.nodes[i].column)

        for i in range(0, len(Path)):
            X.append(Path[i].row)
            Y.append(Path[i].column)

        if not metricFlag == 0:
            PopulatePath(MAINPATH, graph.nodes[goal], graph.nodes[start])
        else:
            PopulatePath2(MAINPATH, graph.nodes[goal], graph.nodes[start], graph.nodes[PlaceHolder])
            MAINPATH.reverse()
            PopulatePath3(MAINPATH, graph.nodes[goal], graph.nodes[start], graph.nodes[PlaceHolder])
            for i in range(0, len(MAINPATH) - 1):
                MAINPATH[i+1].setCost(MAINPATH[i])
            print("Cost is: "+str(MAINPATH[len(MAINPATH)-1].cost))
        
        try:
            for i in range(0, len(MAINPATH)):
                MAINPATHX.append(MAINPATH[i].row)
                MAINPATHY.append(MAINPATH[i].column)

            SGX.append(graph.nodes[start].row)
            SGX.append(graph.nodes[goal].row)
            SGY.append(graph.nodes[start].column)
            SGY.append(graph.nodes[goal].column)

            t = time.process_time()
            print("Time (in seconds):", t*1e-9)
            pl.scatter(X, Y, s=4, c='green')
            pl.scatter(A, B, s=10, c='black')
            pl.scatter(MAINPATHX, MAINPATHY, s=12, c='blue')
            pl.scatter(SGX, SGY, s=20, c='yellow')
            pl.show()
        except:
            return "NO IMAGE"


# step-by-step user input instructions in order to choose how to present and search mazes
runner = int(input("Enter 1 for graph built on specifications of mazes/problem.txt.\n"
    +"Enter 2 for randomly generated graph.\n"
    +"Enter 3 for graph built on parameters you choose.\n"))
if runner == 1:
    graph = Graph()
    GivenGraph(graph)
elif runner == 2:
    graph = Graph()
    prob = float(input("Enter a probability between 0 and 1 for probability of a wall.\n"))
    if (prob < 0) or (prob > 1):
        print("INVALID INPUT")
    else:
        metric = int(input("Choose a search:\n"
            +"0 for Bidrectional Search\n"
            +"1 for Uniform Cost Search\n"
            +"2 for A* Euclidean Matric\n"
            +"3 for A* Taxi Cab Metric\n"
            +"4 for A* Min(Euclidean, Taxi Cab)\n"
            +"5 for A* Max(Euclidean, Taxi Cab)\n"
            +"6 for A* Adjusted Taxi Cab Metric: 2x Column\n"
            +"7 for A* Adjusted Euclidean Metric: 2x Column\n"
            +"8 for A* Max(Adj Taxi Cab, Adj Euclidean)\n"
            +"9 for A* Chebyshev Metric\n"))
        if metric not in range(0,10):
            print("INVALID INPUT")
        else:
            RandomGraph(graph, prob, metric)
elif runner == 3:
    graph = Graph()
    print("To pick start and goal nodes provide four numbers bound by 0 and 100.")
    a = int(input("start column:\n"))
    b = int(input("start row:\n"))
    c = int(input("goal column:\n"))
    d = int(input("goal row:\n"))
    if (a < 0 or a > 100) or (b < 0 or b > 100) or (c < 0 or c > 100) or (d < 0 or d > 100):
        print("INVALID INPUT")
    else:
        metric = int(input("Choose a search:\n"
            +"0 for Bidrectional Search\n"
            +"1 for Uniform Cost Search\n"
            +"2 for A* Euclidean Matric\n"
            +"3 for A* Taxi Cab Metric\n"
            +"4 for A* Min(Euclidean, Taxi Cab)\n"
            +"5 for A* Max(Euclidean, Taxi Cab)\n"
            +"6 for A* Adjusted Taxi Cab Metric: 2x Column\n"
            +"7 for A* Adjusted Euclidean Metric: 2x Column\n"
            +"8 for A* Max(Adj Taxi Cab, Adj Euclidean)\n"
            +"9 for A* Chebyshev Metric\n"))
        if metric not in range(0,10):
            print("INVALID INPUT")
        else:
            map = str(input("Now choose a graph as a 3-digit number string including front zeroes that is bound by 000 and 100.\n"
                +"As an example: 003, 010, 034, 099, 100.\n"))
            if int(map) < 0 or int(map) > 100:
                print("INVALID INPUT")
            else:
                ChosenGraph(graph, a, b, c, d, metric, map)
else:
    print("INVALID INPUT")