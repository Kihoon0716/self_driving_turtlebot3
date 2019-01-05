import heapq
import numpy as np
import math
import timeit
import matplotlib.pyplot as plt

def distance_dot2dot(x1, y1, x2, y2):
    distance = math.sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2-y1))
    return distance
class Node():
    def __init__(self, row, col, parent, heuristic):
        self.row = row
        self.col = col
        self.parent = parent
        self.heuristic = heuristic
    def __cmp__(self, other):
        return cmp(self.heuristic, other.heuristic)
    def show(self):
        print "row : ", self.row
        print "col : ", self.col
        print "row of parent : ", self.parent.row
        print "col of parent : ", self.parent.col
        print "heuristic : ", self.heuristic
        print "---------"
class PQ():
    def __init__(self):
        self.heap = []
        self.size_ = 0
    def push(self, data):
        heapq.heappush(self.heap, data)
        self.size_ += 1
    def pop(self):
        self.size_ -= 1
        return heapq.heappop(self.heap)
    def front(self):
        return self.heap[0]
    def size(self):
        return self.size_
    def empty(self):
        return self.size_ == 0

class Solver():
    def __init__(self, map, size_row, size_col, row_init, col_init, row_goal, col_goal):
        self.map = map
        self.visit = np.zeros((size_row, size_col))
        self.size_row = size_row
        self.size_col = size_col
        self.row_init = row_init
        self.col_init = col_init
        self.row_goal = row_goal
        self.col_goal = col_goal
        self.path = [] 
        self.parent = np.zeros((self.size_row, self.size_col, 2))
        
    def calculateHeuristic(self, row, col):
        return math.sqrt(math.pow(self.row_goal - row, 2) + pow(self.col_goal - col, 2))
    def solve(self):
        start = timeit.default_timer()
        q = PQ()
        newNode = Node(self.row_init, self.col_init, None, distance_dot2dot(self.row_init, self.col_init, self.row_goal, self.col_goal))
        self.visit[self.row_init][self.col_init] = 1
        q.push(newNode)
        while q.size() > 0:
            node = q.pop()
            row_now = node.row
            col_now = node.col
            if(row_now == self.row_goal and col_now == self.col_goal):# found goal
                # print "found!", timeit.default_timer() - start
                return True
            for i in range(-1, 2):
                for j in range(-1, 2):
                    # when position_now equal to position_next
                    if i == 0 and j == 0: 
                        continue
                    row_next = row_now + i
                    col_next = col_now + j
                    # outside of the map
                    if row_next < 0 or col_next < 0 or row_next >= self.size_row or col_next >= self.size_col: 
                        continue
                    # when visited position_next
                    if self.visit[row_next][col_next] != 0:
                        continue
                    # there is a obstacle in this position
                    if self.map[row_next][col_next] == 1:
                        continue
                    self.parent[row_next][col_next] = [row_now, col_now]
                    #print "row_next : ", row_next, "col_next : ", col_next, "row_now : ", row_now, "col_now : ", col_now
                    h = distance_dot2dot(row_next, col_next, self.row_goal, self.col_goal) + distance_dot2dot(0, 0, i, j) + node.heuristic
                    nextNode = Node(row_next, col_next, node, h)
                    self.visit[row_next][col_next] = 1
                    q.push(nextNode)
                    
        #print "path not exist!"
        return False # there is no path to goal
    def getPath(self):
        point = [self.row_goal,self.col_goal]
        self.path.append(point)
        while True:
            #print "point : ", point
            point = self.parent[int(point[0])][int(point[1])]
            self.path.append(point)
            if point[0] == self.row_init and point[1] == self.col_init:
                break
        

        for i in range(len(self.path)):
            self.map[int(self.path[i][0])][int(self.path[i][1])] = 2

        # draw plot

        # x1 = []
        # y1 = []
        # x2 = []
        # y2 = []
        # x_start = []
        # y_start = []
        # x_goal = []
        # y_goal = []
        # x_start.append(self.row_init)
        # y_start.append(self.col_init)
        # x_goal.append(self.row_goal)
        # y_goal.append(self.col_goal)

        # for i in range(self.row_init - 60, self.row_init + 60):
        #     for j in range(self.col_init - 60, self.col_init + 60):
        #         if self.map[i][j] == 1:
        #             x1.append(i)
        #             y1.append(j)
        #         elif self.map[i][j] == 2:
        #             x2.append(i)
        #             y2.append(j)

        # plt.plot(x1,y1,'ro')
        # plt.plot(x2,y2,'bo')
        # plt.plot(x_start,y_start,'rx')
        # plt.plot(x_goal,y_goal,'bx')
        # plt.show()
        
        return self.path


# rowsize = 40
# colsize = 40
# map = np.zeros((rowsize, colsize))

# for i in range(0, rowsize/2 + 10):
#     map[i][rowsize/2] = 1
#     map[i][rowsize/2 + 1] = 1
#     map[i][rowsize/2 + 2] = 1

# solver = Solver(map, rowsize, colsize, rowsize-1, 0, 0, colsize-1)
# solver.solve()
# path = solver.getPath()

# for i in range(len(path)):
#     map[int(path[i][0])][int(path[i][1])] = 2

# # draw plot
# x1 =[]
# y1 =[]
# x2 =[]
# y2 =[]

# for i in range(rowsize):
#     for j in range(colsize):
#         if map[i][j] == 1:
#             x1.append(i)
#             y1.append(j)
#         elif map[i][j] == 2:
#             x2.append(i)
#             y2.append(j)

# plt.plot(x1,y1,'ro')
# plt.plot(x2,y2,'bo')
# plt.show()