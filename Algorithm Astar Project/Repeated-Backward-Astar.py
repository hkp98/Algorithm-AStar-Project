
# Google colab link-> https://colab.research.google.com/drive/14vLmaGcGi5-yVZx281g4QFarpBAczsdY?usp=sharing

from HeapPriorityQueue import HeapPriorityQueue
import time
import random as RandomValue
import matplotlib.pyplot as plt
import seaborn as sns
import copy
import numpy as np


def generate_maze(dim=101, p=0.2):
    rows = dim
    cols = dim
    mat = np.ones((dim, dim))  # a maze with all free cells is generated

    for i in range(rows):
        for j in range(cols):
            if (RandomValue.uniform(1,
                                    0) < p):  # if random number is less than p, an obstacle is added to the cell (i, j)
                mat[i][j] = 0

    mat[0][0] = 1  # start needs to be unblocked
    mat[dim - 1][dim - 1] = 1  # goal needs to be unblocked
    # print(mat)
    return mat


def visualise_maze(maze):  # visualize the initial maze
    if maze.shape[0] < 25:
        x = 5
    elif maze.shape[0] > 25 and maze.shape[0] < 75:
        x = 10
    else:
        x = 20

    plt.figure(figsize=(x, x))
    ax = sns.heatmap(maze, linewidths=0.1, linecolor="white", square=True, cbar=False, xticklabels=False,
                     yticklabels=False, cmap=sns.diverging_palette(20, 220, n=200))
    plt.show()


def visualise_path(maze, var, start, goal):  # returns the intermediate paths for each Astar call
    maze3 = copy.deepcopy(maze)
    dim = maze3.shape[0]
    li = []
    key = goal
    while True:
        li.append(key)
        value = var.get(key)
        key = value
        if key == start:
            li.append(key)
            break

    for k in range(0, len(li)):
        x, y = li[k]
        maze3[x][y] = 2

    # plt.figure(figsize = (5,5))
    # ax = sns.heatmap(maze3, linewidths=0.1,linecolor= "white", square = True, cbar=False, xticklabels=False, yticklabels=False,cmap=sns.diverging_palette(20, 120, n=200), vmin= 0 , vmax = 2)
    # plt.show()
    return li


def my_final_visualizer(maze, var, start, goal):  # prints the final path from source to destination
    maze4 = copy.deepcopy(maze)
    ln = var
    for k in range(0, len(ln)):
        x, y = ln[k]
        maze4[x][y] = 2
    if maze.shape[0] < 25:
        x = 5
    elif maze.shape[0] > 25 and maze.shape[0] < 75:
        x = 10
    else:
        x = 20

    plt.figure(figsize=(x, x))
    # plt.figure(figsize=(5, 5))
    ax = sns.heatmap(maze4, linewidths=0.1, linecolor="white", square=True, cbar=False, xticklabels=False,
                     yticklabels=False, cmap=sns.diverging_palette(20, 220, n=200), vmin=0, vmax=2)
    plt.show()


def all_actions(var, maze):  # returns the neighbours for current cell and blocked cells
    actions = ()
    block_list = []
    i, j = var
    dim = maze.shape[0]

    if (j + 1 < dim) and ((maze[i][j + 1] == 0) and (not ((i, j + 1) in block_list))):
        block_list.append((i, j + 1))
    if j - 1 >= 0 and ((maze[i][j - 1] == 0) and (not ((i, j - 1) in block_list))):
        block_list.append((i, j - 1))
    if (i + 1 < dim) and ((maze[i + 1][j] == 0) and (not ((i + 1, j) in block_list))):
        block_list.append((i + 1, j))
    if i - 1 >= 0 and ((maze[i - 1][j] == 0) and (not ((i - 1, j) in block_list))):
        block_list.append((i - 1, j))

    if (j + 1 < dim) and (maze[i][j + 1] != 0):
        actions = actions + ((i, j + 1),)
    if j - 1 >= 0 and (maze[i][j - 1] != 0):
        actions = actions + ((i, j - 1),)
    if i + 1 < dim and (maze[i + 1][j] != 0):
        actions = actions + ((i + 1, j),)
    if i - 1 >= 0 and (maze[i - 1][j] != 0):
        actions = actions + ((i - 1, j),)

    return actions, block_list


print("Enter the maze size ")
alpha = int(input())
maze111 = generate_maze(alpha + 1, 0.3)
Main_maze = maze111


def Compute_Path_Backward(search, open_list, closed_list, g, f, h, maze,
                          counter):  # functions that perform the Astar Algorithm
    search_path = {}
    ct = 0
    while (not open_list.is_empty()) and (g[0][0] > open_list.min()):
        ct = ct + 1
        var = open_list.remove_min()
        a, b = var
        closed_list.append(var)
        actions = ()
        actions, block_list = all_actions(var, maze)

        for i in range(0, len(actions)):
            x, y = actions[i]
            if search[x][y] < counter:
                g[x][y] = 1000
                search[x][y] = counter

            if g[x][y] > g[a][b] + 1:
                g[x][y] = g[a][b] + 1
                search_path[(x, y)] = (a, b)  # tree
                f[x][y] = g[x][y] + h[x][y]
                p, q = open_list.search_Queue(f[x][y], (x, y))
                if not open_list.is_empty() and (x, y) == q:
                    open_list.remove_index(p)
                open_list.add(f[x][y], (x, y))
    return search_path


def astar_backward():  # function for calling the astar Algorithm and making repeated Astar calls
    start_time = time.time()
    maze = copy.deepcopy(Main_maze)
    mazer = np.ones((alpha + 1, alpha + 1))
    maze_temp = maze * 100  # maze_temp is used for visualisation
    visualise_maze(maze)
    counter = 0
    dim = maze.shape[0]
    search = np.zeros((dim, dim))
    my_visitors = []
    h = np.zeros((dim, dim))
    g = np.zeros((dim, dim))
    f = np.zeros((dim, dim))
    for i in range(0, dim):
        for j in range(0, dim):
            h[i][j] = (abs(0 - i) + abs(0 - j))

    open_list = HeapPriorityQueue()
    start = (alpha, alpha)
    goal = (0, 0)
    p, q = goal
    while start != goal:
        g = np.zeros((dim, dim))
        f = np.zeros((dim, dim))
        c, d = start
        g[c][d] = 0
        counter = counter + 1
        search[c][d] = counter
        g[p][q] = 1000
        search[p][q] = counter
        open_list.remove_all()
        closed_list = []
        f[c][d] = g[c][d] + h[c][d]
        open_list.add(f[c][d], (c, d))
        var = Compute_Path_Backward(search, open_list, closed_list, g, f, h, mazer, counter)
        if open_list.is_empty():
            print("Path doesnot exist ")
            break;
        nodes = visualise_path(mazer, var, start, goal)
        my_visiters21 = []
        for i in range(len(nodes) - 1, -1, -1):
            x, y = nodes[i]
            actions1 = ()
            found = False
            actions1, block_list1 = all_actions((x, y), maze)
            for k in block_list1:
                a, b = k
                mazer[a][b] = 0
                if (a, b) in nodes:
                    found = True
            if found:
                break
            my_visitors.append((x, y))
        start = (x, y)

    print("Counter Backward Astar ", counter)
    print("Expanded Cells Backward Astar ", len(my_visitors))
    my_final_visualizer(mazer, my_visitors, start, goal)
    end_time = time.time()
    print("Total_Time  ", end_time - start_time)
    open_list.remove_all()


astar_backward()
