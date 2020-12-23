
# Google colab link -> https://colab.research.google.com/drive/1N5tY8tKnWXL8DC-xVjYVSqsUCwACjyld?usp=sharing

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


def Compute_Path_Adaptive(search, open_list, closed_list, g, f, h, maze,
                          counter):  # functions that perform the Astar Algorithm
    search_path = {}
    ct = 0
    while (not open_list.is_empty()) and (g[alpha][alpha] > open_list.min()):
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


def astar_adaptive():
    start_time = time.time()
    maze9 = copy.deepcopy(Main_maze)
    mazer2 = np.ones((alpha + 1, alpha + 1))
    maze_temp = maze9 * 100  # maze_temp is used for visualisation
    visualise_maze(maze9)
    counter1 = 0
    dim = maze9.shape[0]
    search1 = np.zeros((dim, dim))
    my_visitors1 = []
    h1 = np.zeros((dim, dim))
    g1 = np.zeros((dim, dim))
    f1 = np.zeros((dim, dim))
    for i in range(0, dim):
        for j in range(0, dim):
            h1[i][j] = (abs(alpha - i) + abs(alpha - j))

    open_list1 = HeapPriorityQueue()
    start1 = (0, 0)
    goal1 = (alpha, alpha)
    p, q = goal1
    while start1 != goal1:
        g1 = np.zeros((dim, dim))
        f1 = np.zeros((dim, dim))
        c, d = start1
        g1[c][d] = 0
        counter1 = counter1 + 1
        search1[c][d] = counter1
        g1[p][q] = 1000
        search1[p][q] = counter1
        open_list1.remove_all()
        closed_list1 = []
        f1[c][d] = g1[c][d] + h1[c][d]
        open_list1.add(f1[c][d], (c, d))
        var1 = Compute_Path_Adaptive(search1, open_list1, closed_list1, g1, f1, h1, mazer2, counter1)
        if open_list1.is_empty():
            print("Path doesnot exist ")
            break;
        nodes1 = visualise_path(mazer2, var1, start1, goal1)
        for i in range(len(nodes1) - 1, -1, -1):
            x, y = nodes1[i]
            actions2 = ()
            found = False
            actions3, block_list2 = all_actions((x, y), maze9)
            for k in block_list2:
                a, b = k
                mazer2[a][b] = 0
                if (a, b) in nodes1:
                    found = True
            if found:
                break
            my_visitors1.append((x, y))
            for ad in my_visitors1:
                n, m = ad
                h1[n][m] = abs(g1[p][q] - g1[n][m])
        start1 = (x, y)
    print("Counter Adaptive-Astar ", counter1)
    print("Expanded Cells Adaptive-Astar ", len(my_visitors1))
    my_final_visualizer(mazer2, my_visitors1, start1, goal1)
    end_time = time.time()
    print("Total_Time Adaptive-Astar ", end_time - start_time)


astar_adaptive()
