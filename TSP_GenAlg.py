#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Apr 17 13:17:33 2021

@author: fieldstd
"""

import time
import numpy as np
import matplotlib.pyplot as plt
import astar
start_time=time.time()
#%matplotlib inline
#%config InlineBackend.figure_format = 'svg'
plt.style.use("seaborn")
np.random.seed(42)
waypoints = [
    (0, 0),
    (1, 1),
    (4, 4),
    (1, 9),
    (9, 7),
    (9, 4),
    (6, 14),
    (3, 11),
    (14, 1),
    (1, 14),
    (14, 14),
    (7, 10),
]
cities = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,11]
Max_x = 15
Max_y = 15
min_x=0
min_y=0
Obstacle_x = [2, 2, 2, 2, 0, 1, 2, 3, 4, 5, 5, 5, 5, 5, 8, 9, 10, 11, 12, 13, 8, 8, 8, 8,
8, 8, 8, 2, 3, 4, 5, 6, 9, 10, 11, 12, 15, 2, 2, 2, 2, 2, 2, 5, 5, 5, 5, 5,
6, 7, 8, 9, 10, 11, 12, 12, 12, 12, 12]
Obstacle_y = [2, 3, 4, 5, 5, 5, 5, 5, 5, 5, 2, 3, 4, 5, 2, 2, 2, 2, 2, 2, 3, 4, 5, 6,
7, 8, 9, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 8, 9, 10, 11, 12, 13, 9, 10, 11, 12, 13,
12, 12, 12, 12, 12, 12, 8, 9, 10, 11, 12]
gs=0.5
obstacle_dia = 0.5
robot_radius = 0.5
Obstacle_list=[]
    
for x,y in zip(Obstacle_x,Obstacle_y):
         obs=(x,y)
         Obstacle_list.append(obs)

# adjacency_mat = np.asarray(
#     [
#         [0.00, 28.02, 17.12, 27.46, 46.07],
#         [28.02, 0.00, 34.00, 25.55, 25.55],
#         [17.12, 34.00, 0.00, 18.03, 57.38],
#         [27.46, 25.55, 18.03, 0.00, 51.11],
#         [46.07, 25.55, 57.38, 51.11, 0.00],
#     ]
# )

num_cities = 12
adjacency_mat = np.zeros((num_cities, num_cities))
for i in range(num_cities):
    for j in range(num_cities):
        if i != j:
            # Replace this line with your own distance calculation logic
            wplist , cost = astar.dijkstras(waypoints[i][0],waypoints[i][1],min_x,Max_x,min_y,Max_y,waypoints[j][0],waypoints[j][1],gs,Obstacle_list,obstacle_dia,robot_radius) 
            adjacency_mat[i, j] = cost


class Population():
    def __init__(self, bag, adjacency_mat):
        self.bag = bag
        self.parents = []
        self.score = 0
        self.best = None
        self.adjacency_mat = adjacency_mat

def init_population(cities, adjacency_mat, n_population):
    return Population(
        np.asarray([np.random.permutation(cities) for _ in range(n_population)]), 
        adjacency_mat
    )

pop = init_population(cities, adjacency_mat, 5)

def fitness(self, chromosome):
    return sum(
        [
            self.adjacency_mat[chromosome[i], chromosome[i + 1]]
            for i in range(len(chromosome) - 1)
        ]
    )

Population.fitness = fitness

def evaluate(self):
    distances = np.asarray(
        [self.fitness(chromosome) for chromosome in self.bag]
    )
    self.score = np.min(distances)
    self.best = self.bag[distances.tolist().index(self.score)]
    self.parents.append(self.best)
    if False in (distances[0] == distances):
        distances = np.max(distances) - distances
    return distances / np.sum(distances)
    
Population.evaluate = evaluate

def select(self, k=4):
    fit = self.evaluate()
    while len(self.parents) < k:
        idx = np.random.randint(0, len(fit))
        if fit[idx] > np.random.rand():
            self.parents.append(self.bag[idx])
    self.parents = np.asarray(self.parents)

Population.select = select

def swap(chromosome):
    a, b = np.random.choice(len(chromosome), 2)
    chromosome[a], chromosome[b] = (
        chromosome[b],
        chromosome[a],
    )
    return chromosome

def crossover(self, p_cross=0.1):
    children = []
    count, size = self.parents.shape
    for _ in range(len(self.bag)):
        if np.random.rand() > p_cross:
            children.append(
                list(self.parents[np.random.randint(count, size=1)[0]])
            )
        else:
            parent1, parent2 = self.parents[
                np.random.randint(count, size=2), :
            ]
            idx = np.random.choice(range(size), size=2, replace=False)
            start, end = min(idx), max(idx)
            child = [None] * size
            for i in range(start, end + 1, 1):
                child[i] = parent1[i]
            pointer = 0
            for i in range(size):
                if child[i] is None:
                    while parent2[pointer] in child:
                        pointer += 1
                    child[i] = parent2[pointer]
            children.append(child)
    return children

Population.crossover = crossover

def mutate(self, p_cross=0.1, p_mut=0.1):
    next_bag = []
    children = self.crossover(p_cross)
    for child in children:
        if np.random.rand() < p_mut:
            next_bag.append(swap(child))
        else:
            next_bag.append(child)
    return next_bag
    
Population.mutate = mutate

def genetic_algorithm(
    cities,
    adjacency_mat,
    n_population=500,
    n_iter=2000,
    selectivity=0.15,
    p_cross=0.5,
    p_mut=0.1,
    print_interval=100,
    return_history=False,
    verbose=False,
):
    pop = init_population(cities, adjacency_mat, n_population)
    best = pop.best
    score = float("inf")
    history = []
    for i in range(n_iter):
        pop.select(n_population * selectivity)
        history.append(pop.score)
        if verbose:
            print(f"Generation {i}: {pop.score}")
        elif i % print_interval == 0:
            print(f"Generation {i}: {pop.score}")
        if pop.score < score:
            best = pop.best
            score = pop.score
        children = pop.mutate(p_cross, p_mut)
        pop = Population(children, pop.adjacency_mat)
    if return_history:
        return best, history
    return best

best=genetic_algorithm(cities, adjacency_mat, verbose=True)
print(best)

end_time = time.time()
# generate plot
path=[]
for i in best:
    path.append(waypoints[i])
best_wplist = []
for i in range(len(path) -1) :
    Start_x = path[i][0]
    Start_y = path[i][1]
    Goal_x  = path[i+1][0]
    Goal_y  = path[i+1][1]
    wplist , cost = astar.dijkstras(Start_x,Start_y,min_x,Max_x,min_y,Max_y,Goal_x,Goal_y,gs,Obstacle_list,obstacle_dia,robot_radius) 
    wplist = wplist[::-1]
    best_wplist = best_wplist+wplist
    


x_values = [waypoints[wp][0] for wp in best]
y_values = [waypoints[wp][1] for wp in best]
plt.figure(1)
plt.plot(x_values, y_values, marker='o', linestyle='-')
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.title('Optimal TSP Path')
plt.grid(True)
plt.show()

plt.figure(2)
x=np.arange(0,Max_x+gs,gs)
y=np.arange(0,Max_y+gs,gs)
plt.xlim(0, 15)
plt.ylim(0, 15)
def compute_index(min_x,max_x,min_y,max_y,curr_x,curr_y,gs):
        index=((curr_x-min_x)/gs)+((max_x+gs-min_x)/gs)*((curr_y-min_y)/gs)
        return index 
values_list = waypoints
for i in range(0,len(x)):
    for j in range(0,len(y)):
        if(x[i],y[j]) in values_list:
          plt.scatter(x[i],y[j], color = 'blue')
        if (x[i],y[j]) in Obstacle_list:
          plt.text(x[i], y[j], str(int(compute_index(min_x,Max_x,min_y,Max_y,x[i],y[j],gs))),color="red")
          plt.scatter(x[i],y[j],color = 'red')
        else:
          plt.text(x[i], y[j], str(int(compute_index(min_x,Max_x,min_y,Max_y,x[i],y[j],gs))),color="red")
    x_waypoint=[]
    y_waypoint=[]
    for wp in best_wplist:
          x_waypoint.append(wp[0])
          y_waypoint.append(wp[1])

plt.plot(x_waypoint,y_waypoint)   
plt.show()
print("time used is :",end_time - start_time)