import itertools
import astar
import numpy as np
import random
# Define waypoints and their coordinates
waypoints = {
    'Start': (0, 0),
    'Goal 1': (1, 1),
    'Goal 2': (4, 4),
    'Goal 3': (1, 9),
    'Goal 4': (9, 7),
    'Goal 5': (9, 4),
    'Goal 6': (6, 14),
    'Goal 7': (3, 11),
    'Goal 8': (14, 1),
    'Goal 9': (1, 14),
    'Goal 10': (14, 14),
    'Goal 11': (7, 10),
}
#setup for astar
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
    

#5790 2288

# Generate all possible permutations of waypoints
waypoint_names = list(waypoints.keys())
permutations = itertools.permutations(waypoint_names[3:], len(waypoint_names) -3)
# add start in it, I select goal 1 and goal 2 as start because they are closest to the starting point
permutations_with_start = [('Start','Goal 1','Goal 2',) + perm for perm in permutations]
# since there are too many cases , I will do a sample of 4000 cases
randomly_chosen_elements = random.sample(permutations_with_start, 4000)
# Initialize variables to keep track of the best path and minimum distance
best_path = None
min_cost = float('inf')
total_dist = 0 
total_wplist = []
all_cost = []
all_solution = []
# Iterate through permutations and calculate total distances
for perm in randomly_chosen_elements:
    total_dist = 0
    total_wplist = []
    for i in range(len(perm) - 1):
        
        Start_x = waypoints[perm[i]][0]
        Start_y = waypoints[perm[i]][1]
        Goal_x  = waypoints[perm[i+1]][0]
        Goal_y  = waypoints[perm[i+1]][1]
        # this is astar, I just named it dijkstra:)
        wplist , cost = astar.dijkstras(Start_x,Start_y,min_x,Max_x,min_y,Max_y,Goal_x,Goal_y,gs,Obstacle_list,obstacle_dia,robot_radius) 
        wplist = wplist[::-1]
        total_wplist = total_wplist + wplist 
        total_dist = total_dist + cost
        if total_dist >= min_cost:
          print("give up on this one")
          break
    
    if total_dist < min_cost:
     min_cost = total_dist
     best_path = perm
     best_wplist = total_wplist
     all_cost.append(str(total_dist))
     all_solution.append(perm)
     print("we found a better one")

    #print("let's go !")
    # total_distance = 0
    # for i in range(len(perm) - 1):
    #     total_distance += distances[(perm[i], perm[i + 1])]
    # if total_distance < min_distance:
    #     min_distance = total_distance
    #     best_path = ('Start',) + perm

# Print the best path and minimum distance
print("Best Path:", best_path)
print("Minimum Distance:", min_cost)
print("best wplist is :", best_wplist)

# Plot the optimal path
import matplotlib.pyplot as plt

x_values = [waypoints[wp][0] for wp in best_path]
y_values = [waypoints[wp][1] for wp in best_path]
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
values_list = list(waypoints.values())
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

# find the highest cost
max_cost=max(all_cost)
max_index = all_cost.index(max_cost)
# find that soultion
worst_plan = all_solution[max_index]

print("the worst plan is :",worst_plan,"it's cost is",max_cost)