import numpy as np
import math  as ma 
from tqdm import tqdm
import time

def dijkstras(start_x,start_y,min_x,max_x,min_y,max_y,goal_x,goal_y,gs,obstcale_list,obstacle_dia,robot_radius):
    class node:
        def __init__(self,x,y,cost,parent_index):
             self.x = x
             self.y = y
             self.cost=cost
             self.parent_index=parent_index

    # the index compute function
    def compute_index(min_x,max_x,min_y,max_y,curr_x,curr_y,gs):
        index=((curr_x-min_x)/gs)+((max_x+gs-min_x)/gs)*((curr_y-min_y)/gs)
        return index             
           
    def get_all_moves(current_x:float, current_y:float, gs:float) -> list:

        move_list = []

        change_in_x = [-gs, 0, gs]
        change_in_y = [-gs, 0, gs]
        
        for dx in change_in_x:
            for dy in change_in_y:
                x_next = current_x + dx
                y_next = current_y + dy
                
                if [x_next, y_next] == [current_x, current_y]:
                    continue
                
                move = [x_next, y_next]
                move_list.append(move)
                
        return move_list

    def is_valid(a:node,obstacle_list:list,obs_diameter:float,x_max:float,y_max:float,x_min:float,y_min:float,step:float,robot_radius:float):
        if a.x > x_max or a.x < x_min or a.y > y_max or a.y < y_min:
            return False
        
        if a.x % step !=0 or a.y % step !=0:
            return False

        for obs in obstacle_list:
                dist = np.sqrt((a.x-obs[0])**2+(a.y-obs[1])**2)
                if dist <= (obs_diameter/2 + robot_radius):
                    return False
                
        return True


     
     # - Make two dictionaries to store the visited and unvisited:
    unvisited = {}
    visited = {}

    curr_node=node(start_x,start_y,0,-1)
    current_idx=compute_index(min_x,max_x,min_y,max_y,start_x,start_y,gs)


    unvisited[current_idx] = curr_node

    while [curr_node.x, curr_node.y] != [goal_x, goal_y]:
     # find the lowerest cost in unvisited pile
          current_idx = min(unvisited, key=lambda x:unvisited[x].cost)

          # take this node out and put this node into visited node and start visit and change the current node into the visiting one
          curr_node = unvisited[current_idx]

          visited[current_idx]=curr_node


          del unvisited[current_idx]
          if [curr_node.x, curr_node.y] == [goal_x, goal_y]:
               print("YAY you found it 1 ")
               
               wp_node = curr_node
               wp_list = []
               wp_list.append([wp_node.x, wp_node.y])
               
               while wp_node.parent_index != -1:
                    next_idx = wp_node.parent_index
                    wp_node  = visited[next_idx]            
                    wp_list.append([wp_node.x, wp_node.y])
               break

          all_moves = get_all_moves(curr_node.x, curr_node.y, gs)

          # find the filtered_moves

          filtered_moves = []

          for move in all_moves:
               tempnode=node(move[0],move[1],0,-100)
               if (is_valid(tempnode,obstcale_list,obstacle_dia,max_x,max_y,min_x,min_y,gs,robot_radius)==True):
                    filtered_moves.append(move)
               else: continue   

          #loop through all filtered moves:
          for moves in filtered_moves:    
               new_index = compute_index(min_x, max_x, min_y, max_y,moves[0], moves[1],gs)
               new_cost  = curr_node.cost+ma.dist(moves,[curr_node.x,curr_node.y])+ma.dist(moves,[goal_x,goal_y])
               if new_index in visited:
                    continue
               if new_index in unvisited:
                    if new_cost<unvisited[new_index].cost:
                         unvisited[new_index].cost=new_cost
                         unvisited[new_index].parent_index=current_idx
                    continue
               # if the next node is not in visted and unvisited, make a new node
               new=node(moves[0],moves[1],new_cost,current_idx)
               unvisited[new_index] = new
     # do a plot to show everything
     # the grid information and obstacles

    x=np.arange(0,max_x+gs,gs)
    y=np.arange(0,max_y+gs,gs)

    #generate the pitcure
    import matplotlib.pyplot as plt
    plt.figure(1)
    plt.xlim(0, 15)
    plt.ylim(0, 15)
    for i in range(0,len(x)):
          for j in range(0,len(y)):
           if (x[i],y[j]) in obstcale_list:
            plt.text(x[i], y[j], str(int(compute_index(min_x,max_x,min_y,max_y,x[i],y[j],gs))), color="red")
            plt.scatter(x[i],y[j])
           else:
            plt.text(x[i], y[j], str(int(compute_index(min_x,max_x,min_y,max_y,x[i],y[j],gs))), color="red")
    x_waypoint=[]
    y_waypoint=[]
    for wp in wp_list:
          x_waypoint.append(wp[0])
          y_waypoint.append(wp[1])

    plt.plot(x_waypoint,y_waypoint)   
     

                

     
    plt.show() 
    return wp_list
    
Start_x = 1
Start_y = 1
Goal_x = 7
Goal_y = 13
Max_x = 15
Max_y = 15
min_x=0
min_y=0
Obstacle_x = [2, 2, 2, 2, 0, 1, 2, 3, 4, 5, 5, 5, 5, 5, 8, 9, 10, 11, 12, 13, 8, 8, 8,
8, 8, 8, 8, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12, 13, 14, 15, 2, 2, 2, 2, 2, 2, 5, 5, 5, 5, 5,
5, 5, 6, 7, 8, 9, 10, 11, 12, 12, 12, 12, 12]
Obstacle_y = [2, 3, 4, 5, 5, 5, 5, 5, 5, 5, 2, 3, 4, 5, 2, 2, 2, 2, 2, 2, 3, 4, 5, 6, 7,
8, 9, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 8, 9, 10, 11, 12, 13, 9, 10, 11, 12, 13,
14, 15, 12, 12, 12, 12, 12, 12, 8, 9, 10, 11, 12]
gs=1
obstacle_dia = 0.5
robot_radius = 0.5
Obstacle_list=[]
    
for x,y in zip(Obstacle_x,Obstacle_y):
    obs=(x,y)
    Obstacle_list.append(obs)
print(Obstacle_list) 
# keep track of the time
start_time = time.time()   
wp_list=dijkstras(Start_x,Start_y,min_x,Max_x,min_y,Max_y,Goal_x,Goal_y,gs,Obstacle_list,obstacle_dia,robot_radius)
end_time = time.time()
print("waypoints are ", wp_list)
                
def calculate_total_distance(coords):
    total_distance = 0
    for i in range(1, len(coords)):
        lat1, lon1 = coords[i - 1]
        lat2, lon2 = coords[i]
        distance = ((lat2 - lat1) ** 2 + (lon2 - lon1) ** 2) ** 0.5
        total_distance += distance
    return total_distance

total_cost = calculate_total_distance(wp_list)

print("the cost is ,",total_cost,"the time used is,",end_time-start_time)

         
        
          







         
        
          



