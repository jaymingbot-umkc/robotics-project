
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import astar
# Load the CSV file into a DataFrame
csv_file = '/home/zhimbot/ros2_ws/src/unmanned_systems_ros2_pkg/unmanned_systems_ros2_pkg/log/Dec_04_16_59.csv'
data = pd.read_csv(csv_file)

# Extract columns
time = data['time'].to_numpy()
x_position = data['x'].to_numpy()
y_position = data['y'].to_numpy()
evader_x = data['evader_x'].to_numpy()
evader_y = data['evader_y'].to_numpy()
# get the desired trajectory 
Start_x = 2
Start_y = 1
Goal_x = 7
Goal_y = 2
Max_x = 8
Max_y = 8
min_x=0
min_y=0

gs=1
obstacle_dia = 0.5
robot_radius = 0.5
Obstacle_list=[[5,0],[5,1],[5,2],[5,3],[5,4],[0,5],[1,4],[2,3],[3,2],[3,3]] 
wp_list=astar.dijkstras(Start_x,Start_y,min_x,Max_x,min_y,Max_y,Goal_x,Goal_y,gs,Obstacle_list,obstacle_dia,robot_radius)


    # plt.figure(1)
    # plt.xlim(0, 15)
    # plt.ylim(0, 15)

x_waypoint=[]
y_waypoint=[]
for wp in wp_list:
    x_waypoint.append(wp[0])
    y_waypoint.append(wp[1])

# Create subplots
fig, axs = plt.subplots(2, 2, figsize=(10, 10))
fig.suptitle('Data of x vs y')

# Subplot 1: X Position vs. Time
axs[0, 0].plot(x_position, y_position)
axs[0, 0].plot(x_waypoint,y_waypoint,color = 'red')
axs[0, 0].set_title('pursuer x vs y')
axs[0, 0].set_xlabel('X position')
axs[0, 0].set_ylabel('Y Position')

# Subplot 2: Y Position vs. Time
axs[0, 1].plot(evader_x, evader_y)
axs[0, 1].plot(x_waypoint,y_waypoint,color = 'red')
axs[0, 1].set_title('evader x vs y')
axs[0, 1].set_xlabel('X position')
axs[0, 1].set_ylabel('Y Position')

# Subplot 3: Velocity vs. Time
axs[1, 0].plot(x_position, y_position)
axs[1, 0].plot(evader_x, evader_y)
axs[1, 0].set_title('both together')
axs[1, 0].set_xlabel('X position')
axs[1, 0].set_ylabel('Y Position')

#Subplot 4: Angular Velocity vs. Time
axs[1, 1].plot(time, x_position)
axs[1, 1].set_title('time used')
axs[1, 1].set_xlabel('Time')
axs[1, 1].set_ylabel('x_position of pursuer')

# Adjust layout for better visualization
plt.tight_layout(rect=[0, 0.03, 1, 0.95])

# Show the plots
plt.show()

