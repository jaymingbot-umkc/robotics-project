import itertools

# Define waypoints and their coordinates
waypoints = {
    'Start': (0, 0),
    'Goal 1': (2, 2),
    'Goal 2': (5, 3),
    'Goal 3': (3, 4),
    'Goal 4': (6, 4),
}

# Create a lookup table for distances
distances = {}
for wp1, coord1 in waypoints.items():
    for wp2, coord2 in waypoints.items():
        if wp1 != wp2:
            distance = ((coord1[0] - coord2[0])**2 + (coord1[1] - coord2[1])**2) ** 0.5
            distances[(wp1, wp2)] = distance

# Generate all possible permutations of waypoints
waypoint_names = list(waypoints.keys())
permutations = itertools.permutations(waypoint_names[1:], len(waypoint_names) - 1)

# Initialize variables to keep track of the best path and minimum distance
best_path = None
min_distance = float('inf')

# Iterate through permutations and calculate total distances
for perm in permutations:
    total_distance = distances[('Start', perm[0])]
    for i in range(len(perm) - 1):
        total_distance += distances[(perm[i], perm[i + 1])]
    if total_distance < min_distance:
        min_distance = total_distance
        best_path = ('Start',) + perm

# Print the best path and minimum distance
print("Best Path:", best_path)
print("Minimum Distance:", min_distance)

# Plot the optimal path
import matplotlib.pyplot as plt

x_values = [waypoints[wp][0] for wp in best_path]
y_values = [waypoints[wp][1] for wp in best_path]

plt.plot(x_values, y_values, marker='o', linestyle='-')
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.title('Optimal TSP Path')
plt.grid(True)
plt.show()
