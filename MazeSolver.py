import cv2
import numpy as np
from collections import deque

def transform_waypoints(waypoints, homography):
    """
    Transforms a list of waypoints from image to real-world coordinates.

    Parameters:
    - waypoints: List of tuples [(x1, y1), (x2, y2), ...]
    - homography: 3x3 transformation matrix

    Returns:
    - transformed_waypoints: List of tuples [(x1_real, y1_real), (x2_real, y2_real), ...]
    """
    transformed_waypoints = []
    for point in waypoints:
        img_point = np.array([ [point[0], point[1]] ], dtype='float32')
        img_point = np.array([img_point])
        real_point = cv2.perspectiveTransform(img_point, homography)
        real_x, real_y = real_point[0][0]
        transformed_waypoints.append((real_x, real_y))
    return transformed_waypoints

# Configuration Parameters
maze_image_path = 'Maze/1.png'  # Updated to use captured image
output_path = 'Solved/Solved_Maze_Centered_Path.png'
grid_size = 40  # Define the grid resolution (e.g., 40x40)
cell_size = 10  # Size of each cell in pixels
margin = 20     # Margin around the maze in pixels

# Load and preprocess the maze image
maze = cv2.imread(maze_image_path, cv2.IMREAD_GRAYSCALE)
if maze is None:
    print(f"Failed to load maze image from {maze_image_path}")
    exit()

# Resize the maze to the defined grid size
maze = cv2.resize(maze, (grid_size, grid_size), interpolation=cv2.INTER_NEAREST)

# Threshold to binary
_, binary_maze = cv2.threshold(maze, 127, 255, cv2.THRESH_BINARY)

# Create binary grid
grid = (binary_maze // 255).astype(int)

# Detect start and end points
start = None
end = None
for i in range(len(grid)):
    for j in range(len(grid[0])):
        if grid[i][j] == 1:
            if start is None:
                start = (i, j)
            else:
                end = (i, j)

if start is None or end is None:
    print("Start or end point not found in the grid!")
    exit()

# BFS to find shortest path
def bfs(grid, start, end):
    queue = deque([start])
    visited = set()
    visited.add(start)
    parent = {start: None}

    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, down, left, right

    while queue:
        current = queue.popleft()
        if current == end:
            break

        for d in directions:
            neighbor = (current[0] + d[0], current[1] + d[1])
            if (
                0 <= neighbor[0] < len(grid)
                and 0 <= neighbor[1] < len(grid[0])
                and grid[neighbor[0]][neighbor[1]] == 1
                and neighbor not in visited
            ):
                queue.append(neighbor)
                visited.add(neighbor)
                parent[neighbor] = current

    # Reconstruct path if endpoint was reached
    if end not in parent:
        print("Endpoint is not reachable!")
        return []

    path = []
    current = end
    while current:
        path.append(current)
        current = parent[current]
    path.reverse()
    return path

# Solve the maze
path = bfs(grid, start, end)

if not path:
    print("No path found!")
    exit()

# Prepare visualization
visual_size = grid_size * cell_size + 2 * margin
solved_maze = np.zeros((visual_size, visual_size, 3), dtype=np.uint8)

# Draw maze background
for i in range(len(grid)):
    for j in range(len(grid[0])):
        color = (255, 255, 255) if grid[i, j] == 1 else (0, 0, 0)
        top_left = (j * cell_size + margin, i * cell_size + margin)
        bottom_right = ((j + 1) * cell_size + margin, (i + 1) * cell_size + margin)
        cv2.rectangle(
            solved_maze,
            top_left,
            bottom_right,
            color,
            -1,
        )

# Function to calculate center coordinates
def get_center(i, j):
    center_x = j * cell_size + cell_size // 2 + margin
    center_y = i * cell_size + cell_size // 2 + margin
    return (center_x, center_y)

# Generate waypoints and draw path with centered lines
turn_waypoints = []
for idx in range(1, len(path) - 1):
    prev = path[idx - 1]
    current = path[idx]
    next_point = path[idx + 1]

    # Calculate direction vectors
    prev_dir = (current[0] - prev[0], current[1] - prev[1])
    next_dir = (next_point[0] - current[0], next_point[1] - current[1])

    # Check for direction change (turn)
    if prev_dir != next_dir:
        # Calculate center of current cell for turn waypoint
        turn_waypoint = get_center(current[0], current[1])
        turn_waypoints.append(turn_waypoint)

    # Draw line between cell centers
    current_center = get_center(current[0], current[1])
    next_center = get_center(next_point[0], next_point[1])
    cv2.line(solved_maze, current_center, next_center, (0, 0, 255), 2, cv2.LINE_AA)

# Add the start and end waypoints explicitly
start_waypoint = get_center(start[0], start[1])
end_waypoint = get_center(end[0], end[1])
turn_waypoints.insert(0, start_waypoint)
turn_waypoints.append(end_waypoint)

# Draw waypoints on the maze
for waypoint in turn_waypoints:
    cv2.circle(solved_maze, waypoint, cell_size // 3, (255, 0, 0), -1)

# Save the updated maze visualization
cv2.imwrite(output_path, solved_maze)
print(f"Solved maze with centered path saved to {output_path}")
print("Turn waypoints (Image Coordinates):", turn_waypoints)

# Define real-world coordinates of the four corners
# Replace these with your actual real-world coordinates
# Example:
# Top-Left: (0, 0)
# Top-Right: (10, 0)
# Bottom-Left: (0, 10)
# Bottom-Right: (10, 10)
real_world_corners = np.float32([
    [0, 0],    # Top-Left
    [10, 0],   # Top-Right
    [0, 10],   # Bottom-Left
    [10, 10]   # Bottom-Right
])

# Define image coordinates of the four corners
image_corners = np.float32([
    [margin, margin],  # Top-Left
    [visual_size - margin, margin],  # Top-Right
    [margin, visual_size - margin],  # Bottom-Left
    [visual_size - margin, visual_size - margin]  # Bottom-Right
])

# Compute the homography matrix
homography_matrix = cv2.getPerspectiveTransform(image_corners, real_world_corners)

# Transform waypoints to real-world coordinates
real_world_waypoints = transform_waypoints(turn_waypoints, homography_matrix)

print("Turn waypoints (Real-World Coordinates):", real_world_waypoints)

# Optionally, save real-world waypoints to a file
with open('Solved/Real_World_Waypoints.txt', 'w') as f:
    for idx, waypoint in enumerate(real_world_waypoints):
        f.write(f"Waypoint {idx + 1}: {waypoint}\n")

print("Real-world waypoints saved to 'Solved/Real_World_Waypoints.txt'")

# Optionally, display the solved maze
# cv2.imshow("Solved Maze", solved_maze)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
