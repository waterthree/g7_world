import cv2
import numpy as np
import math
# Constants
MAP_SIZE = 470  # in cm
GRID_SIZE = int(MAP_SIZE / 10)  # in cm
OBSTACLE_SIZE = (GRID_SIZE, GRID_SIZE)  # in cm
GRID_ROWS = int(MAP_SIZE / GRID_SIZE)
GRID_COLS = int(MAP_SIZE / GRID_SIZE)

# Function to draw grid lines on the map
def draw_grid(image):
    color = (100, 100, 100)  # Grey color
    for x in range(0, image.shape[1], GRID_SIZE):
        cv2.line(image, (x, 0), (x, image.shape[0]), color, 1)
    for y in range(0, image.shape[0], GRID_SIZE):
        cv2.line(image, (0, y), (image.shape[1], y), color, 1)

# Function to add obstacles to the map based on cell indices
def add_obstacles(image, obstacles):
    for obstacle in obstacles:
        row, col = obstacle
        x1 = col * GRID_SIZE
        y1 = row * GRID_SIZE
        x2 = x1 + OBSTACLE_SIZE[1]
        y2 = y1 + OBSTACLE_SIZE[0]
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 0, 200), -1)  # Blue color for obstacles

# Function to add robot to the map based on cell index
def add_robot(image, position, direction_degrees=0):
    row, col = position
    center_x = int((col + 0.5) * GRID_SIZE)
    center_y = int((row + 0.5) * GRID_SIZE)
    radius = int(GRID_SIZE / 3)
    cv2.circle(image, (center_x, center_y), radius, (0, 0, 0), -1)  # Black color for robot
    # Draw a white line indicating direction
    line_length = int(radius * 0.8)
    angle_radians = math.radians(direction_degrees)
    end_x = int(center_x + line_length * math.cos(angle_radians))
    end_y = int(center_y - line_length * math.sin(angle_radians))  # Negative because the y-axis is inverted in images
    cv2.line(image, (center_x, center_y), (end_x, end_y), (255, 255, 255), 2)

# Function to add goal position to the map based on cell index
def add_goal(image, position):
    row, col = position
    center_x = int((col + 0.5) * GRID_SIZE)
    center_y = int((row + 0.5) * GRID_SIZE)
    radius = int(GRID_SIZE / 4)
    cv2.circle(image, (center_x, center_y), radius, (0, 255, 0), -1)  # Green color for goal

# Main function
def main():
    # Create a grey flooring map
    map_image = np.ones((MAP_SIZE, MAP_SIZE, 3), dtype=np.uint8) * 200  # Grey color

    # Draw grid lines
    draw_grid(map_image)

    # Define obstacle positions based on cell indices
    obstacle_positions = [(1, 1), (0, 9), (4, 1), (4, 7), (8, 3), (8, 6)]

    # Add obstacles to the map
    add_obstacles(map_image, obstacle_positions)

    # Add robot to the map
    robot_position = (4, 5)
    add_robot(map_image, robot_position, direction_degrees=90)

    # Add goal position to the map
    goal_position = (8, 8)
    add_goal(map_image, goal_position)

    # Display the map
    cv2.imshow('Gridworld: Turtlebot3 with Obstacles', map_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
