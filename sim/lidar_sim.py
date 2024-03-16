import pygame
import math
import sys
import numpy as np
from matplotlib import pyplot as plt
import math
from collections import deque

# Initialize Pygame
pygame.init()

# Screen dimensions
width, height = 600, 600
screen = pygame.display.set_mode((width, height))

# Colors
black = (0, 0, 0)
white = (255, 255, 255)

# Box (car) properties
box_pos = [width // 2, height // 2]
box_angle = 0
box_speed = 0
box_size = (40, 20)

# Game loop
running = True
clock = pygame.time.Clock()
steering = 0.0 
throttle = 0.0

rect1 = pygame.Rect(200, 150, 100, 50)  # (x, y, width, height)
rect2 = pygame.Rect(550, 300, 80, 120)
rect3 = pygame.Rect(400, 200, 200, 40)
rect4 = pygame.Rect(200, 400, 200, 40)

# border
rect5 = pygame.Rect(0, 0, width, 10)
rect6 = pygame.Rect(0, height-10, width, 10)
rect7 = pygame.Rect(0, 0, 10, height)
rect8 = pygame.Rect(width-10, 0, 10, height)

obstacles = [rect1, rect2, rect3, rect4, rect5, rect6, rect7, rect8]

global_occupancy_map = np.zeros((100,100))


def bresenham(start, end):
    x1, y1 = start
    x2, y2 = end
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1
    err = dx - dy

    while True:
        yield x1, y1  # Return points as a generator
        if x1 == x2 and y1 == y2: 
            break 
        e2 = 2 * err
        if e2 > -dy: 
            err -= dy
            x1 += sx
        if e2 < dx: 
            err += dx
            y1 += sy 

def calc_grid_map_config(ox, oy, xy_resolution):
    min_x = round(min(ox) - 1/2)
    min_y = round(min(oy) - 1/2)
    max_x = round(max(ox) + 1/2)
    max_y = round(max(oy) + 1/2)
    xw = int(round((max_x - min_x) / xy_resolution))
    yw = int(round((max_y - min_y) / xy_resolution))
    return min_x, min_y, max_x, max_y, xw, yw


def init_flood_fill(center_point, obstacle_points, xy_points, min_coord,
                    xy_resolution):

    center_x, center_y = center_point
    prev_ix, prev_iy = center_x - 1, center_y
    ox, oy = obstacle_points
    xw, yw = xy_points
    min_x, min_y = min_coord
    occupancy_map = (np.zeros((xw, yw)))
    for (x, y) in zip(ox, oy):
        ix = int(round((x - min_x) / xy_resolution))
        iy = int(round((y - min_y) / xy_resolution))
        free_area = bresenham((prev_ix, prev_iy), (ix, iy))
        for fa in free_area:
            occupancy_map[fa[0]][fa[1]] = 0
        prev_ix = ix
        prev_iy = iy
    return occupancy_map

def flood_fill(center_point, occupancy_map):
    sx, sy = occupancy_map.shape
    fringe = deque()
    fringe.appendleft(center_point)
    while fringe:
        n = fringe.pop()
        nx, ny = n
        # West
        if nx > 0:
            if occupancy_map[nx - 1, ny] == 0.5:
                occupancy_map[nx - 1, ny] = 0.0
                fringe.appendleft((nx - 1, ny))
        # East
        if nx < sx - 1:
            if occupancy_map[nx + 1, ny] == 0.5:
                occupancy_map[nx + 1, ny] = 0.0
                fringe.appendleft((nx + 1, ny))
        # North
        if ny > 0:
            if occupancy_map[nx, ny - 1] == 0.5:
                occupancy_map[nx, ny - 1] = 0.0
                fringe.appendleft((nx, ny - 1))
        # South
        if ny < sy - 1:
            if occupancy_map[nx, ny + 1] == 0.5:
                occupancy_map[nx, ny + 1] = 0.0
                fringe.appendleft((nx, ny + 1))

def generate_ray_casting_grid_map(ox, oy, xy_resolution):
    min_x, min_y, max_x, max_y, x_w, y_w = calc_grid_map_config(
        ox, oy, xy_resolution)
    occupancy_map = np.zeros((x_w, y_w)) 
    center_x = int(
        round(-min_x / xy_resolution))
    center_y = int(
        round(-min_y / xy_resolution))
    occupancy_map = init_flood_fill((center_x, center_y), (ox, oy),
                                    (x_w, y_w),
                                    (min_x, min_y), xy_resolution)
    
    flood_fill((center_x, center_y), occupancy_map)

    occupancy_map = np.array(occupancy_map, dtype=float)
    for (x, y) in zip(ox, oy):
        ix = int(round((x - min_x) / xy_resolution))
        iy = int(round((y - min_y) / xy_resolution))
        occupancy_map[ix][iy] = 1.0  # occupied area 1.0
        occupancy_map[ix + 1][iy] = 1.0  # extend the occupied area
        occupancy_map[ix][iy + 1] = 1.0  # extend the occupied area
        occupancy_map[ix + 1][iy + 1] = 1.0  # extend the occupied area
    return occupancy_map, min_x, max_x, min_y, max_y, xy_resolution


def simulate_lidar(box_pos, box_angle, num_points=360, max_distance=1000, sim_jitter=True):
    angles = np.linspace(0, 360, num_points, endpoint=False)
    #distances = np.linspace(0.5, 1, num_points) 
    distances = np.zeros(num_points)
    for i in range(num_points):
        angle = angles[i]
        direction = np.array([np.cos(np.radians(angle)), np.sin(np.radians(angle))])  # Unit vector
        distance = raycast(box_pos, direction, obstacles)
        if distance:
            distances[i] = distance
    
    # Points that dont intersect are discarded
    angles = angles[np.where(distances>0)]
    distances = distances[np.where(distances>0)]
    
    if sim_jitter:
        distances += np.random.normal(-0.5, 0.5, size=len(distances))

    # Convert to Cartesian coordinates in the box's coordinate systen
    x_points = distances * np.cos(np.radians(angles + box_angle))
    y_points = distances * np.sin(np.radians(angles + box_angle)) 

    lidar_points = np.stack((x_points, y_points), axis=1)

    rotation_matrix = np.array([[np.cos(np.radians(box_angle)), -np.sin(np.radians(box_angle))],
                                [np.sin(np.radians(box_angle)), np.cos(np.radians(box_angle))]])

    lidar_points = np.dot(lidar_points, rotation_matrix) + np.array(box_pos) 
    return lidar_points


def generate_map(lidar_points, global_occupancy_map, xy_resolution=0.01):
    """Generates a simplified map from lidar points."""
    x = lidar_points[:, 0]
    y = lidar_points[:, 1]

    occupancy_map, *_ = generate_ray_casting_grid_map(
        x, y, xy_resolution)
    
    global_occupancy_map = np.logical_or(occupancy_map, global_occupancy_map)

    plot_map = np.copy(global_occupancy_map)
    plot_map = np.flip(np.rot90(plot_map), axis=0)
    fig, ax = plt.subplots(figsize=(2, 2))
    ax.imshow(plot_map, cmap='gray')
    ax.axis('off')
    plt.subplots_adjust(left=0, right=1, top=1, bottom=0)
    canvas = fig.canvas
    canvas.draw() 
    string = canvas.tostring_rgb()
    size = canvas.get_width_height()
    map_surface = pygame.image.fromstring(string, size, "RGB")
    plt.close(fig)
    return map_surface, global_occupancy_map

def convert_lidar_points(lidar_points, max_distance):
  """Converts lidar points to radians and scales distances between 0 and 1."""
  angles = np.arctan2(lidar_points[:, 0], lidar_points[:, 1])
  distances = np.linalg.norm(lidar_points, axis=1)
  # Scale distances between 0 and 1
  distances /= max_distance
  ox = np.sin(angles) * distances
  oy = np.cos(angles) * distances
  return np.stack((ox, oy), axis=1)

def raycast(start_point, direction, obstacles, max_len=300):
    shortest_distance = None
    for obstacle in obstacles:
        intersection_points = obstacle.clipline(start_point, start_point + direction * max_len)
        if intersection_points:
            for point in intersection_points:
                distance = np.linalg.norm(np.array(start_point) - np.array(point))  
                if shortest_distance is None or distance < shortest_distance:
                    shortest_distance = distance
    return shortest_distance 

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:  # Key pressed
            if event.key == pygame.K_w:
                throttle = 1.0  # Accelerate
            if event.key == pygame.K_s:
                throttle = -1.0  # Reverse/brake
            if event.key == pygame.K_a:
                steering = -1.0  # Turn left
            if event.key == pygame.K_d:
                steering = 1.0  # Turn right 
        if event.type == pygame.KEYUP:  # Key released
            if event.key in (pygame.K_w, pygame.K_s):
                throttle = 0.0 
            if event.key in (pygame.K_a, pygame.K_d):
                steering = 0.0

    # Update box angle and speed based on steering and throttle
    box_angle += steering * 5
    box_speed += throttle * 0.1
    box_speed *= 0.98  

    box_rect = pygame.Rect(box_pos[0] - box_size[0] // 2, box_pos[1] - box_size[1] // 2, *box_size)

    for rect in obstacles: 
        if rect.colliderect(box_rect):
            if box_speed != 0: # Don't bounce if already stopped
                if abs(box_pos[0] - rect.center[0]) > abs(box_pos[1] - rect.center[1]):
                    box_speed *= -0.5 # Bounce on X-axis 
                else:
                    box_speed *= -0.5 # Bounce on Y-axis

                diff_x = box_pos[0] - rect.center[0]
                diff_y = box_pos[1] - rect.center[1]
                angle_to_center = math.atan2(diff_y, diff_x)  # Angle towards the obstacle center

                while rect.colliderect(box_rect):
                    box_pos[0] += math.cos(angle_to_center) * 2
                    box_pos[1] += math.sin(angle_to_center) * 2
                    box_rect.center = box_pos 
    
    # Calculate new position
    box_pos[0] += math.cos(math.radians(box_angle)) * box_speed
    box_pos[1] += math.sin(math.radians(box_angle)) * box_speed

    pivot_offset = box_size[0]*3 / 4  # Offset from the center to the front 1/3
    pivot_x = box_pos[0] + pivot_offset * math.cos(math.radians(box_angle))
    pivot_y = box_pos[1] + pivot_offset * math.sin(math.radians(box_angle))

    # Wrap around screen edges
    box_pos[0] %= width
    box_pos[1] %= height

    lidar_points = simulate_lidar(box_pos, box_angle, num_points=300, max_distance=1000)

    # Drawing
    screen.fill(black)
    # Calculate box's corner points
    corners = []
    for corner in [(1, 1), (1, -1), (-1, -1), (-1, 1)]:
        x_offset = corner[0] * box_size[0] / 2
        y_offset = corner[1] * box_size[1] / 2

        # Offset from the pivot point
        x_from_pivot = x_offset - pivot_offset
        y_from_pivot = y_offset

        # Rotate around the pivot
        rotated_x = pivot_x + x_from_pivot * math.cos(math.radians(box_angle)) - y_from_pivot * math.sin(math.radians(box_angle))
        rotated_y = pivot_y + x_from_pivot * math.sin(math.radians(box_angle)) + y_from_pivot * math.cos(math.radians(box_angle))

        corners.append((rotated_x, rotated_y))

    converted_lidar_points = convert_lidar_points(lidar_points, max_distance=math.sqrt(width**2+height**2)) 

    map_surface, global_occupancy_map = generate_map(converted_lidar_points, global_occupancy_map)
    
    for rect in obstacles:
        pygame.draw.rect(screen, (255, 0, 0), rect)  # Red

    for point in lidar_points:
       pygame.draw.circle(screen, (0, 255, 0), (int(point[0]), int(point[1])), 2)  # Green dots

    pygame.draw.polygon(screen, white, corners)
    screen.blit(map_surface, (0, 0))
    pygame.display.flip()

    clock.tick(60)

pygame.quit()
sys.exit()
