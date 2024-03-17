import pygame
import math
import sys
import numpy as np
import math
from typing import List, Tuple, Deque
from numba import njit
# Initialize Pygame
pygame.init()

# Screen dimensions
width: int = 600
height: int = 600
screen = pygame.display.set_mode((width, height))

# Colors
black = (0, 0, 0)
white = (255, 255, 255)

# Box (car) properties
box_pos = [width // 2, height // 2]
box_angle: float = 0
box_speed: float = 0
box_size = (40, 20)

# Game loop
running = True
clock = pygame.time.Clock()
steering: float = 0.0
throttle: float = 0.0

# obstacles
rect1 = pygame.Rect(200, 150, 100, 50)  # (x, y, width, height)
rect2 = pygame.Rect(550, 300, 80, 120)
rect3 = pygame.Rect(400, 200, 200, 40)
rect4 = pygame.Rect(200, 400, 200, 40)

# border
rect5 = pygame.Rect(0, 0, width, 10)
rect6 = pygame.Rect(0, height-10, width, 10)
rect7 = pygame.Rect(0, 0, 10, height)
rect8 = pygame.Rect(width-10, 0, 10, height)

obstacles: List[pygame.Rect] = [rect1, rect2, rect3, rect4, rect5, rect6, rect7, rect8]

global_occupancy_map = np.zeros((100,100))
                
def generate_ray_casting_grid_map(ox: np.ndarray, oy: np.ndarray, xy_resolution: float) -> np.ndarray:
    occupancy_map = np.zeros(global_occupancy_map.shape)
    ix = (ox / xy_resolution).astype(int) + 1
    iy = (oy / xy_resolution).astype(int) + 1
    occupancy_map[ix, iy] = 1.0  # occupied area 1.0
    return occupancy_map

def simulate_lidar(box_pos: List[int], box_angle: float, num_points: int = 360, max_distance: int = 300, sim_jitter: bool = True) -> np.ndarray:
    angles = np.linspace(0, 360, num_points, endpoint=False)
    distances = np.zeros(num_points)
    directions = np.array([np.cos(np.radians(angles)), np.sin(np.radians(angles))])  # Unit vectors
    for i in range(num_points):
        distance = raycast(box_pos, directions[:, i], obstacles, max_distance)
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

def convert_lidar_points(lidar_points: np.ndarray, max_distance: int) -> np.ndarray:
  """Converts lidar points to radians and scales distances between 0 and 1."""
  angles = np.arctan2(lidar_points[:, 0], lidar_points[:, 1])
  distances = np.linalg.norm(lidar_points, axis=1)
  # Scale distances between 0 and 1
  distances /= max_distance
  ox = np.sin(angles) * distances
  oy = np.cos(angles) * distances
  return np.stack((ox, oy), axis=1)

def generate_map(lidar_points: np.ndarray, global_occupancy_map: np.ndarray, xy_resolution: float = 0.01) -> Tuple[pygame.Surface, np.ndarray]:
    """Generates a simplified map from lidar points."""
    x = lidar_points[:, 0]
    y = lidar_points[:, 1]

    occupancy_map = generate_ray_casting_grid_map(
        x, y, xy_resolution)
    
    global_occupancy_map = np.logical_or(occupancy_map, global_occupancy_map).astype(int)
    plot_map = np.copy(global_occupancy_map)*255
    map_surface = pygame.surfarray.make_surface(plot_map)
    return map_surface, global_occupancy_map

# def raycast(start_point: Tuple[int, int], direction: Tuple[float, float], obstacles: List[pygame.Rect], max_len: int = 1000) -> float:
#     shortest_distance = None
#     for obstacle in obstacles:
#         intersection_points = obstacle.clipline(start_point, start_point + direction * max_len)
#         if intersection_points:
#             for point in intersection_points:
#                 distance = np.linalg.norm(np.array(start_point) - np.array(point))  
#                 if shortest_distance is None or distance < shortest_distance:
#                     shortest_distance = distance
#     return shortest_distance 

from numba import njit

@njit(fastmath=True)
def raycast(start_point: Tuple[int, int], direction: Tuple[float, float], obstacles: List[Tuple[int, int, int, int]], max_len: int = 1000) -> float:
    for obstacle in obstacles:
        intersection_points = obstacle_clipline(start_point, start_point + direction * max_len, obstacle)
        if intersection_points:
            distance = np.linalg.norm(np.array(start_point) - np.array(intersection_points[0]))
            return distance
    return None

@njit(fastmath=True)
def obstacle_clipline(start_point: Tuple[int, int], end_point: Tuple[int, int], obstacle: Tuple[int, int, int, int]) -> Tuple[Tuple[int, int], Tuple[int, int]]:
    x1, y1, x2, y2 = obstacle
    x3, y3 = start_point
    x4, y4 = end_point
    den = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)
    if den == 0:
        return None
    ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / den
    if ua < 0 or ua > 1:
        return None
    ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / den
    if ub < 0 or ub > 1:
        return None
    x = x1 + ua * (x2 - x1)
    y = y1 + ua * (y2 - y1)
    return ((int(x), int(y)),)

i = 0
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
    
    # simulate 6 Hz
    if i % 10 == 0:
        lidar_points = simulate_lidar(box_pos, box_angle, num_points=200, max_distance=300)
        converted_lidar_points = convert_lidar_points(lidar_points, max_distance=math.sqrt(width**2+height**2)) 
        map_surface, global_occupancy_map = generate_map(converted_lidar_points, global_occupancy_map)
    
    for rect in obstacles:
        pygame.draw.rect(screen, (255, 0, 0), rect)  # Red

    for point in lidar_points:
        pygame.draw.circle(screen, (0, 255, 0), (int(point[0]), int(point[1])), 2)  # Green dots

    pygame.draw.polygon(screen, white, corners)
    #map_surface = pygame.transform.scale_by(map_surface, 2)
    screen.blit(map_surface, (0, 0))
    pygame.display.flip()

    clock.tick(60)
    i+=1
    

pygame.quit()
sys.exit()