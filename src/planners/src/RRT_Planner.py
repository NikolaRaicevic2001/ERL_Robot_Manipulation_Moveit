import numpy as np
import pygame
import random

# Constants
WINDOW_SIZE = 600
NUM_OBSTACLES = 10
OBSTACLE_SIZE = 30
STEP_SIZE = 20
GOAL_BIAS = 0.05  # Probability of choosing goal as the new point
MAX_ITERATIONS = 1000

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

# Pygame initialization
pygame.init()
screen = pygame.display.set_mode((WINDOW_SIZE, WINDOW_SIZE))
pygame.display.set_caption('RRT Path Planning')
clock = pygame.time.Clock()


class RRTNode:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


def distance(node1, node2):
    return np.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)


def is_collision_free(point, obstacles):
    for obstacle in obstacles:
        if distance(point, obstacle) <= OBSTACLE_SIZE:
            return False
    return True


def generate_obstacles():
    obstacles = []
    for _ in range(NUM_OBSTACLES):
        x = random.randint(0, WINDOW_SIZE)
        y = random.randint(0, WINDOW_SIZE)
        obstacles.append(RRTNode(x, y))
    return obstacles


def extend_tree(tree, obstacles, goal):
    rand_node = RRTNode(random.randint(0, WINDOW_SIZE), random.randint(0, WINDOW_SIZE))
    if random.random() < GOAL_BIAS:
        rand_node = goal

    nearest_node = None
    min_distance = float('inf')
    for node in tree:
        d = distance(rand_node, node)
        if d < min_distance and is_collision_free(node, obstacles):
            min_distance = d
            nearest_node = node

    if nearest_node:
        new_node_x = nearest_node.x + (STEP_SIZE * (rand_node.x - nearest_node.x) / min_distance)
        new_node_y = nearest_node.y + (STEP_SIZE * (rand_node.y - nearest_node.y) / min_distance)
        new_node = RRTNode(new_node_x, new_node_y)
        if is_collision_free(new_node, obstacles):
            new_node.parent = nearest_node
            tree.append(new_node)
            pygame.draw.line(screen, BLACK, (nearest_node.x, nearest_node.y), (new_node.x, new_node.y))

            if distance(new_node, goal) < STEP_SIZE:
                goal.parent = new_node
                return True

    return False


def find_path(tree, goal):
    path = [goal]
    current = goal
    while current.parent:
        path.append(current.parent)
        current = current.parent
    return path


def draw_obstacles(obstacles):
    for obstacle in obstacles:
        pygame.draw.circle(screen, RED, (obstacle.x, obstacle.y), OBSTACLE_SIZE)


def draw_tree(tree):
    for node in tree:
        if node.parent:
            pygame.draw.line(screen, BLACK, (node.x, node.y), (node.parent.x, node.parent.y))


def main():
    obstacles = generate_obstacles()
    start = RRTNode(50, 50)
    goal = RRTNode(WINDOW_SIZE - 50, WINDOW_SIZE - 50)

    tree = [start]

    for _ in range(MAX_ITERATIONS):
        if extend_tree(tree, obstacles, goal):
            path = find_path(tree, goal)
            for i in range(len(path) - 1):
                pygame.draw.line(screen, GREEN, (path[i].x, path[i].y), (path[i + 1].x, path[i + 1].y))
            break

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return

        screen.fill(WHITE)
        draw_obstacles(obstacles)
        draw_tree(tree)
        pygame.draw.circle(screen, BLUE, (start.x, start.y), 10)
        pygame.draw.circle(screen, BLUE, (goal.x, goal.y), 10)

        pygame.display.flip()
        clock.tick(60)


if __name__ == "__main__":
    main()
