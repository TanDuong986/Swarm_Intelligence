import random
import pygame
import json
import os
from heapq import heappush, heappop
import time
from queue import PriorityQueue

# Màu sắc
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 255, 0)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165 ,0)
GREY = (128, 128, 128)
TURQUOISE = (64, 224, 208)

# Hàm tạo màu dựa trên cost
def get_cost_color(cost, max_cost):
    if cost == float('inf'):
        return (0, 0, 0)  # Đen cho vật cản
    normalized = min(cost / max_cost, 1.0)
    r = int(255 * normalized)
    g = int(255 * (1 - normalized))
    b = 255
    return (r, g, b)

class Node:
    def __init__(self, x, y, is_obstacle=False, cost=2):
        self.x = x
        self.y = y
        self.is_obstacle = is_obstacle
        self.neighbors = []
        self.cost = float('inf') if is_obstacle else cost
        self.g_score = float('inf')
        self.f_score = float('inf')
        self.parent = None

    def add_neighbor(self, neighbor):
        self.neighbors.append(neighbor)

class Graph:
    def __init__(self, grid_size, use_random=False, obstacle_ratio=0.3, json_file=None):
        self.grid_size = grid_size
        self.nodes = {}
        self.start = None
        self.goal = None
        self.path_log = []
        self.max_cost = grid_size * 2
        self.frontier = set()  # Biên tìm kiếm

        if json_file and os.path.exists(json_file):
            self.load_from_json(json_file)
        else:
            for x in range(grid_size):
                for y in range(grid_size):
                    cost = 1
                    self.nodes[(x, y)] = Node(x, y, is_obstacle=False, cost=cost)
            self.set_start(0, 0)
            self.goal = self.get_random_free_cell()
            while self.goal == self.start:
                self.goal = self.get_random_free_cell()

        for x in range(grid_size):
            for y in range(grid_size):
                current_node = self.nodes[(x, y)]
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < grid_size and 0 <= ny < grid_size:
                        current_node.add_neighbor(self.nodes[(nx, ny)])

    def load_from_json(self, json_file):
        with open(json_file, 'r') as f:
            data = json.load(f)
            maze_data = data.get("data")

            for x in range(self.grid_size):
                for y in range(self.grid_size):
                    is_obstacle = (maze_data[x][y] == 1)
                    cost = float('inf') if is_obstacle else 1
                    self.nodes[(x, y)] = Node(x, y, is_obstacle, cost)

    def get_node(self, x, y):
        return self.nodes.get((x, y))

    def get_random_free_cell(self):
        while True:
            x = random.randint(0, self.grid_size - 1)
            y = random.randint(0, self.grid_size - 1)
            if not self.nodes[(x, y)].is_obstacle:
                return self.nodes[(x, y)]

    def set_start(self, x, y):
        self.start = self.nodes[(x, y)]

    def set_goal(self, x, y):
        self.goal = self.nodes[(x, y)]

    def a_star(self, draw):
        open_set = []
        closed_set = set()
        self.start.g_score = 0
        self.start.f_score = abs(self.start.x - self.goal.x) + abs(self.start.y - self.goal.y)
        heappush(open_set, (self.start.f_score, id(self.start), self.start))

        log = []

        while open_set:
            _, _, current = heappop(open_set)
            self.frontier.add(current)  # Thêm node vào biên tìm kiếm
            if current == self.goal:
                path = []
                while current:
                    path.append((current.x, current.y))
                    current = current.parent
                return path[::-1], log

            closed_set.add(current)
            log.append((current.x, current.y))

            for neighbor in current.neighbors:
                if neighbor in closed_set:
                    continue
                tentative_g_score = current.g_score + neighbor.cost

                if tentative_g_score < neighbor.g_score:
                    neighbor.parent = current
                    neighbor.g_score = tentative_g_score
                    neighbor.f_score = neighbor.g_score + abs(neighbor.x - self.goal.x) + abs(neighbor.y - self.goal.y)
                    if neighbor not in [n[2] for n in open_set]:
                        heappush(open_set, (neighbor.f_score, id(neighbor), neighbor))

            draw(self.frontier)  # Cập nhật biên tìm kiếm khi vẽ lại
        return [], log

def draw_map(graph, grid_size, screen, timestep=-1, frontier=set()):
    cell_size = 600 // grid_size
    screen.fill((255, 255, 255))

    for x in range(grid_size):
        for y in range(grid_size):
            node = graph.get_node(x, y)

            if node == graph.start:
                color = ORANGE
            elif node == graph.goal:
                color = TURQUOISE
            elif (x, y) in frontier:
                color = GREEN  # Màu xanh lá cây cho biên tìm kiếm
            elif (x, y) in graph.path_log[:timestep + 1]:
                color = YELLOW  # Màu vàng cho điểm đã đi qua
            elif node.is_obstacle:
                color = BLACK
            else:
                color = get_cost_color(node.cost, graph.max_cost)

            pygame.draw.rect(screen, color, (x * cell_size, y * cell_size, cell_size, cell_size))
            pygame.draw.rect(screen, (50, 50, 50), (x * cell_size, y * cell_size, cell_size, cell_size), 1)

    if timestep == len(graph.path_log) - 1:
        node = graph.goal
        while node:
            pygame.draw.circle(screen, PURPLE, (node.x * cell_size + cell_size // 2, node.y * cell_size + cell_size // 2), cell_size // 4)
            node = node.parent

def save_image(screen, filename):
    pygame.image.save(screen, filename)

def main():
    pygame.init()
    grid_size = 25
    width, height = 600, 600
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("Maze Path Planning")

    graph = Graph(grid_size, json_file="map/none.json")
    graph.set_start(24, 1)
    graph.set_goal(1, 24)

    path, log = graph.a_star(lambda frontier: draw_map(graph, grid_size, screen, len(log), frontier))
    graph.path_log = log

    clock = pygame.time.Clock()
    running = True
    timestep = 0
    start = time.time()
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                pygame.quit()

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    time.sleep(10)

        if time.time() - start > 0.05:
            timestep = (timestep + 1) % (len(graph.path_log) + 1)
            start = time.time()

        draw_map(graph, grid_size, screen, timestep, graph.frontier)
        pygame.display.flip()
        clock.tick(20)

    pygame.quit()

if __name__ == "__main__":
    main()
