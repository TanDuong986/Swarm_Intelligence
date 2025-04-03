import random
import pygame
import json
import os
from heapq import heappush, heappop
import time
import math

# Hàm tạo màu dựa trên cost
def get_cost_color(cost, max_cost):
    if cost == float('inf'):
        return (0, 0, 0)  # Đen cho vật cản
    normalized = min(cost / max_cost, 1.0)
    r = int(255 * normalized)
    g = int(255 * (1 - normalized))
    b = 0
    return (r, g, b)

class Node:
    def __init__(self, x, y, is_obstacle=False, cost=1):
        self.x = x
        self.y = y
        self.is_obstacle = is_obstacle
        self.neighbors = []
        self.cost = float('inf') if is_obstacle else cost  # Cost mặc định
        self.g_score = float('inf')  # Dùng cho A* hoặc các thuật toán khác
        self.h_score = float('inf')  # heuristic
        self.f_score = float('inf')  # Dùng cho A*
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

        if json_file and os.path.exists(json_file):
            self.load_from_json(json_file)
        else:
            for x in range(grid_size):
                for y in range(grid_size):
                    cost = 1  # Mặc định cost là 1
                    self.nodes[(x, y)] = Node(x, y, is_obstacle=False, cost=cost)
            self.set_start(0, 0)
            self.goal = self.get_random_free_cell()
            while self.goal == self.start:
                self.goal = self.get_random_free_cell()

        for x in range(grid_size):
            for y in range(grid_size):
                current_node = self.nodes[(x, y)]
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1),
                               (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < grid_size and 0 <= ny < grid_size:
                        current_node.add_neighbor(self.nodes[(nx, ny)])
        if not self.goal:
            self.goal = self.get_random_free_cell()

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
        self.goal.g_score = 0

# Lớp GraphRRT kế thừa từ Graph, tích hợp thuật toán RRT và hàm save_image
class GraphRRT(Graph):
    def rrt(self, max_iterations=1000, step_size=1):
        """
        Thuật toán RRT (Rapidly-exploring Random Tree) trên không gian grid.
        Trả về: (path, log, frontier_log, total_explored, final_cost, iterations)
          - path: danh sách các điểm (x,y) từ start đến goal
          - log: danh sách các điểm được thêm vào cây theo thứ tự
          - frontier_log: danh sách snapshot của cây (các điểm hiện có trong cây tại mỗi iteration)
          - total_explored: số lượng node trong cây
          - final_cost: tổng chi phí đường đi (tích lũy move_cost)
          - iterations: số vòng lặp đã chạy
        """
        def distance(p1, p2):
            return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
        
        def allowed_move(current, neighbor):
            dx = neighbor.x - current.x
            dy = neighbor.y - current.y
            if abs(dx) == 1 and abs(dy) == 1:
                node_horizontal = self.get_node(current.x + dx, current.y)
                node_vertical = self.get_node(current.x, current.y + dy)
                if node_horizontal.is_obstacle or node_vertical.is_obstacle:
                    return False
            return True

        # Đặt g_score của nút xuất phát bằng 0
        self.start.g_score = 0

        tree = [self.start]  # Cây RRT chứa các Node đã được thêm vào
        log = [(self.start.x, self.start.y)]
        frontier_log = []
        iterations = 0

        while iterations < max_iterations:
            iterations += 1
            # Lưu snapshot của cây
            frontier_log.append([(node.x, node.y) for node in tree])
            # Sinh điểm ngẫu nhiên trong không gian grid
            rand_x = random.randint(0, self.grid_size - 1)
            rand_y = random.randint(0, self.grid_size - 1)
            random_point = (rand_x, rand_y)
            # Tìm nút trong cây có khoảng cách gần nhất đến random_point
            nearest = min(tree, key=lambda node: distance((node.x, node.y), random_point))
            dx = rand_x - nearest.x
            dy = rand_y - nearest.y
            dist = math.sqrt(dx**2 + dy**2)
            if dist == 0:
                continue
            step_dx = int(round((dx / dist) * step_size))
            step_dy = int(round((dy / dist) * step_size))
            new_x = nearest.x + step_dx
            new_y = nearest.y + step_dy
            new_x = max(0, min(self.grid_size - 1, new_x))
            new_y = max(0, min(self.grid_size - 1, new_y))
            new_node = self.get_node(new_x, new_y)
            if new_node.is_obstacle:
                continue
            if abs(step_dx) == 1 and abs(step_dy) == 1:
                if not allowed_move(nearest, new_node):
                    continue
            if (new_node.x, new_node.y) in [(node.x, node.y) for node in tree]:
                continue
            new_node.parent = nearest
            tree.append(new_node)
            log.append((new_node.x, new_node.y))
            move_cost = 1.41 if (abs(step_dx) == 1 and abs(step_dy) == 1) else 1
            new_node.g_score = nearest.g_score + move_cost
            # Kiểm tra nếu new_node đủ gần goal
            if distance((new_node.x, new_node.y), (self.goal.x, self.goal.y)) <= step_size:
                self.goal.parent = new_node
                dist_to_goal = distance((new_node.x, new_node.y), (self.goal.x, self.goal.y))
                self.goal.g_score = new_node.g_score + dist_to_goal
                final_cost = self.goal.g_score
                log.append((self.goal.x, self.goal.y))
                tree.append(self.goal)
                frontier_log.append([(node.x, node.y) for node in tree])
                # Xây dựng path từ goal ngược về start
                path = []
                current = self.goal
                while current:
                    path.append((current.x, current.y))
                    current = current.parent
                return path[::-1], log, frontier_log, len(tree), final_cost, iterations

        return [], log, frontier_log, len(tree), 0, iterations

    def save_image(self, screen, filename):
        pygame.image.save(screen, filename)

# Hàm calculate_deviation dùng chung cho các thuật toán
def calculate_deviation(path):
    if len(path) < 3:
        return 0
    total_deviation = 0
    for i in range(1, len(path) - 1):
        p_prev = path[i - 1]
        p_curr = path[i]
        p_next = path[i + 1]
        dx1 = p_curr[0] - p_prev[0]
        dy1 = p_curr[1] - p_prev[1]
        dx2 = p_next[0] - p_curr[0]
        dy2 = p_next[1] - p_curr[1]
        angle1 = math.atan2(dy1, dx1)
        angle2 = math.atan2(dy2, dx2)
        angle_diff = abs(angle2 - angle1)
        if angle_diff > math.pi:
            angle_diff = 2 * math.pi - angle_diff
        total_deviation += angle_diff
    return total_deviation

def save_image(screen, filename):
    pygame.image.save(screen, filename)
# Hàm draw_map dùng chung (giống như các thuật toán khác)
def draw_map(graph, grid_size, screen, timestep=-1, frontier_log=None):
    cell_size = 600 // grid_size
    screen.fill((255, 255, 255))
    for x in range(grid_size):
        for y in range(grid_size):
            node = graph.get_node(x, y)
            if node == graph.start:
                color = (0, 0, 255)  # Start: xanh nước biển
            elif node == graph.goal:
                color = (255, 0, 0)  # Goal: đỏ
            elif (x, y) in graph.path_log[:timestep + 1]:
                color = (255, 255, 0)  # Đã đi qua: vàng
            elif node.is_obstacle:
                color = (0, 0, 0)  # Vật cản: đen
            else:
                color = get_cost_color(node.cost, graph.max_cost)
            pygame.draw.rect(screen, color, (x * cell_size, y * cell_size, cell_size, cell_size))
            pygame.draw.rect(screen, (50, 50, 50), (x * cell_size, y * cell_size, cell_size, cell_size), 1)
    
    # Vẽ frontier (biên tìm kiếm) nếu có
    if frontier_log and timestep >= 0 and timestep < len(frontier_log):
        for x, y in frontier_log[timestep]:
            pygame.draw.rect(screen, (212, 126, 252), (x * cell_size, y * cell_size, cell_size, cell_size))
            pygame.draw.rect(screen, (50, 50, 50), (x * cell_size, y * cell_size, cell_size, cell_size), 1)
    
    pygame.draw.rect(screen, (0, 0, 255), (graph.start.x * cell_size, graph.start.y * cell_size, cell_size, cell_size)) # Ve them goal khi hoan thanh
    pygame.draw.rect(screen, (50, 50, 50), (graph.start.x * cell_size, graph.start.y * cell_size, cell_size, cell_size), 1)
    # Khi hoàn thành, vẽ đường đi tối ưu
    if timestep >= len(graph.path_log) - 1:
        node = graph.goal
        while node:
            pygame.draw.circle(screen, (3, 252, 202), (node.x * cell_size + cell_size // 2, node.y * cell_size + cell_size // 2), cell_size // 4)
            node = node.parent
        graph.save_image(screen, "maze_rrt.png")

def main():
    pygame.init()
    grid_size = 25
    width, height = 600, 600
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("RRT Path Planning Visualization")

    # Sử dụng GraphRRT thay cho Graph
    graph = GraphRRT(grid_size, json_file="map/aStar.json")
    graph.set_start(5, 8)
    graph.set_goal(1, 23)

    start_time = time.time()
    path, log, frontier_log, total_explored, final_cost, iterations = graph.rrt(max_iterations=3000, step_size=1)
    comp_time = time.time() - start_time
    graph.path_log = log

    deviation = calculate_deviation(path) if path else None

    print("RRT Results:")
    print("Path:", path)
    print("Total nodes expanded:", total_explored)
    print("Final cost:", final_cost)
    print("Iterations:", iterations)
    print("Computation time (s):", comp_time)
    print("Path deviation (radians):", deviation)

    clock = pygame.time.Clock()
    running = True
    timestep = 0
    start_iter = time.time()
    is_paused = False
    search_done = False

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    is_paused = not is_paused

        if not search_done and not is_paused:
            if time.time() - start_iter > 0.05:
                timestep = (timestep + 1) % (len(graph.path_log) + 1)
                start_iter = time.time()
                if timestep == len(graph.path_log):
                    search_done = True

        draw_map(graph, grid_size, screen, timestep, frontier_log)
        pygame.display.flip()
        clock.tick(24)

    pygame.quit()

if __name__ == "__main__":
    main()
