import random
import pygame
import json
import os
import time
import math
from heapq import heappush, heappop

# Hàm tạo màu dựa trên cost (giữ nguyên)
def get_cost_color(cost, max_cost):
    if cost == float('inf'):
        return (0, 0, 0)
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
        self.cost = float('inf') if is_obstacle else cost
        self.g_score = float('inf')
        self.h_score = float('inf')
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

    def a_star(self):
        # Đây là phiên bản a_star của Graph (như đã có)
        open_set = [(self.start.f_score, id(self.start), self.start)]
        closed_set = set()
        iteration_count = 0
        self.start.g_score = 0
        self.start.f_score = abs(self.start.x - self.goal.x) + abs(self.start.y - self.goal.y)
        log = []
        frontier_log = []

        while open_set:
            iteration_count += 1
            frontier_log.append([(node.x, node.y) for _, _, node in open_set])
            current_tuple = min(open_set, key=lambda x: (x[0], x[1]))
            current = current_tuple[2]
            open_set.remove(current_tuple)

            if current == self.goal:
                path = []
                while current:
                    path.append((current.x, current.y))
                    current = current.parent
                total_explored = len(closed_set) + 1
                final_cost = self.goal.g_score
                return path[::-1], log, frontier_log, total_explored, final_cost, iteration_count

            closed_set.add(current)
            log.append((current.x, current.y))

            for neighbor in current.neighbors:
                if neighbor in closed_set or neighbor.is_obstacle:
                    continue
                dx = neighbor.x - current.x
                dy = neighbor.y - current.y
                if abs(dx) == 1 and abs(dy) == 1:
                    node_horizontal = self.get_node(current.x + dx, current.y)
                    node_vertical = self.get_node(current.x, current.y + dy)
                    if node_horizontal.is_obstacle or node_vertical.is_obstacle:
                        continue
                    move_cost = 1.41
                else:
                    move_cost = 1
                tentative_g_score = current.g_score + move_cost

                if tentative_g_score < neighbor.g_score:
                    neighbor.parent = current
                    neighbor.g_score = tentative_g_score
                    neighbor.f_score = neighbor.g_score + abs(neighbor.x - self.goal.x) + abs(neighbor.y - self.goal.y)
                    neighbor_in_open = False
                    for item in open_set:
                        if item[2] == neighbor:
                            neighbor_in_open = True
                            open_set.remove(item)
                            open_set.append((neighbor.f_score, id(neighbor), neighbor))
                            break
                    if not neighbor_in_open:
                        open_set.append((neighbor.f_score, id(neighbor), neighbor))
        return [], log, frontier_log, len(closed_set), 0, iteration_count

def save_image(screen, filename):
    pygame.image.save(screen, filename)


# -------------------------
# Lớp kế thừa Graph để tích hợp ACO
class GraphACO(Graph):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # Khởi tạo pheromone cho mỗi cạnh
        self.pheromones = {}
        self._initialize_pheromones()
        
    def _edge_key(self, n1, n2):
        return tuple(sorted(((n1.x, n1.y), (n2.x, n2.y))))
    
    def _initialize_pheromones(self):
        for node in self.nodes.values():
            for neighbor in node.neighbors:
                if neighbor.is_obstacle:
                    continue
                key = self._edge_key(node, neighbor)
                if key not in self.pheromones:
                    self.pheromones[key] = 0.1
                    
    def allowed_move(self, current, neighbor):
        dx = neighbor.x - current.x
        dy = neighbor.y - current.y
        if abs(dx) == 1 and abs(dy) == 1:
            node_horizontal = self.get_node(current.x + dx, current.y)
            node_vertical = self.get_node(current.x, current.y + dy)
            if node_horizontal.is_obstacle or node_vertical.is_obstacle:
                return False
        return True
    
    def move_cost(self, current, neighbor):
        dx = abs(neighbor.x - current.x)
        dy = abs(neighbor.y - current.y)
        if dx == 1 and dy == 1:
            return 1.41
        return 1
    
    def heuristic(self, node):
        return abs(node.x - self.goal.x) + abs(node.y - self.goal.y)
    
    def aco(self, num_ants=50, num_iterations=100, evaporation_rate=0.1, alpha=1, beta=2):
        best_path = None
        best_cost = float("inf")
        all_paths = []  # Log đường đi của các ant trong mỗi iteration
        
        # Đặt lại pheromone (nếu cần)
        for key in self.pheromones:
            self.pheromones[key] = 0.1
        
        for iteration in range(num_iterations):
            iteration_paths = []
            for ant in range(num_ants):
                current = self.start
                path = [current]
                total_cost = 0
                visited = set()
                visited.add((current.x, current.y))
                max_steps = self.grid_size * self.grid_size
                steps = 0
                while current != self.goal and steps < max_steps:
                    steps += 1
                    allowed_neighbors = []
                    for neighbor in current.neighbors:
                        if neighbor.is_obstacle:
                            continue
                        if (neighbor.x, neighbor.y) in visited:
                            continue
                        if not self.allowed_move(current, neighbor):
                            continue
                        allowed_neighbors.append(neighbor)
                    if not allowed_neighbors:
                        break
                    # Tính xác suất chuyển động
                    probs = []
                    for neighbor in allowed_neighbors:
                        key = self._edge_key(current, neighbor)
                        pheromone = self.pheromones.get(key, 0.1)
                        eta = 1.0 / (self.move_cost(current, neighbor) + 1e-6)
                        probs.append((pheromone ** alpha) * (eta ** beta))
                    total = sum(probs)
                    probs = [p / total for p in probs]
                    r = random.random()
                    cumulative = 0.0
                    next_node = None
                    for i, neighbor in enumerate(allowed_neighbors):
                        cumulative += probs[i]
                        if r <= cumulative:
                            next_node = neighbor
                            break
                    if next_node is None:
                        break
                    path.append(next_node)
                    visited.add((next_node.x, next_node.y))
                    total_cost += self.move_cost(current, next_node)
                    current = next_node
                if current == self.goal:
                    iteration_paths.append((path, total_cost))
                    if total_cost < best_cost:
                        best_cost = total_cost
                        best_path = path
            # Cập nhật pheromone: bay hơi
            for key in self.pheromones:
                self.pheromones[key] *= (1 - evaporation_rate)
            # Tăng pheromone theo đường đi của các ant đạt goal
            for path, cost in iteration_paths:
                if path[-1] == self.goal:
                    deposit = 1.0 / cost
                    for i in range(len(path) - 1):
                        key = self._edge_key(path[i], path[i+1])
                        self.pheromones[key] += deposit
            all_paths.append(iteration_paths)
        return best_path, best_cost, all_paths

def draw_grid(screen, grid_size, cell_size):
    for x in range(grid_size):
        for y in range(grid_size):
            rect = pygame.Rect(x * cell_size, y * cell_size, cell_size, cell_size)
            pygame.draw.rect(screen, (50, 50, 50), rect, 1)

def draw_nodes(screen, graph, cell_size):
    # Vẽ các ô theo trạng thái ban đầu (start, goal, vật cản, cost)
    for x in range(graph.grid_size):
        for y in range(graph.grid_size):
            node = graph.get_node(x, y)
            if node == graph.start:
                color = (0, 0, 255)  # Start: xanh nước biển
            elif node == graph.goal:
                color = (255, 0, 0)  # Goal: đỏ
            elif node.is_obstacle:
                color = (0, 0, 0)  # Vật cản: đen
            else:
                color = get_cost_color(node.cost, graph.max_cost)
            pygame.draw.rect(screen, color, (x * cell_size, y * cell_size, cell_size, cell_size))
    draw_grid(screen, graph.grid_size, cell_size)

def draw_iteration_paths(screen, cell_size, iteration_paths, color=(212, 126, 252)):
    # Vẽ các đường đi của các con kiến trong một iteration (mỗi path là danh sách Node)
    for path, cost in iteration_paths:
        if len(path) < 2:
            continue
        for i in range(len(path) - 1):
            start_pos = (path[i].x * cell_size + cell_size // 2, path[i].y * cell_size + cell_size // 2)
            end_pos   = (path[i+1].x * cell_size + cell_size // 2, path[i+1].y * cell_size + cell_size // 2)
            pygame.draw.line(screen, color, start_pos, end_pos, 2)

def draw_best_path(screen, cell_size, best_path, color=(3,252,202)):
    if not best_path or len(best_path) < 2:
        return
    for i in range(len(best_path) - 1):
        start_pos = (best_path[i].x * cell_size + cell_size // 2, best_path[i].y * cell_size + cell_size // 2)
        end_pos   = (best_path[i+1].x * cell_size + cell_size // 2, best_path[i+1].y * cell_size + cell_size // 2)
        pygame.draw.line(screen, color, start_pos, end_pos, 4)
    

def get_cost_color(cost, max_cost):
    if cost == float('inf'):
        return (0, 0, 0)
    normalized = min(cost / max_cost, 1.0)
    r = int(255 * normalized)
    g = int(255 * (1 - normalized))
    b = 0
    return (r, g, b)

def calculate_deviation(path):
    """Tính tổng góc chuyển hướng (radians) của đường đi.
    
    Đầu vào:
      path: danh sách các tuple (x, y) thể hiện tọa độ các điểm trên đường đi.
    Trả về:
      Tổng góc chuyển hướng (radians) của đường đi.
    """
    if len(path) < 3:
        return 0
    total_deviation = 0
    for i in range(1, len(path) - 1):
        p_prev = path[i - 1]
        p_curr = path[i]
        p_next = path[i + 1]
        # Tạo vector từ p_prev đến p_curr và từ p_curr đến p_next
        v1 = (p_curr[0] - p_prev[0], p_curr[1] - p_prev[1])
        v2 = (p_next[0] - p_curr[0], p_next[1] - p_curr[1])
        angle1 = math.atan2(v1[1], v1[0])
        angle2 = math.atan2(v2[1], v2[0])
        angle_diff = abs(angle2 - angle1)
        if angle_diff > math.pi:
            angle_diff = 2 * math.pi - angle_diff
        total_deviation += angle_diff
    return total_deviation


def main():
    pygame.init()
    grid_size = 25
    width = 600
    screen = pygame.display.set_mode((width, width))
    pygame.display.set_caption("ACO Path Planning Visualization")

    # Khởi tạo đồ thị bằng GraphACO (kế thừa từ Graph và tích hợp ACO)
    graph = GraphACO(grid_size, json_file="map/basic.json")
    graph.set_start(5, 8) #(x, y theo hệ trục của ảnhh)
    graph.set_goal(1, 23)

    # Chạy ACO và nhận kết quả
    start_time = time.time()
    best_path, best_cost, all_paths = graph.aco(num_ants=100, num_iterations=200, evaporation_rate=0.1, alpha=1, beta=2)
    comp_time = time.time() - start_time

    # In ra thông số benchmark
    print("ACO Results:")
    if best_path:
        print("Best path:", [(node.x, node.y) for node in best_path])
        print("Best cost:", best_cost)
        # print("Path deviation (radians):", calculate_deviation(best_path))
    else:
        print("No path found by ACO.")
    print("Computation time (s):", comp_time)

    # Thiết lập hiển thị
    cell_size = width // grid_size
    font = pygame.font.SysFont("Arial", 18)
    clock = pygame.time.Clock()

    num_iterations = len(all_paths)
    iteration_index = 0
    iteration_delay = 0.1  # thời gian hiển thị mỗi iteration (giây)
    last_update = time.time()
    show_best = False  # Khi các iteration đã hết, chuyển sang hiển thị đường đi tốt nhất

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # screen.fill(WHITE)
        # Vẽ các ô nền (grid, obstacles, start, goal)
        draw_nodes(screen, graph, cell_size)

        if not show_best:
            if iteration_index < num_iterations:
                # Vẽ các đường đi của các con kiến tại iteration hiện tại
                current_paths = all_paths[iteration_index]
                draw_iteration_paths(screen, cell_size, current_paths)
                text_iter = font.render(f"Iteration: {iteration_index+1}/{num_iterations}", True, (0,0,0))
                screen.blit(text_iter, (15, 15))
            else:
                show_best = True
        else:
            # Vẽ đường đi tốt nhất
            draw_best_path(screen, cell_size, best_path)

            text_best = font.render(f"Best cost: {best_cost}", True, (0,0,0))
            screen.blit(text_best, (15, 15))
            text_time = font.render(f"Time: {comp_time:.2f} s", True, (0,0,0))
            screen.blit(text_time, (10, 30))
            save_image(screen, "maze_aco.png")

        pygame.display.flip()
        clock.tick(30)

        # Cập nhật iteration sau delay
        if not show_best and time.time() - last_update > iteration_delay:
            iteration_index += 1
            last_update = time.time()

    pygame.quit()

if __name__ == "__main__":
    main()
