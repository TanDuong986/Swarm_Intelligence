import random
import pygame
import json
import os
from heapq import heappush, heappop
import time

# Hàm tạo màu dựa trên cost
def get_cost_color(cost, max_cost):
    if cost == float('inf'):
        return (0, 0, 0)  # Đen cho vật cản
    # Chuyển cost thành gradient từ xanh (thấp) đến đỏ (cao)
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
        self.g_score = float('inf')  # Dùng cho A*
        self.f_score = float('inf')  # Dùng cho A*


    def add_neighbor(self, neighbor):
        self.neighbors.append(neighbor)

class Graph:
    def __init__(self, grid_size, obstacle_ratio=0.3, json_file=None):
        self.grid_size = grid_size
        self.nodes = {} # directory gom cac node
        self.start = None
        self.goal = None
        self.path_log = []
        self.max_cost = grid_size * 2

        # Khởi tạo từ file JSON hoặc ngẫu nhiên
        if json_file and os.path.exists(json_file):
            self.load_from_json(json_file)
        else: # sinh ra bản đồ và đảm bảo có đường đến đích 
            # Khởi tạo tất cả ô là đường đi với cost ngẫu nhiên
            for x in range(grid_size):
                for y in range(grid_size):
                    # cost = random.randint(1, 10)
                    cost = 1
                    self.nodes[(x, y)] = Node(x, y, is_obstacle=False, cost=cost)

            # Đặt điểm start và goal trước
            self.set_start(0, 0)  # Start ở góc trên trái
            self.goal = self.get_random_free_cell()
            while self.goal == self.start:  # Đảm bảo goal khác start
                self.goal = self.get_random_free_cell()

            # Tạo một đường đi cơ bản từ start đến goal
            current_x, current_y = self.start.x, self.start.y
            goal_x, goal_y = self.goal.x, self.goal.y
            while current_x != goal_x or current_y != goal_y:
                if current_x < goal_x:
                    current_x += 1
                elif current_x > goal_x:
                    current_x -= 1
                if current_y < goal_y:
                    current_y += 1
                elif current_y > goal_y:
                    current_y -= 1
                self.nodes[(current_x, current_y)].is_obstacle = False  # Đảm bảo đường đi không bị chặn
                # self.nodes[(current_x, current_y)].cost = random.randint(1, 10)
                self.nodes[(current_x, current_y)].cost = 1

            # Thêm vật cản ngẫu nhiên nhưng không chặn đường cơ bản
            for x in range(grid_size):
                for y in range(grid_size):
                    if random.random() < obstacle_ratio and (x, y) != (self.start.x, self.start.y) and (x, y) != (self.goal.x, self.goal.y):
                        # Kiểm tra xem ô này có nằm trên đường cơ bản không
                        is_on_basic_path = False
                        temp_x, temp_y = self.start.x, self.start.y
                        while temp_x != goal_x or temp_y != goal_y:
                            if (temp_x, temp_y) == (x, y):
                                is_on_basic_path = True
                                break
                            if temp_x < goal_x:
                                temp_x += 1
                            elif temp_x > goal_x:
                                temp_x -= 1
                            if temp_y < goal_y:
                                temp_y += 1
                            elif temp_y > goal_y:
                                temp_y -= 1
                        if not is_on_basic_path:
                            self.nodes[(x, y)].is_obstacle = True
                            self.nodes[(x, y)].cost = float('inf')

        # Thiết lập neighbors
        for x in range(grid_size):
            for y in range(grid_size):
                current_node = self.nodes[(x, y)]
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < grid_size and 0 <= ny < grid_size:
                        current_node.add_neighbor(self.nodes[(nx, ny)])

        if not self.goal:
            self.goal = self.get_random_free_cell()

    def load_from_json(self, json_file):
        with open(json_file, 'r') as f:
            data = json.load(f)
            # Nếu JSON có key (ví dụ: "easy_maze"), lấy ma trận từ key đó
            if isinstance(data, dict):
                # Giả sử key là "easy_maze", thay đổi nếu tên key khác
                maze_data = data.get("data")  # Hoặc tên key tương ứng trong file của bạn
            else:
                maze_data = data  # Nếu JSON là ma trận trực tiếp

            # Kiểm tra xem maze_data có phải là ma trận 25x25 không
            if not maze_data or len(maze_data) != self.grid_size or len(maze_data[0]) != self.grid_size:
                raise ValueError("JSON file must contain a 25x25 maze matrix")

            # Tải dữ liệu từ ma trận
            for x in range(self.grid_size):
                for y in range(self.grid_size):
                    is_obstacle = (maze_data[x][y] == 1)
                    cost = float('inf') if is_obstacle else random.randint(1, 10)
                    self.nodes[(x, y)] = Node(x, y, is_obstacle, cost)

    def get_node(self, x, y):
        return self.nodes.get((x, y)) # lay node o toa do (a,b)

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
        open_set = []
        closed_set = set()
        self.start.g_score = 0  # cost
        self.start.f_score = abs(self.start.x - self.goal.x) + abs(self.start.y - self.goal.y) #khoang cach Mahattan | heuristic
        
        log = []  # Lưu log các bước

        while open_set:
            _, _, current = heappop(open_set)
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

        return [], log  # Không tìm thấy đường

def draw_map(graph, grid_size, screen, timestep=-1):
    cell_size = 600 // grid_size
    screen.fill((255, 255, 255))  # Nền trắng

    # Vẽ grid với màu dựa trên cost
    for x in range(grid_size):
        for y in range(grid_size):
            node = graph.get_node(x, y) # lay node tu danh sach node 
            color = get_cost_color(node.cost, graph.max_cost) # tinh toán màu sắc của ô dựa trên obstacle của nó 
            pygame.draw.rect(screen, color, (x * cell_size, y * cell_size, cell_size, cell_size)) # vẽ các ô màu nền 
            pygame.draw.rect(screen, (50, 50, 50), (x * cell_size, y * cell_size, cell_size, cell_size), 1) # vẽ các ô viền màu xám

    # Vẽ start và goal
    if graph.start: # Nếu là ô bắt đầu thì vẽ thêm một chấm tròn màu xanh lá 
        pygame.draw.circle(screen, (0, 255, 0), (graph.start.x * cell_size + cell_size // 2, graph.start.y * cell_size + cell_size // 2), cell_size // 4)
    if graph.goal: # Nếu là ô kết thúc thì thêm chấm tròn màu đỏđỏ
        pygame.draw.circle(screen, (255, 0, 0), (graph.goal.x * cell_size + cell_size // 2, graph.goal.y * cell_size + cell_size // 2), cell_size // 4)

    # Vẽ log theo timestep
    if timestep >= 0 and timestep < len(graph.path_log):
        for i, (x, y) in enumerate(graph.path_log[:timestep + 1]):
            pygame.draw.circle(screen, (0, 255, 255), (x * cell_size + cell_size // 2, y * cell_size + cell_size // 2), cell_size // 4)

def save_image(screen, filename):
    pygame.image.save(screen, filename)

def main():
    pygame.init()
    grid_size = 25
    width, height = 600, 600
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("Maze Path Planning")

    # Tạo graph (có thể dùng JSON hoặc random)
    # Ví dụ JSON: [[0, 1, 0], [0, 0, 1], [1, 0, 0]] (0 là đường, 1 là vật cản)
    graph = Graph(grid_size,json_file="map\hand_craft.json")  # Hoặc json_file="maze.json"
    graph.set_start(1, 1)
    # graph.set_goal(graph.goal.x, graph.goal.y)
    graph.set_goal(15,10)

    # Tìm đường bằng A*
    path, log = graph.a_star()
    graph.path_log = log  # Lưu log để vẽ từng bước

    clock = pygame.time.Clock()
    running = True
    timestep = 0
    start = time.time()
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            
            # elif event.type == pygame.KEYDOWN:
            #     if event.key == pygame.K_SPACE:
            #         timestep = (timestep + 1) % (len(graph.path_log) + 1)

        if time.time() - start > 0.1:
            timestep = (timestep + 1) % (len(graph.path_log) + 1)  # Cập nhật timestep
            start = time.time()  # Reset thời gian bắt đầu


        draw_map(graph, grid_size, screen, timestep)
        pygame.display.flip()
        clock.tick(10)  # Điều chỉnh tốc độ vẽ

        # Lưu ảnh tại timestep hiện tại (tùy chọn)
        if timestep == len(graph.path_log):
            save_image(screen, f"maze_final.png")

    pygame.quit()

if __name__ == "__main__":
    main()