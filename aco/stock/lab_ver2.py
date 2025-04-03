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
        self.h_score = float('inf') # heuristic
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

        # Khởi tạo từ file JSON hoặc ngẫu nhiên
        if json_file and os.path.exists(json_file):
            self.load_from_json(json_file)
        else:
            # Khởi tạo tất cả ô là đường đi với cost ngẫu nhiên
            for x in range(grid_size):
                for y in range(grid_size):
                    cost = 1  # Mặc định cost là 1
                    self.nodes[(x, y)] = Node(x, y, is_obstacle=False, cost=cost)

            # Đặt điểm start và goal
            self.set_start(0, 0)  # Start ở góc trên trái
            self.goal = self.get_random_free_cell()
            while self.goal == self.start:  # Đảm bảo goal khác start
                self.goal = self.get_random_free_cell()

        # Thiết lập neighbors
        for x in range(grid_size):
            for y in range(grid_size):
                current_node = self.nodes[(x, y)]
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < grid_size and 0 <= ny < grid_size: # nếu không kịch trần thì thêm class node vào list hàng xóm của nono
                        current_node.add_neighbor(self.nodes[(nx, ny)])

        if not self.goal: # nếu chưa có hàng xóm thì lấy một điểm ngẫu nhiên 
            self.goal = self.get_random_free_cell()

    def load_from_json(self, json_file): # đọc bản đồ từ file json thôi không có gìgì
        with open(json_file, 'r') as f:
            data = json.load(f)
            maze_data = data.get("data")  # Hoặc tên key tương ứng trong file của bạn

            # Tải dữ liệu từ ma trận
            for x in range(self.grid_size):
                for y in range(self.grid_size):
                    is_obstacle = (maze_data[x][y] == 1)
                    cost = float('inf') if is_obstacle else 1  # Vật cản có cost vô cùng
                    self.nodes[(x, y)] = Node(x, y, is_obstacle, cost)

    def get_node(self, x, y): # lấy instance node theo vị trí 
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
        open_set = []
        closed_set = set()
        self.start.g_score = 0
        self.start.f_score = abs(self.start.x - self.goal.x) + abs(self.start.y - self.goal.y) # tính khoảng cách MahattanMahattan
        heappush(open_set, (self.start.f_score, id(self.start), self.start))
        
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

    # Vẽ grid với màu dựa trên trạng thái
    for x in range(grid_size):
        for y in range(grid_size):
            node = graph.get_node(x, y)

            if node == graph.start:
                color = (0, 255, 255)  # Màu xanh nước biển cho start
            elif node == graph.goal:
                color = (255, 0, 0)  # Màu đỏ cho goal
            elif (x, y) in graph.path_log[:timestep + 1]:  # Màu vàng cho điểm đã đi qua
                color = (255, 255, 0)
            elif node.is_obstacle:
                color = (0, 0, 0)  # Màu đen cho vật cản
            else:
                color = get_cost_color(node.cost, graph.max_cost)  # Màu gradient cho cost

            pygame.draw.rect(screen, color, (x * cell_size, y * cell_size, cell_size, cell_size))
            pygame.draw.rect(screen, (50, 50, 50), (x * cell_size, y * cell_size, cell_size, cell_size), 1)

    # Vẽ log theo timestep (Đường đi ngắn nhất)
    if timestep == len(graph.path_log) - 1:  # Nếu đã khám phá được điểm đích, vẽ đường đi ngắn nhất
        node = graph.goal
        while node:
            pygame.draw.circle(screen, (128,0,128), (node.x * cell_size + cell_size // 2, node.y * cell_size + cell_size // 2), cell_size // 4)
            node = node.parent
        save_image(screen, f"maze_final.png")

def save_image(screen, filename):
    pygame.image.save(screen, filename)

def main():
    pygame.init()
    grid_size = 25
    width, height = 600, 600
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("Maze Path Planning")

    # Tạo graph (có thể dùng JSON hoặc random)
    graph = Graph(grid_size, json_file="map/none.json")  # Hoặc json_file="maze.json"
    graph.set_start(1, 1)
    graph.set_goal(15, 10)

    # Tìm đường bằng A*
    path, log = graph.a_star()
    graph.path_log = log  # Lưu log để vẽ từng bước

    clock = pygame.time.Clock()
    running = True
    timestep = 0
    start = time.time()
    is_paused = False  # Biến điều khiển trạng thái pause
    search_done = False  # Biến để kiểm tra xem tìm kiếm đã hoàn thành chưa

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            # Kiểm tra sự kiện bấm phím space để pause hoặc tiếp tục
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    is_paused = not is_paused  # Đảo trạng thái pause

        # Nếu tìm kiếm chưa hoàn thành và không tạm dừng, tiếp tục tìm kiếm
        if not search_done and not is_paused:
            if time.time() - start > 0.1:
                timestep = (timestep + 1) % (len(graph.path_log) + 1)  # Cập nhật timestep
                start = time.time()  # Reset thời gian bắt đầu

                # Kiểm tra nếu tìm xong đường đi ngắn nhất
                if timestep == len(graph.path_log):  # Đã vẽ xong đường đi ngắn nhất
                    search_done = True  # Đánh dấu tìm kiếm đã hoàn thành

        # Vẽ bản đồ
        
        pygame.display.flip()
        clock.tick(10)  # Điều chỉnh tốc độ vẽ
        draw_map(graph, grid_size, screen, timestep)

        # Lưu ảnh tại timestep cuối cùng (đường đi ngắn nhất)
        # if search_done and timestep == len(graph.path_log):
        #     save_image(screen, f"maze_final.png")
        #     break  # Dừng lại sau khi lưu ảnh

    pygame.quit()

if __name__ == "__main__":
    main()
