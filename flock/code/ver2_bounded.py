import pygame
import random
import math
from pygame.math import Vector2
import numpy as np

# --------------------------- Configuration ---------------------------
WIDTH, HEIGHT = 1200, 600               # Window size
NUM_BOIDS = 30                         # Number of agents
MAX_SPEED = 100.0                      # Maximum speed (pixels/second)
MAX_FORCE = 300.0                      # Maximum steering force (pixels/second²)
PERCEPTION_RADIUS = 50                 # Neighborhood radius (pixels)
PATH_WAYPOINTS = 6                # Number of waypoints defining the path
WAYPOINT_THRESHOLD = 25                # Distance to switch to next waypoint (pixels)
CELL_SIZE = 100                        # Cell size for entropy calculation (pixels)
PRINT_INTERVAL = 100                   # Frames between metric prints
USE_OBSTACLES = True                   # Toggle obstacles on/off
FPS = 60                               # Target frames per second

# --------------------------- Boid Class ---------------------------
class Boid:
    def __init__(self):
        self.pos = Vector2(random.uniform(0, WIDTH), random.uniform(0, HEIGHT)) # sinh ra một vị trí bất kỳ cho boid 
        angle = random.uniform(0, 2 * math.pi) # góc cũng bất kỳ 
        self.vel = Vector2(math.cos(angle), math.sin(angle)) * MAX_SPEED # vector là instance giúp có thể cộng trừ nhân chia cho ins này tiện hơn cho vận tốc
        self.acc = Vector2(0, 0) # vector gia tốc cho 2 hướng 
        self.current_wp = np.random.choice(PATH_WAYPOINTS) # nó cần có một đích để hướng để, đây mình set ngẫu nhiên để lúc bắt đầu nó không tụm lại một chỗ

    def apply_force(self, force): # hàm này cộng dồn các lực tác động để trở thành gia tốc, không tác động thẳng vào vận tốc
        self.acc += force

    def update(self, dt): # cập nhật trạng thái bầy theo thời gian 
        # Clamp total acceleration
        if self.acc.length() > MAX_FORCE: # nếu gia tốc quá mức thì cutoff , gia tốc đơn vị là pixel/sec^2
            self.acc.scale_to_length(MAX_FORCE)
        # Integrate velocity and position
        self.vel += self.acc * dt # tính giá trị vận tốc bằng tích phân gia tốc 
        if self.vel.length() > MAX_SPEED:
            self.vel.scale_to_length(MAX_SPEED)
        self.pos += self.vel * dt # tích phân gia tốc để ra vị trítrí
        # Reset acceleration & handle boundaries
        self.acc = Vector2(0, 0) # Sau khi tính được vị trí mới thì reset gia tốc (nhưng không reset vận tốc ) 
        self.handle_boundaries() # Cấu hình trường hợp đâm vào tường thì đảo vận tốc theo chiều x, y lại 

    def handle_boundaries(self):
        # Bounce off the edges instead of wrap-around
        if self.pos.x <= 0: #nếu đập vào tường trái thì đảo vận tốc đi ngược 
            self.pos.x = 0
            self.vel.x *= -1
        elif self.pos.x >= WIDTH:
            self.pos.x = WIDTH
            self.vel.x *= -1
        if self.pos.y <= 0:
            self.pos.y = 0
            self.vel.y *= -1
        elif self.pos.y >= HEIGHT:
            self.pos.y = HEIGHT
            self.vel.y *= -1

    def flock(self, boids, path, obstacles): # hàm mô phỏng tổng hợp hành vi thực tế từ nhiều nguồn 
        sep = self.separation(boids) * 300.0 # tách nhau ra 
        ali = self.alignment(boids)   *0.0 
        coh = self.cohesion(boids)    * 0.0
        pat = self.follow_path(path)  * 450.0
        obs = self.avoid_obstacles(obstacles) * 200.0
        for force in (sep, ali, coh, pat, obs):
            self.apply_force(force)

    def separation(self, boids):
        steer = Vector2(0, 0) # gia tốc lái tức thời của một hành vi 
        total = 0 # đếm tổng số flockmate trong vùng nhìn thấy 
        for other in boids: # trong tất cả các flockmate 
            d = self.pos.distance_to(other.pos) 
            if 0 < d < PERCEPTION_RADIUS/2: # chỉ tính những con trong phạm vi nhìn được
                diff = (self.pos - other.pos) / d # tính vector chuẩn hóa = vector/ distance 
                steer += diff #  cộng vào để đẩy khỏi con ở gần 
                total += 1
        if total > 0:
            steer /= total # chia để giữ tổng < 1 
            return self._steer_to(steer) # khuếch đại gia tốc điều khiển thành khoảng giá trị điều khiển 
        return Vector2(0, 0)

    def alignment(self, boids):
        avg_vel = Vector2(0, 0)
        total = 0
        for other in boids:
            if other != self and self.pos.distance_to(other.pos) < PERCEPTION_RADIUS:
                avg_vel += other.vel
                total += 1
        if total > 0:
            avg_vel /= total
            return self._steer_to(avg_vel)
        return Vector2(0, 0)

    def cohesion(self, boids):
        center = Vector2(0, 0)
        total = 0
        for other in boids:
            if other != self and self.pos.distance_to(other.pos) < PERCEPTION_RADIUS:
                center += other.pos
                total += 1
        if total > 0:
            center /= total
            return self.seek(center)
        return Vector2(0, 0)

    def _steer_to(self, vector): # steer theo một vector nào đó
        desired = vector.normalize() * MAX_SPEED # desired là vector hướng đến vị trí mong muốn
        steer = desired - self.vel # steer là vector điều khiển
        if steer.length() > MAX_FORCE: # nếu vector điều khiển lớn hơn lực
            steer.scale_to_length(MAX_FORCE) # thì cắt lại thành lực tối đa
        return steer 

    def seek(self, target): # seek một vector đến vị trí target 
        desired = target - self.pos 
        if desired.length() > 0: 
            desired = desired.normalize() * MAX_SPEED 
            steer = desired - self.vel
            if steer.length() > MAX_FORCE:
                steer.scale_to_length(MAX_FORCE)
            return steer
        return Vector2(0, 0)

    def follow_path(self, path):
        target = path[self.current_wp]
        if self.pos.distance_to(target) < WAYPOINT_THRESHOLD:
            self.current_wp = (self.current_wp + 1) % len(path)
            target = path[self.current_wp]
        return self.seek(target)

    # def avoid_obstacles(self, obstacles): # old 
    #     steer = Vector2(0, 0)
    #     for obs in obstacles:
    #         center = obs['centroid']
    #         r = obs['radius'] + PERCEPTION_RADIUS/2
    #         d = self.pos.distance_to(center)
    #         if d < r:
    #             diff = (self.pos - center).normalize() * MAX_SPEED
    #             s = diff - self.vel
    #             if s.length() > MAX_FORCE:
    #                 s.scale_to_length(MAX_FORCE)
    #             steer += s
    #     return steer
    
    def avoid_obstacles(self, obstacles):
        steer = Vector2(0, 0)
        for obs in obstacles:
            center = obs['centroid']
            r = obs['radius'] + PERCEPTION_RADIUS/2
            d = self.pos.distance_to(center)
        if d > 0:
            strength = MAX_FORCE * (r - d) / r  # càng gần, lực càng lớn (tuyến tính)
            diff = (self.pos - center).normalize() * strength
            s = diff - self.vel
            if s.length() > MAX_FORCE:
                s.scale_to_length(MAX_FORCE)
            steer += s
        return steer

    


    def draw(self, screen):
        angle = self.vel.angle_to(Vector2(1, 0))
        pts = [Vector2(10, 0), Vector2(-5, 5), Vector2(-5, -5)]
        poly = [p.rotate(-angle) + self.pos for p in pts]
        pygame.draw.polygon(screen, (255, 255, 255), poly)

# --------------------------- Metrics ---------------------------
def compute_metrics(boids):
    sum_vel = Vector2(0, 0)
    for b in boids:
        sum_vel += b.vel
    order = sum_vel.length() / (len(boids) * MAX_SPEED)
    cols, rows = WIDTH // CELL_SIZE, HEIGHT // CELL_SIZE
    counts = [0] * (cols * rows)
    for b in boids:
        c = int(min(max(b.pos.x // CELL_SIZE, 0), cols-1))
        r = int(min(max(b.pos.y // CELL_SIZE, 0), rows-1))
        counts[r*cols + c] += 1
    entropy = 0
    N = len(boids)
    for c in counts:
        if c > 0:
            p = c/N
            entropy -= p * math.log(p)
    return order, entropy

# --------------------------- Environment Setup ---------------------------
# def create_obstacles():
#     obs = []
#     for center, rad in [(Vector2(200,300),50), (Vector2(600,200),75)]:
#         obs.append({'type':'circle', 'pos':center, 'radius':rad, 'centroid':center})
#     poly_pts = [Vector2(400,400), Vector2(450,500), Vector2(350,500)]
#     centroid = sum(poly_pts, Vector2(0,0)) / len(poly_pts)
#     rad = max(centroid.distance_to(p) for p in poly_pts)
#     obs.append({'type':'polygon', 'points':poly_pts, 'radius':rad, 'centroid':centroid})
#     return obs

def create_obstacles():
    obs = []

    # --- Circle obstacles (solid circles) ---
    circle_data = [
        (Vector2(200, 300), 50),
        (Vector2(600, 200), 75),
    ]
    for center, radius in circle_data:
        obs.append({
            'type': 'circle',
            'center': center,
            'radius': radius,
        })

    # --- Polygon obstacle (solid polygon) ---
    poly_pts = [
        Vector2(400, 400),
        Vector2(450, 500),
        Vector2(350, 500),
    ]
    obs.append({
        'type': 'polygon',
        'points': poly_pts,
    })

    return obs


def create_path_circle(): # create a circular path
    # Create a circular path with PATH_WAYPOINTS points
    path = []
    center = Vector2(WIDTH/2, HEIGHT/2)
    radius = min(WIDTH, HEIGHT) * 0.4
    for i in range(PATH_WAYPOINTS):
        theta = 2 * math.pi * i / PATH_WAYPOINTS
        path.append(center + Vector2(math.cos(theta), math.sin(theta)) * radius)
    return path
def create_path_diagonal(): #create a diagonal path
    # Create a diagonal path with PATH_WAYPOINTS points
    path = []
    if PATH_WAYPOINTS < 2:
        # Nếu chỉ có 1 điểm, cho luôn về góc trên trái
        return [Vector2(0, 0)]
    # Với i chạy từ 0 đến PATH_WAYPOINTS-1, t chạy từ 0.0 → 1.0
    for i in range(PATH_WAYPOINTS):
        t = i / (PATH_WAYPOINTS - 1)
        x = t * WIDTH
        y = t * HEIGHT
        path.append(Vector2(x, y))
    return path


# --------------------------- Main Simulation ---------------------------
# def run_simulation():
#     pygame.init()
#     screen = pygame.display.set_mode((WIDTH, HEIGHT))
#     clock = pygame.time.Clock()
#     boids = [Boid() for _ in range(NUM_BOIDS)]
#     obstacles = create_obstacles() if USE_OBSTACLES else []
#     path = create_path_circle()
#     frame = 0
#     running = True
#     while running:
#         dt = clock.tick(FPS) / 1000.0
#         for event in pygame.event.get():
#             if event.type == pygame.QUIT:
#                 running = False
#         screen.fill((30, 30, 30))
#         # Draw boundary walls
#         pygame.draw.rect(screen, (200,200,200), (0, 0, WIDTH, HEIGHT), 2)
#         # Draw path
#         for wp in path:
#             pygame.draw.circle(screen, (50,50,50), (int(wp.x), int(wp.y)), 3)
#         # Draw obstacles
#         for obs in obstacles:
#             if obs['type']=='circle':
#                 pygame.draw.circle(screen, (200,50,50), (int(obs['pos'].x), int(obs['pos'].y)), obs['radius'], 2)
#             else:
#                 pts = [(int(p.x), int(p.y)) for p in obs['points']]
#                 pygame.draw.polygon(screen, (200,50,50), pts, 2)
#         # Update & draw boids
#         for b in boids:
#             b.flock(boids, path, obstacles)
#             b.update(dt)
#             b.draw(screen)
#         if frame % PRINT_INTERVAL == 0:
#             order, entropy = compute_metrics(boids)
#             print(f"Frame {frame}: Order={order:.3f}, Entropy={entropy:.3f}")
#         pygame.display.flip()
#         frame += 1
#     pygame.quit()

def run_simulation():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    clock = pygame.time.Clock()
    boids = [Boid() for _ in range(NUM_BOIDS)]
    obstacles = create_obstacles() if USE_OBSTACLES else []
    path = create_path_circle()
    frame = 0
    running = True

    while running:
        dt = clock.tick(FPS) / 1000.0
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill((30, 30, 30))
        pygame.draw.rect(screen, (200,200,200), (0, 0, WIDTH, HEIGHT), 2)
        for wp in path:
            pygame.draw.circle(screen, (50,50,50), (int(wp.x), int(wp.y)), 3)

        # ---- Draw obstacles ----
        for obs in obstacles:
            if obs['type'] == 'circle':
                # Hỗ trợ cả key 'center' và 'pos'
                c = obs.get('center') or obs.get('pos')
                pygame.draw.circle(
                    screen,
                    (200,50,50),
                    (int(c.x), int(c.y)),
                    obs['radius'],
                    0   # width=0 để vẽ khối đặc, đổi thành >0 nếu chỉ muốn viền
                )
            elif obs['type'] == 'polygon':
                pts = [(int(p.x), int(p.y)) for p in obs['points']]
                pygame.draw.polygon(
                    screen,
                    (200,50,50),
                    pts,
                    0   # width=0 để vẽ khối đặc
                )

        # ---- Update & draw boids ----
        for b in boids:
            b.flock(boids, path, obstacles)
            b.update(dt)
            b.draw(screen)

        if frame % PRINT_INTERVAL == 0:
            order, entropy = compute_metrics(boids)
            print(f"Frame {frame}: Order={order:.3f}, Entropy={entropy:.3f}")

        pygame.display.flip()
        frame += 1

    pygame.quit()


if __name__ == '__main__':
    run_simulation()
