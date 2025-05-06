import pygame
import random
import math
from pygame.math import Vector2
import numpy as np
import time 
import csv
import os   
import sys

# --------------------------- Configuration ---------------------------
WIDTH, HEIGHT = 1200, 600               # Window size
NUM_BOIDS = 10                          # Number of agents
MAX_SPEED = 150.0                       # Maximum speed (pixels/second)
MAX_FORCE = 500.0                       # Maximum steering force (pixels/second²)
PERCEPTION_RADIUS = 100                  # Neighborhood radius (pixels)
WAYPOINT_THRESHOLD = 40                 # Distance to switch to next waypoint (pixels)
PRINT_INTERVAL = 100                    # Frames between metric prints
USE_OBSTACLES = True                    # Toggle obstacles on/off
FPS = 60                                # Target frames per second
CELL_SIZE = 100                         # Size of each cell for entropy calculation
SEED = 23                               # Seed for reproducible randomness
NUM_PATH_WPS = 2

# Base gains for behaviors
BASE_GAINS = {
    'separation': 230.0,
    'alignment': 200.0,
    'cohesion': 200.0,
    'path': 300.0,
    'obstacle': 300.0
}

# --------------------------- Obstacle Setup ---------------------------
def create_obstacles():
    obs = []
    circle_data = [
        # (Vector2(200, 300), 50),
        (Vector2(600, 300), 75),
    ]
    for center, radius in circle_data:
        obs.append({'type': 'circle', 'center': center, 'radius': radius})
    # poly_pts = [Vector2(400, 400), Vector2(450, 500), Vector2(350, 500)] # tạo vật cản dạng polygon 
    # obs.append({'type': 'polygon', 'points': poly_pts})
    return obs


def build_occupancy(obstacles):
    occupied = []
    from pygame import Surface, draw
    for o in obstacles:
        if o['type'] == 'circle':
            cx, cy = int(o['center'].x), int(o['center'].y)
            r = int(o['radius'])
            for dx in range(-r, r+1):
                for dy in range(-r, r+1):
                    if dx*dx + dy*dy <= r*r:
                        occupied.append(Vector2(cx+dx, cy+dy))
        else:
            pts = [(int(p.x), int(p.y)) for p in o['points']]
            min_x = max(min(x for x, y in pts), 0)
            max_x = min(max(x for x, y in pts), WIDTH)
            min_y = max(min(y for x, y in pts), 0)
            max_y = min(max(y for x, y in pts), HEIGHT)
            mask_surf = Surface((max_x-min_x+1, max_y-min_y+1), pygame.SRCALPHA)
            draw.polygon(mask_surf, (255,255,255), [(x-min_x, y-min_y) for x, y in pts])
            mask = pygame.mask.from_surface(mask_surf)
            for x in range(max_x-min_x+1):
                for y in range(max_y-min_y+1):
                    if mask.get_at((x, y)):
                        occupied.append(Vector2(min_x+x, min_y+y))
    return occupied

# --------------------------- Boid Class ---------------------------
class Boid:
    def __init__(self):
        self.pos = Vector2(random.uniform(0, WIDTH/10), random.uniform(0, HEIGHT/10))
        angle = random.uniform(0, 2*math.pi)
        self.vel = Vector2(math.cos(angle), math.sin(angle)) * MAX_SPEED
        self.acc = Vector2(0, 0)
        self.current_wp = 0

    def apply_force(self, force):
        self.acc += force

    def update(self, dt):
        if self.acc.length() > MAX_FORCE:
            self.acc.scale_to_length(MAX_FORCE)
        self.vel += self.acc * dt
        if self.vel.length() > MAX_SPEED:
            self.vel.scale_to_length(MAX_SPEED)
        self.pos += self.vel * dt
        self.acc = Vector2(0,0)
        self.handle_boundaries()

    def handle_boundaries(self):
        if self.pos.x < 0: self.pos.x, self.vel.x = 0, -self.vel.x
        if self.pos.x > WIDTH: self.pos.x, self.vel.x = WIDTH, -self.vel.x
        if self.pos.y < 0: self.pos.y, self.vel.y = 0, -self.vel.y
        if self.pos.y > HEIGHT: self.pos.y, self.vel.y = HEIGHT, -self.vel.y

    def flock(self, boids, waypoints, occupied):
        # compute behaviors
        sep = self.separation(boids)
        ali = self.alignment(boids)
        coh = self.cohesion(boids)
        target, reached = self.follow_path(waypoints)
        pat = self.seek(target)
        obsf = self.avoid_occupancy(occupied)
        # dynamic weighting based on proximity to obstacle
        min_d = min((self.pos.distance_to(p) for p in occupied), default=PERCEPTION_RADIUS)
        urgency = max(0.0, (PERCEPTION_RADIUS - min_d) / PERCEPTION_RADIUS)
        gains = BASE_GAINS.copy()
        gains['obstacle'] *= 1.0 + urgency * 2.0
        scale_others = 1.0 - urgency
        for key in ('separation','alignment','cohesion','path'):
            gains[key] *= scale_others
        # apply forces
        for force, key in zip((sep, ali, coh, pat, obsf),
                              ('separation','alignment','cohesion','path','obstacle')):
            self.apply_force(force * gains[key])

    def separation(self, boids):
        steer = Vector2(0,0); total=0
        for other in boids:
            d = self.pos.distance_to(other.pos)
            if 0 < d < PERCEPTION_RADIUS/2:
                steer += (self.pos - other.pos)/d; total += 1
        if total > 0:
            steer /= total
            return self._steer_to(steer)
        return Vector2(0,0)

    def alignment(self, boids):
        avg_vel = Vector2(0,0); total=0
        for other in boids:
            if other != self and self.pos.distance_to(other.pos) < PERCEPTION_RADIUS:
                avg_vel += other.vel; total += 1
        if total > 0:
            avg_vel /= total
            return self._steer_to(avg_vel)
        return Vector2(0,0)

    def cohesion(self, boids):
        center = Vector2(0,0); total=0
        for other in boids:
            if other != self and self.pos.distance_to(other.pos) < PERCEPTION_RADIUS:
                center += other.pos; total += 1
        if total > 0:
            center /= total
            return self.seek(center)
        return Vector2(0,0)

    def follow_path(self, pts):
        target = pts[self.current_wp]
        if self.pos.distance_to(target) < WAYPOINT_THRESHOLD:
            self.current_wp = (self.current_wp + 1) % len(pts)
            return target, True
        return target, False

    def avoid_occupancy(self, occupied):
        steer = Vector2(0,0); count=0
        for p in occupied:
            d = self.pos.distance_to(p)
            if d < PERCEPTION_RADIUS:
                strength = MAX_FORCE * (PERCEPTION_RADIUS - d) / PERCEPTION_RADIUS
                steer += (self.pos - p).normalize() * strength; count += 1
        if count > 0:
            steer /= count
            if steer.length() > MAX_FORCE:
                steer.scale_to_length(MAX_FORCE)
        return steer

    def seek(self, target):
        desired = target - self.pos
        if desired.length() > 0:
            desired = desired.normalize() * MAX_SPEED
            steer = desired - self.vel
            if steer.length() > MAX_FORCE:
                steer.scale_to_length(MAX_FORCE)
            return steer
        return Vector2(0,0)

    def _steer_to(self, vec):
        desired = vec.normalize() * MAX_SPEED
        steer = desired - self.vel
        if steer.length() > MAX_FORCE:
            steer.scale_to_length(MAX_FORCE)
        return steer

    def draw(self, screen):
        pts = [Vector2(10,0), Vector2(-5,5), Vector2(-5,-5)]
        ang = self.vel.angle_to(Vector2(1,0))
        poly = [p.rotate(-ang) + self.pos for p in pts]
        pygame.draw.polygon(screen, (255,255,255), poly)


def compute_metrics(boids):
    # Tính toán chỉ số order
    sum_vel = Vector2(0, 0)
    for b in boids:
        sum_vel += b.vel
    order = sum_vel.length() / (len(boids) * MAX_SPEED)

    # Tính toán entropy
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
            p = c / N
            entropy -= p * math.log(p)

    return order, entropy

# --------------------------- Main Simulation ---------------------------
# def run_simulation(): #old version not have timer for end process 
#     pygame.init()
#     random.seed(SEED); np.random.seed(SEED)
#     screen = pygame.display.set_mode((WIDTH,HEIGHT))
#     font = pygame.font.SysFont(None,24)
#     clock = pygame.time.Clock()

#     boids = [Boid() for _ in range(NUM_BOIDS)]
#     obstacles = create_obstacles() if USE_OBSTACLES else []
#     occupied = build_occupancy(obstacles)
#     # path = [Vector2(random.uniform(0,WIDTH), random.uniform(0,HEIGHT)) for _ in range(NUM_PATH_WPS)]
#     path = [Vector2(70, 60), Vector2(1100, 500)]

#     frame = 0; running = True
#     while running:
#         dt = clock.tick(FPS) / 1000
#         for e in pygame.event.get():
#             if e.type == pygame.QUIT:
#                 running = False
#         screen.fill((30,30,30))
#         # draw obstacles
#         for o in obstacles:
#             if o['type'] == 'circle':
#                 c, r = o['center'], o['radius']
#                 pygame.draw.circle(screen, (200,50,50), (int(c.x), int(c.y)), r)
#             else:
#                 pts = [(int(p.x), int(p.y)) for p in o['points']]
#                 pygame.draw.polygon(screen, (200,50,50), pts)
#         # draw waypoints
#         for idx, wp in enumerate(path,1):
#             color = (50,200,50) if idx-1 == boids[0].current_wp else (100,200,100)
#             pygame.draw.circle(screen, color, (int(wp.x), int(wp.y)), 6)
#             screen.blit(font.render(str(idx), True, (255,255,255)), (wp.x+8, wp.y-8))
#         # update & draw boids
#         for b in boids:
#             b.flock(boids, path, occupied)
#             b.update(dt)
#             b.draw(screen)
#         pygame.display.flip(); frame += 1
#     pygame.quit()

def run_simulation():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    font = pygame.font.SysFont(None, 24)
    clock = pygame.time.Clock()
    boids = [Boid() for _ in range(NUM_BOIDS)]
    obstacles = create_obstacles() if USE_OBSTACLES else []
    #path = [Vector2(random.uniform(0,WIDTH), random.uniform(0,HEIGHT)) for _ in range(NUM_PATH_WPS)]
    path = [Vector2(70, 60), Vector2(1100, 500)]
    frame = 0
    running = True
    total_time = 300  # Total time in seconds for each trial
    trials = 5  # Number of trials to run
    trial_num = 1  # Start from trial 1

    while trials > 0:
        # Generate new file name for each trial
        file_name = f"trial{trial_num}.csv"

        # Open the CSV file and write header
        with open(file_name, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Time (s)", "Order", "Entropy"])

        start_time = time.time()
        print(f"Starting Trial {trial_num}...")

        while time.time() - start_time < total_time:  # Run trial for total_time seconds
            dt = clock.tick(FPS) / 1000.0
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            screen.fill((30, 30, 30))
            pygame.draw.rect(screen, (200,200,200), (0, 0, WIDTH, HEIGHT), 2)
            for wp in path:
                pygame.draw.circle(screen, (50,50,50), (int(wp.x), int(wp.y)), 3)

            # Draw obstacles
            for obs in obstacles:
                if obs['type'] == 'circle':
                    c = obs.get('center') or obs.get('pos')
                    pygame.draw.circle(screen, (200,50,50), (int(c.x), int(c.y)), obs['radius'], 0)
                elif obs['type'] == 'polygon':
                    pts = [(int(p.x), int(p.y)) for p in obs['points']]
                    pygame.draw.polygon(screen, (200,50,50), pts, 0)

            # Update & draw boids
            for b in boids:
                b.flock(boids, path, obstacles)
                b.update(dt)
                b.draw(screen)

            # Log metrics to CSV after each frame
            if frame % PRINT_INTERVAL == 0:
                log_metrics_to_csv(boids, frame, file_name)

            pygame.display.flip()
            frame += 1

        # After trial ends, check if more trials remain
        trials -= 1
        trial_num += 1  # Increment trial number

        # If there are more trials, restart the script for the next trial
        if trials > 0:
            print(f"Trial {trial_num - 1} completed. Starting trial {trial_num}...")
            os.execl(sys.executable, sys.executable, *sys.argv)  # Restart the script

    pygame.quit()


if __name__ == '__main__':
    run_simulation()