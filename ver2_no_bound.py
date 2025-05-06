import pygame
import random
import math
from pygame.math import Vector2

# --------------------------- Configuration ---------------------------
WIDTH, HEIGHT = 800, 600               # Window size
NUM_BOIDS = 50                         # Number of agents
MAX_SPEED = 200.0                      # Maximum speed (pixels/second)
MAX_FORCE = 300.0                      # Maximum steering force (pixels/second²)
PERCEPTION_RADIUS = 50                 # Neighborhood radius (pixels)
PATH_WAYPOINTS = 36                    # Number of waypoints defining the path
WAYPOINT_THRESHOLD = 15                # Distance to switch to next waypoint (pixels)
CELL_SIZE = 100                        # Cell size for entropy calculation (pixels)
PRINT_INTERVAL = 100                   # Frames between metric prints
USE_OBSTACLES = False                   # Toggle obstacles on/off
FPS = 60                               # Target frames per second

# --------------------------- Boid Class ---------------------------
class Boid:
    def __init__(self):
        self.pos = Vector2(random.uniform(0, WIDTH), random.uniform(0, HEIGHT))
        angle = random.uniform(0, 2 * math.pi)
        self.vel = Vector2(math.cos(angle), math.sin(angle)) * MAX_SPEED
        self.acc = Vector2(0, 0)
        self.current_wp = 0

    def apply_force(self, force):
        self.acc += force

    def update(self, dt): 
        # update the position following the velocity and acceleration
        # Clamp total acceleration
        if self.acc.length() > MAX_FORCE:
            self.acc.scale_to_length(MAX_FORCE)
        # Integrate velocity and position
        self.vel += self.acc * dt
        if self.vel.length() > MAX_SPEED:
            self.vel.scale_to_length(MAX_SPEED)
        self.pos += self.vel * dt
        # Reset acceleration
        self.acc = Vector2(0, 0) # after applying the forces reset the acceleration
        self.edges() # align the edges of the screen

    def edges(self):
        # Wrap around the screen
        if self.pos.x > WIDTH:  self.pos.x = 0
        if self.pos.x < 0:      self.pos.x = WIDTH
        if self.pos.y > HEIGHT: self.pos.y = 0
        if self.pos.y < 0:      self.pos.y = HEIGHT

    def flock(self, boids, path, obstacles):
        # Compute behavior forces
        sep = self.separation(boids) * 200.0
        ali = self.alignment(boids)   * 100.0
        coh = self.cohesion(boids)    * 100.0
        pat = self.follow_path(path)  * 200.0
        obs = self.avoid_obstacles(obstacles) * 300.0
        pat = self.follow_path(path)  * 200.0
        # Apply
        for force in (sep, ali, coh, pat, obs):
            self.apply_force(force)

    def separation(self, boids):
        steer = Vector2(0, 0)
        total = 0
        for other in boids:
            d = self.pos.distance_to(other.pos)
            if 0 < d < PERCEPTION_RADIUS/2:
                diff = (self.pos - other.pos) / d
                steer += diff
                total += 1
        if total > 0:
            steer /= total
            return self._steer_to(steer)
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

    def _steer_to(self, vector):
        desired = vector.normalize() * MAX_SPEED
        steer = desired - self.vel
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
        return Vector2(0, 0)

    def follow_path(self, path):
        target = path[self.current_wp]
        if self.pos.distance_to(target) < WAYPOINT_THRESHOLD:
            self.current_wp = (self.current_wp + 1) % len(path)
            target = path[self.current_wp]
        return self.seek(target)

    def avoid_obstacles(self, obstacles):
        steer = Vector2(0, 0)
        for obs in obstacles:
            # Use centroid for both circle & polygon
            center = obs['centroid']
            r = obs['radius'] + PERCEPTION_RADIUS/2
            d = self.pos.distance_to(center)
            if d < r:
                diff = (self.pos - center).normalize() * MAX_SPEED
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
        if c>0:
            p = c/N
            entropy -= p * math.log(p)
    return order, entropy

# --------------------------- Environment Setup ---------------------------
def create_obstacles():
    obs = []
    # Circle obstacles
    for center, rad in [(Vector2(200,300),50), (Vector2(600,200),75)]:
        obs.append({'type':'circle', 'pos':center, 'radius':rad, 'centroid':center})
    # Polygon obstacle example (triangle)
    poly_pts = [Vector2(400,400), Vector2(450,500), Vector2(350,500)]
    centroid = sum(poly_pts, Vector2(0,0)) / len(poly_pts)
    rad = max(centroid.distance_to(p) for p in poly_pts)
    obs.append({'type':'polygon', 'points':poly_pts, 'radius':rad, 'centroid':centroid})
    return obs

def create_path():
    path = []
    center = Vector2(WIDTH/2, HEIGHT/2)
    radius = min(WIDTH, HEIGHT) * 0.4
    for i in range(PATH_WAYPOINTS):
        theta = 2 * math.pi * i / PATH_WAYPOINTS
        path.append(center + Vector2(math.cos(theta), math.sin(theta)) * radius)
    return path

# --------------------------- Main Simulation ---------------------------
def run_simulation():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    clock = pygame.time.Clock()

    boids = [Boid() for _ in range(NUM_BOIDS)]
    obstacles = create_obstacles() if USE_OBSTACLES else []
    path = create_path()
    frame = 0
    running = True
    while running:
        dt = clock.tick(FPS) / 1000.0
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        screen.fill((30, 30, 30))
        # Draw path
        for wp in path:
            pygame.draw.circle(screen, (50,50,50), (int(wp.x), int(wp.y)), 3)
        # Draw obstacles
        for obs in obstacles:
            if obs['type']=='circle':
                pygame.draw.circle(screen, (200,50,50), (int(obs['pos'].x), int(obs['pos'].y)), obs['radius'], 2)
            else:
                pts = [(int(p.x), int(p.y)) for p in obs['points']]
                pygame.draw.polygon(screen, (200,50,50), pts, 2)
        # Update & draw boids
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
