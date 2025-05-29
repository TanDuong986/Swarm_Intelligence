import pygame
import numpy as np
from scipy.interpolate import CubicSpline
import sys

# ==================== PSO and Path Planning Core ====================
class Particle:
    def __init__(self, position, velocity, cost, details):
        self.position = position
        self.velocity = velocity
        self.cost = cost
        self.details = details
        self.best_position = position.copy()
        self.best_cost = cost
        self.best_details = details

class PSO:
    def __init__(self, num_vars, var_min, var_max, cost_function,
                 pop_size=50, c1=1.5, c2=1.5, inertia=0.8, wdamp=0.99, reset_interval=None):
        self.num_vars = num_vars
        self.var_min = var_min
        self.var_max = var_max
        self.cost_function = cost_function
        self.pop_size = pop_size
        self.c1 = c1
        self.c2 = c2
        self.inertia = inertia
        self.wdamp = wdamp
        self.reset_interval = reset_interval
        self.swarm = []
        self.global_best_pos = None
        self.global_best_cost = float('inf')
        self.global_best_details = None

    def initialize_swarm(self):
        self.swarm = []
        for _ in range(self.pop_size):
            pos = np.random.uniform(self.var_min, self.var_max, self.num_vars)
            vel = np.zeros(self.num_vars)
            cost, details = self.cost_function(pos)
            p = Particle(pos, vel, cost, details)
            self.swarm.append(p)
            if cost < self.global_best_cost:
                self.global_best_cost = cost
                self.global_best_pos = pos.copy()
                self.global_best_details = details

    def update_particle(self, p):
        r1 = np.random.rand(self.num_vars)
        r2 = np.random.rand(self.num_vars)
        p.velocity = (
            self.inertia * p.velocity
            + self.c1 * r1 * (p.best_position - p.position)
            + self.c2 * r2 * (self.global_best_pos - p.position)
        )
        p.position += p.velocity
        p.position = np.clip(p.position, self.var_min, self.var_max)
        cost, details = self.cost_function(p.position)
        p.cost = cost
        p.details = details
        if cost < p.best_cost:
            p.best_cost = cost
            p.best_position = p.position.copy()
            p.best_details = details
            if cost < self.global_best_cost:
                self.global_best_cost = cost
                self.global_best_pos = p.position.copy()
                self.global_best_details = details

    def run(self, max_iter=100, callback=None):
        self.initialize_swarm()
        for i in range(1, max_iter+1):
            if self.reset_interval and i % self.reset_interval == 0:
                self.initialize_swarm()
            for p in self.swarm:
                self.update_particle(p)
            self.inertia *= self.wdamp
            if callback:
                callback(i, self.global_best_details)
        return self.global_best_pos, self.global_best_cost, self.global_best_details

# ==================== Obstacles ====================
class CircularObstacle:
    def __init__(self, center, radius):
        self.center = np.array(center)
        self.radius = radius
    def in_collision(self, point, robot_radius=0):
        return np.linalg.norm(point - self.center) <= self.radius + robot_radius

class SquareObstacle:
    def __init__(self, center, side):
        self.center = np.array(center)
        self.half = side/2
    def in_collision(self, point, robot_radius=0):
        x,y = point
        return (self.center[0]-self.half-robot_radius <= x <= self.center[0]+self.half+robot_radius and
                self.center[1]-self.half-robot_radius <= y <= self.center[1]+self.half+robot_radius)

# ==================== Environment ====================
class Environment:
    def __init__(self, width, height, robot_radius, start, goal):
        self.width = width
        self.height = height
        self.robot_radius = robot_radius
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.obstacles = []
    def add_obstacle(self, o): self.obstacles.append(o)
    def in_collision(self, pt): return any(o.in_collision(pt, self.robot_radius) for o in self.obstacles)
    def clip_path(self, path): return np.clip(path, [self.robot_radius,self.robot_radius], [self.width-self.robot_radius,self.height-self.robot_radius])
    def path_length(self, path): return np.sum(np.linalg.norm(np.diff(path,axis=0),axis=1))
    def count_violations(self, path):
        start_v = np.linalg.norm(path[0]-self.start)>self.robot_radius
        goal_v = np.linalg.norm(path[-1]-self.goal)>self.robot_radius
        env_v = sum((pt[0]<self.robot_radius or pt[0]>self.width-self.robot_radius or pt[1]<self.robot_radius or pt[1]>self.height-self.robot_radius) for pt in path)
        coll_v = sum(self.in_collision(pt) for pt in path)
        return int(start_v)+int(goal_v)+env_v+coll_v, {'start_violation':start_v,'goal_violation':goal_v,'environment_violations':env_v,'collision_violations':coll_v}

# ==================== Spline Path ====================
class SplinePath:
    def __init__(self, env, cps, resolution):
        self.env = env
        self.cps = np.array(cps)
        self.resolution = resolution
    @classmethod
    def from_vector(cls, env, vec, resolution):
        pts = np.array(vec).reshape(-1,2)
        pts[:,0] *= env.width
        pts[:,1] *= env.height
        return cls(env, pts, resolution)
    def get_path(self):
        pts = np.vstack((self.env.start, self.cps, self.env.goal))
        t = np.linspace(0,1,len(pts))
        cs = CubicSpline(t, pts, bc_type='clamped')
        dense = np.linspace(0,1,self.resolution)
        path = cs(dense)
        return self.env.clip_path(path)

# ==================== Robot Simulation ====================
class Robot:
    def __init__(self, path, color, scale):
        self.path = path
        self.color = color
        self.scale = scale
        self.index = 0
    def update(self):
        if self.index < len(self.path)-1:
            self.index += 1
    def draw(self, surface):
        pos = self.path[self.index] * self.scale
        pygame.draw.circle(surface, self.color, pos.astype(int), int(5))
        if self.index < len(self.path)-1:
            nxt = self.path[self.index+1] * self.scale
            dir_vec = nxt - pos
            if np.linalg.norm(dir_vec)>0:
                dir_u = dir_vec/np.linalg.norm(dir_vec)
                end = pos + dir_u*15
                pygame.draw.line(surface, (0,0,0), pos.astype(int), end.astype(int), 2)

# ==================== Main ====================
def main():
    pygame.init()
    scale = 6
    width, height = 100, 100
    env = Environment(width, height, robot_radius=1, start=[5,5], goal=[95,95])
    # obstacles
    # for c in [([30,30],10), ([70,40],15)]:
    #     env.add_obstacle(CircularObstacle(*c))
    for s in [([30,30],35), ([70,30],35),([30,70],35),([70,70],35), ([50,50],15)]:
        env.add_obstacle(SquareObstacle(*s))

    resolution = 80
    num_cp = 3

    # Define cost function here
    def path_cost_from_vec(vec):
        sol = SplinePath.from_vector(env, vec, resolution)
        path = sol.get_path()
        length = env.path_length(path)
        _, vio = env.count_violations(path)
        cost = length * (1 + vio['collision_violations']*10 + vio['environment_violations'])
        return cost, {'solution': sol, 'path': path}

    solver = PSO(num_vars=2*num_cp, var_min=0, var_max=1,
                 cost_function=path_cost_from_vec, pop_size=50,
                 c1=2.0, c2=2.0, inertia=0.9, wdamp=0.99, reset_interval=20)

    # plan paths for 3 robots
    starts = [[5,5], [50,10], [10,50]]
    robots = []
    colors = [(255,0,0),(0,255,0),(0,0,255)]
    for st, col in zip(starts, colors):
        env.start = np.array(st)
        solver.global_best_cost = float('inf')
        solver.global_best_pos = None
        solver.global_best_details = None
        _, _, detail = solver.run(max_iter=100)
        path = detail['path']
        robots.append(Robot(path, col, scale))

    screen = pygame.display.set_mode((width*scale, height*scale))
    clock = pygame.time.Clock()
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        screen.fill((255,255,255))
        # draw obstacles
        for o in env.obstacles:
            if isinstance(o, CircularObstacle):
                pygame.draw.circle(screen, (128,128,128), (o.center*scale).astype(int), int(o.radius*scale))
            else:
                rect = pygame.Rect((o.center - o.half)*scale, (2*o.half*scale,2*o.half*scale))
                pygame.draw.rect(screen, (139,69,19), rect)
        # draw goal
        gpos = env.goal * scale
        pygame.draw.circle(screen, (0,0,0), gpos.astype(int), 8)
        # draw each robot's global path
        for r in robots:
            if len(r.path) > 1:
                pts = (r.path * scale).astype(int).tolist()
                pygame.draw.lines(screen, r.color, False, pts, 2)
        # update and draw robots
        for r in robots:
            r.update()
            r.draw(screen)
        pygame.display.flip()
        clock.tick(30)
    pygame.quit()
    sys.exit()

if __name__=='__main__':
    main()