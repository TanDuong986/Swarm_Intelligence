#visualize use pygame
# visualization.py
import pygame
import math
from vector import Vector
from obstacle import Wall, CircleObstacle

def init_screen(width, height):
    pygame.init()
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("Flocking Simulation")
    return screen

def draw_boid(screen, boid: 'Boid'):
    pos = boid.position
    vel = boid.velocity
    angle = math.atan2(vel.y, vel.x)
    size = 6
    # tam giác: mũi + 2 chân
    p1 = Vector(math.cos(angle), math.sin(angle)) * size + pos
    p2 = Vector(math.cos(angle + 2.5), math.sin(angle + 2.5)) * size + pos
    p3 = Vector(math.cos(angle - 2.5), math.sin(angle - 2.5)) * size + pos
    pygame.draw.polygon(screen, (100, 200, 100),
                        [p1.tuple(), p2.tuple(), p3.tuple()])

def draw_obstacle(screen, obs):
    if isinstance(obs, Wall):
        pygame.draw.line(screen, (200,200,200),
                         obs.start.tuple(), obs.end.tuple(), 2)
    elif isinstance(obs, CircleObstacle):
        pygame.draw.circle(screen, (200,200,200),
                           (int(obs.center.x),int(obs.center.y)),
                           int(obs.radius), 2)

def render(screen, boids, obstacles, order, entropy):
    screen.fill((30,30,30))
    # vẽ chướng ngại
    for obs in obstacles:
        draw_obstacle(screen, obs)
    # vẽ boids
    for b in boids:
        draw_boid(screen, b)
    # hiển thị metrics
    font = pygame.font.SysFont(None, 24)
    screen.blit(font.render(f"Order: {order:.2f}", True, (255,255,255)), (10,10))
    screen.blit(font.render(f"Entropy: {entropy:.2f}", True, (255,255,255)), (10,30))
    pygame.display.flip()
