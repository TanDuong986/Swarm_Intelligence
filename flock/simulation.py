# simulation.py
import random, math
import pygame
from vector import Vector
from boid import Boid
from obstacle import Wall, CircleObstacle
from trajectory import Trajectory
from metric import compute_order, compute_entropy
from visualization import init_screen, render

WIDTH, HEIGHT = 800, 600
NUM_BOIDS    = 50

def main():
    screen = init_screen(WIDTH, HEIGHT)
    clock = pygame.time.Clock()

    # spawn boids quanh tâm trong bán kính R
    center = Vector(WIDTH/2, HEIGHT/2)
    R = 100
    boids = []
    for _ in range(NUM_BOIDS):
        theta = random.random() * 2 * math.pi
        r = random.uniform(0, R)
        pos = Vector(center.x + math.cos(theta)*r,
                     center.y + math.sin(theta)*r)
        vel = Vector(random.uniform(-1,1), random.uniform(-1,1))
        boids.append(Boid(position=pos, velocity=vel))

    # chướng ngại
    obstacles = [
        Wall(Vector(100,100), Vector(700,100)),
        CircleObstacle(center, 80)
    ]

    # quỹ đạo đường tròn
    traj = Trajectory('circle', {
        'center': center,
        'radius': 200,
        'angular_speed': 0.3
    })

    t, dt = 0.0, 0.1
    running = True
    while running:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False

        # điều khiển
        for b in boids:
            b.navigate(boids, obstacles, traj.target_point(t))

        # cập nhật và wrap-around
        for b in boids:
            b.update(dt)
            b.position.x %= WIDTH
            b.position.y %= HEIGHT

        # metrics + render
        order   = compute_order(boids)
        entropy = compute_entropy(boids)
        render(screen, boids, obstacles, order, entropy)

        t += dt
        clock.tick(60)

    pygame.quit()

if __name__ == '__main__':
    main()
