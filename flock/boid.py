# boid.py
from vector import Vector

class Boid:
    def __init__(self,
                 position: Vector,
                 velocity: Vector,
                 max_speed: float = 100.0,
                 max_force: float = 10.0):
        self.position = position
        self.velocity = velocity
        self.acceleration = Vector()
        self.max_speed = max_speed
        self.max_force = max_force
        # bán kính nhận diện lân cận
        self.sep_radius = 50
        self.align_radius = 100
        self.cohesion_radius = 100

    def apply_force(self, force: Vector):
        self.acceleration += force

    def separation(self, neighbors) -> Vector:
        steer = Vector()
        total = 0
        for other in neighbors:
            diff = self.position - other.position
            d = diff.magnitude()
            if d > 0 and d < self.sep_radius:
                steer += diff.normalize() / d
                total += 1
        if total > 0:
            steer = (steer / total).normalize() * self.max_force
        return steer

    def alignment(self, neighbors) -> Vector:
        avg = Vector()
        total = 0
        for other in neighbors:
            diff = other.position - self.position
            if diff.magnitude() < self.align_radius:
                avg += other.velocity
                total += 1
        if total > 0:
            avg = (avg / total).normalize() * self.max_speed
            return (avg - self.velocity).limit(self.max_force)
        return Vector()

    def cohesion(self, neighbors) -> Vector:
        center = Vector()
        total = 0
        for other in neighbors:
            diff = other.position - self.position
            if diff.magnitude() < self.cohesion_radius:
                center += other.position
                total += 1
        if total > 0:
            center = center / total
            desired = (center - self.position).normalize() * self.max_speed
            return (desired - self.velocity).limit(self.max_force)
        return Vector()

    def avoid_obstacles(self, obstacles) -> Vector:
        steer = Vector()
        for obs in obstacles:
            steer += obs.steer_to_avoid(self.position, self.velocity)
        return steer

    def migrate(self, target: Vector) -> Vector:
        desired = (target - self.position).normalize() * self.max_speed
        return (desired - self.velocity).limit(self.max_force)

    def navigate(self, boids, obstacles, target: Vector):
        # lọc neighbor cho từng rule
        neigh_sep = [b for b in boids
                     if b is not self and (self.position - b.position).magnitude() < self.sep_radius]
        neigh_ali = [b for b in boids
                     if b is not self and (self.position - b.position).magnitude() < self.align_radius]
        neigh_coh = [b for b in boids
                     if b is not self and (self.position - b.position).magnitude() < self.cohesion_radius]

        # tính từng lực và nhân hệ số weight
        f_sep = self.separation(neigh_sep) * 1.5
        f_obs = self.avoid_obstacles(obstacles) * 2.0
        f_ali = self.alignment(neigh_ali) * 1.0
        f_coh = self.cohesion(neigh_coh) * 1.0
        f_mig = self.migrate(target) * 0.5

        # tổng hợp và giới hạn tổng lực
        total = (f_sep + f_obs + f_ali + f_coh + f_mig).limit(self.max_force)
        self.apply_force(total)

    def update(self, dt: float):
        self.velocity = (self.velocity + self.acceleration * dt).limit(self.max_speed)
        self.position = self.position + self.velocity * dt
        self.acceleration = Vector()
