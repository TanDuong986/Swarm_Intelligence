#difine the obstacle type like wall, circle, polygon, etc.
# obstacle.py
from vector import Vector

class Obstacle:
    def steer_to_avoid(self, position: Vector, velocity: Vector) -> Vector:
        return Vector()

class Wall(Obstacle):
    def __init__(self, start: Vector, end: Vector):
        self.start = start
        self.end = end

    def steer_to_avoid(self, position, velocity) -> Vector:
        # tính điểm gần nhất trên đoạn thẳng
        seg_v = self.end - self.start
        pt_v = position - self.start
        seg_len = seg_v.magnitude()
        seg_dir = seg_v.normalize()
        proj = pt_v.x*seg_dir.x + pt_v.y*seg_dir.y
        proj = max(0, min(seg_len, proj))
        closest = self.start + seg_dir * proj

        diff = position - closest
        dist = diff.magnitude()
        threshold = 30
        if dist < threshold and dist > 0:
            return diff.normalize() * self.max_avoid_force(dist, threshold)
        return Vector()

    def max_avoid_force(self, dist, threshold):
        # né mạnh hơn khi càng gần
        return (threshold - dist) / threshold * 1.0

class CircleObstacle(Obstacle):
    def __init__(self, center: Vector, radius: float):
        self.center = center
        self.radius = radius

    def steer_to_avoid(self, position, velocity) -> Vector:
        to_center = self.center - position
        d = to_center.magnitude()
        buffer = 10
        if d < self.radius + buffer:
            # né ngược về tâm
            return (position - self.center).normalize()
        return Vector()
