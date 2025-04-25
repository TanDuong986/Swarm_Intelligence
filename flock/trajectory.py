#define trajectory (round, line, rectangle, circle)
# trajectory.py
from vector import Vector
import math

class Trajectory:
    def __init__(self, type: str, params: dict):
        self.type = type
        self.params = params

    def target_point(self, t: float) -> Vector:
        if self.type == 'line':
            start = self.params['start']
            end = self.params['end']
            duration = self.params.get('duration', 10.0)
            u = min(t / duration, 1.0)
            return Vector(
                start.x + (end.x - start.x) * u,
                start.y + (end.y - start.y) * u
            )
        elif self.type == 'circle':
            center = self.params['center']
            radius = self.params['radius']
            speed = self.params.get('angular_speed', 0.5)
            angle = speed * t
            return Vector(
                center.x + math.cos(angle) * radius,
                center.y + math.sin(angle) * radius
            )
        # có thể thêm 'square', 'ellipse', v.v.
        else:
            return Vector()
