# vector.py
import math

class Vector:
    def __init__(self, x: float = 0.0, y: float = 0.0):
        self.x = x
        self.y = y

    def __add__(self, other):
        return Vector(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vector(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar: float):
        return Vector(self.x * scalar, self.y * scalar)
    __rmul__ = __mul__

    def __truediv__(self, scalar: float):
        return Vector(self.x / scalar, self.y / scalar)

    def magnitude(self) -> float:
        return math.hypot(self.x, self.y)

    def normalize(self):
        mag = self.magnitude()
        if mag > 0:
            return self / mag
        return Vector()

    def limit(self, max_val: float):
        mag = self.magnitude()
        if mag > max_val and mag > 0:
            return self.normalize() * max_val
        return self

    def tuple(self):
        return (self.x, self.y)
