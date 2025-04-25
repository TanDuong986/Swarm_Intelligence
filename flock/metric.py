#compute order and entropy of the flock
# metric.py
from math import atan2, log, pi
from vector import Vector

def compute_order(boids):
    avg = Vector()
    for b in boids:
        dir = b.velocity.normalize()
        avg += dir
    avg = avg / len(boids)
    return avg.magnitude()

def compute_entropy(boids, nbins=36):
    angles = [atan2(b.velocity.y, b.velocity.x) for b in boids]
    hist = [0] * nbins
    bin_size = 2 * pi / nbins
    for a in angles:
        idx = int((a + pi) / (2*pi) * nbins)
        idx = min(max(idx, 0), nbins-1)
        hist[idx] += 1

    entropy = 0.0
    N = len(angles)
    for count in hist:
        if count:
            p = count / N
            entropy -= p * log(p)
    return entropy
