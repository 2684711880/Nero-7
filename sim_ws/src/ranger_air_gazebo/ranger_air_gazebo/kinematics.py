"""Model-coordinate kinematics. Order is FL, FR, RL, RR throughout."""
import math
import numpy as np

WHEELS = ('fl', 'fr', 'rl', 'rr')
XY = ((.194, .169), (.194, -.169), (-.194, .169), (-.194, -.169))
STEER_SIGN = (1., 1., 1., -1.)
DRIVE_SIGN = (-1., -1., 1., -1.)
RADIUS = .0605


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def inverse(vx, vy, wz, current, radius=RADIUS):
    """Return joint angles (continuous near current) and signed joint rad/s."""
    angles, rates = [], []
    for (x, y), ss, ds, q in zip(XY, STEER_SIGN, DRIVE_SIGN, current):
        ux, uy = vx - wz * y, vy + wz * x
        speed = math.hypot(ux, uy)
        if speed < 1e-6:
            angles.append(q)
            rates.append(0.)
            continue
        heading = q * ss
        delta = wrap(math.atan2(uy, ux) - heading)
        if abs(delta) > math.pi / 2:
            delta = wrap(delta + math.pi)
            speed = -speed
        angles.append(q + ss * delta)
        rates.append(ds * speed / radius)
    return angles, rates


def forward(angles, rates, radius=RADIUS):
    """Least-squares body twist from measured steering and wheel rates, no slip."""
    a, b = [], []
    for (x, y), ss, ds, q, rate in zip(XY, STEER_SIGN, DRIVE_SIGN, angles, rates):
        theta, speed = ss * q, ds * rate * radius
        a.extend(((1., 0., -y), (0., 1., x)))
        b.extend((speed * math.cos(theta), speed * math.sin(theta)))
    return np.linalg.lstsq(a, b, rcond=None)[0]
