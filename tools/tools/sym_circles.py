from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import numpy as np
import scipy.linalg


def rot3d(theta, axis):
    wx = np.cross(np.identity(3), axis / np.linalg.norm(axis) * theta)
    return scipy.linalg.expm3(wx)


def rot_to(v1, v2):
    unit_axis = unitize(np.cross(v1, v2))
    theta = np.arccos(np.dot(v1, v2))
    return rot3d(theta, unit_axis)


def unitize(v, bias=np.array([0.0, 0.0, 1.0])):
    norm = np.linalg.norm(v)
    if norm < 1e-3:
        return bias
    else:
        return v / norm


def fig3d():
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim([-2.0, 2.0])
    ax.set_ylim([-2.0, 2.0])
    ax.set_zlim([-2.0, 2.0])
    return ax


def draw_circle(c, r, n):
    t = np.linspace(0.0, 2 * np.pi, 1000)
    _x = np.cos(t)
    _y = np.sin(t)
    _z = np.zeros(t.shape)
    _xyz = np.vstack([_x, _y, _z])

    z_hat = np.array([0.0, 0.0, 1.0])
    R = rot_to(z_hat, n)
    xyz = R.dot(_xyz) + c[:, None]
    plt.plot(*xyz)


def marker(ax, pt):
    plt.scatter(pt[0], pt[1], zs=pt[2], s=[25], c='r')


def line(ax, a, b):
    ax.plot(
        [a[0], b[0]],
        [a[1], b[1]],
        [a[2], b[2]],
    )


def intersect_planes(n1, c1, n2, c2):
    a = unitize(np.cross(n1, n2))
    m = np.vstack([n1, n2])
    print

    b = np.array([n1.dot(c1), n2.dot(c2)])

    pt, _, _, _ = np.linalg.lstsq(m, b)

    double_a = 2 * a
    line(plt.gca(), pt - double_a, pt + double_a)
    return a, pt


def intersect_circles(circ_1, circ_2, eps=0.05):
    def validate_intersections(candidates, radius, center):
        distances = map(lambda a: np.linalg.norm(a - center), candidates)
        # Filter out intersections that don't lie on the second circle
        return list([candidate for n, candidate in enumerate(candidates) if np.fabs(distances[n] - radius) < eps])

    n1, r1, o1 = circ_1
    n2, r2, o2 = circ_2

    # Get the line that describes the intersection of the two planes
    a, x0 = intersect_planes(n1, o1, n2, o2)
    q = x0 - o1

    # The two points at which one circle intersects that line can be found via the quadratic formula
    discriminant = (a.dot(q) ** 2) - (np.linalg.norm(q) ** 2) + (r1 ** 2)

    # Negative discriminant
    if discriminant < 1e-12:
        # No intersections
        return []

    # Non-zero discriminant
    elif discriminant > 1e-1:
        # Two intersections
        t1 = -a.dot(q) + np.sqrt(discriminant)
        t2 = -a.dot(q) - np.sqrt(discriminant)
        candidate_intersections = [x0 + (a * t1), x0 + (a * t2)]

    # Near-zero discriminant
    else:
        # One intersection
        t = -a.dot(q)
        candidate_intersections = [x0 + (a * t)]

    intersections = validate_intersections(candidate_intersections, r2, o2)
    # intersections = candidate_intersections
    for _int in intersections:
        marker(plt.gca(), _int)

if __name__ == '__main__':
    c1 = np.array([0.0, 0.0, 0.0])
    r1 = 1.0
    n1 = unitize(np.array([0.0, 0.0, 1.0]))

    c2 = np.array([0.0, 0.2, 0.0])
    r2 = 1.0
    n2 = unitize(np.array([0.0, 1.0, 1.0]))

    ax = fig3d()
    draw_circle(c1, r1, n1)
    draw_circle(c2, r2, n2)

    line(ax, c1, c1 + n1)
    line(ax, c2, c2 + n2)
    # print intersect_planes(n1, c1, n2, c2)
    intersect_circles((n1, r1, c1), (n2, r2, c2))

    plt.show()