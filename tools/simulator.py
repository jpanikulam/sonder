from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import cv2

import scipy.linalg

from points import points as sonar_points


def rot3d(theta, axis):
    wx = np.cross(np.identity(3), axis / np.linalg.norm(axis) * theta)
    return scipy.linalg.expm3(wx)

def rotate_pts(pts, R, t):
    return R.dot(pts.transpose()).transpose() + t


def fig3d():
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.axis('equal')

    return ax


def render_pts(pts, shape=(256, 256), max_elevation=0.17):

    # Exclude points outside of the imaging range
    rcos_theta = np.linalg.norm(pts[:, :2], axis=1)
    elevations = np.arctan2(pts[:, 2], rcos_theta)
    in_range = np.fabs(elevations) < max_elevation
    pts = pts[in_range]

    ranges = np.linalg.norm(pts, axis=1)
    bearings = np.arctan2(pts[:, 1], pts[:, 0])

    plt.figure('Image')
    plt.scatter(bearings, ranges)
    return np.vstack([ranges, bearings]).transpose()


def draw_arc(ax, distance, bearing, elevation_range, R, t):
    elevations = np.linspace(*elevation_range)
    bearings = np.ones(elevations.shape) * bearing
    ranges = np.ones(elevations.shape) * distance

    rcos_theta = ranges * np.cos(elevations)
    _x = rcos_theta * np.cos(bearings)
    _y = rcos_theta * np.sin(bearings)
    _z = ranges * np.sin(elevations)

    xyz = np.vstack([_x, _y, _z])
    # x, y, z = R.dot(xyz) + t
    x, y, z, = rotate_pts(xyz.transpose(), R, t).transpose()

    ax.plot(x, y, z)


def draw_arcs(ax, range_bearings, R, t, elevation_range=(-0.17, 0.17)):
    for sph_pt in range_bearings:
        draw_arc(ax, sph_pt[0], sph_pt[1], elevation_range, R, t)


def go():
    test_pts = np.array([
        [1.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
        [1.0, -1.0, 0.0],
    ])
    # test_pts = np.random.random((9, 3))
    # test_pts[:, 0] += 1.0
    # test_pts[:, 1] -= 0.5
    points = np.mgrid[1:1.1:9j, -0.5:0.2:2j, 0:1]
    test_pts = points.reshape(3, -1).T

    ax3d = fig3d()
    ax3d.scatter(test_pts[:, 0], test_pts[:, 1], test_pts[:, 2])

    axis = np.array([1.0, 0.0, 0.0])

    for r, t in [(0.0, (0.0, 0.1, 0.0)), (0.25, (0.0, 0.0, 0.0))]:
        rot = rot3d(r, axis)

        rtheta = render_pts(rotate_pts(test_pts, rot, t))
        draw_arcs(ax3d, rtheta, rot.transpose(), -rot.transpose().dot(t))

        ax3d.scatter(*t, color='k')


    plt.show()

if __name__ == '__main__':
    go()