import sympy
import numpy as np

def project_point_to_plane(pt, normal, origin):
    v = pt - origin
    dist = np.dot(v, normal)
    projected = pt - (dist * normal)
    return projected

def intersect(c1, c2):
    n1, r1, o1 = c1
    n2, r2, o2 = c2
    distance = np.linalg.norm(o1 - o2)

    if distance > max(r1, r2):
        # No intersection
        return []

    nx = np.cross(n1, n2)
    cross_norm = np.linalg.norm(nx)
    if cross_norm < 1e-3 and distance > 1e-3:
        # Parallel, non-coplanar
        return []

    return None


def main():
    n1 = np.array([0.0, 0.0, 1.0])
    r1 = 1.0
    o1 = np.array([0.0, 0.0, 0.0])
    c1 = (n1, r1, o1)

    n2 = np.array([0.0, 0.0, 1.0])
    r2 = 1.0
    o2 = np.array([0.0, 1.0, 0.0])
    c2 = (n2, r2, o2)

    print project_point_to_plane(o2, n2, o2)
    print intersect(c1, c2)

if __name__ == '__main__':
    main()
