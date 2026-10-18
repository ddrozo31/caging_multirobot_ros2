#!/usr/bin/env python3
"""
Generate the meshes of the paper's objects (Fig. 10) that Gazebo has no shape for.

  meshes/hemisphere.stl : solid hemisphere, radius 0.10 m, flat face on the floor
  meshes/pyramid.stl    : three-sided pyramid, equilateral base edge 0.15 m, height 0.15 m
                          (the paper's pyramid has rounded edges; ours has sharp edges)

The box (0.2 m cube) is a Gazebo primitive and needs no mesh.

Every mesh has its origin at the center of its base, on the floor (z = 0), so an
object spawned at z = 0 rests on the ground.

An STL file is just a list of triangles. Each triangle = 3 corners (x, y, z),
listed counter-clockwise when seen from OUTSIDE the object (so its normal points out).

Run (from src/caging_description):
  python3 scripts/make_object_meshes.py
"""
import math
import os


def write_stl(path, name, triangles):
    """Write a list of triangles [(p1, p2, p3), ...] as an ASCII STL file."""
    with open(path, 'w') as f:
        f.write(f'solid {name}\n')
        for p1, p2, p3 in triangles:
            # normal = (p2 - p1) x (p3 - p1), normalized
            ux, uy, uz = (p2[i] - p1[i] for i in range(3))
            vx, vy, vz = (p3[i] - p1[i] for i in range(3))
            nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
            length = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
            f.write(f'  facet normal {nx / length:.6f} {ny / length:.6f} {nz / length:.6f}\n')
            f.write('    outer loop\n')
            for p in (p1, p2, p3):
                f.write(f'      vertex {p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n')
            f.write('    endloop\n  endfacet\n')
        f.write(f'endsolid {name}\n')


def hemisphere(radius=0.10, n_around=32, n_up=8):
    """Dome made of n_up rings x n_around slices, plus a flat bottom disk."""
    def point(ring, k):
        # ring 0 = equator (z = 0), ring n_up = top pole
        elevation = (math.pi / 2) * ring / n_up
        azimuth = 2 * math.pi * k / n_around
        r = radius * math.cos(elevation)
        return (r * math.cos(azimuth), r * math.sin(azimuth), radius * math.sin(elevation))

    triangles = []
    for ring in range(n_up):
        for k in range(n_around):
            a, b = point(ring, k), point(ring, k + 1)              # lower edge
            c, d = point(ring + 1, k + 1), point(ring + 1, k)      # upper edge
            triangles.append((a, b, c))
            if ring < n_up - 1:   # the top ring closes on one point: one triangle is enough
                triangles.append((a, c, d))
    center = (0.0, 0.0, 0.0)
    for k in range(n_around):   # bottom disk, seen from below: clockwise from above
        triangles.append((center, point(0, k + 1), point(0, k)))
    return triangles


def pyramid(edge=0.15, height=0.15):
    """Tetrahedron: equilateral triangle base centered at the origin, apex above it."""
    r = edge / math.sqrt(3)   # distance from the center of the base to each corner
    a, b, c = [(r * math.cos(t), r * math.sin(t), 0.0)
               for t in (0.0, 2 * math.pi / 3, 4 * math.pi / 3)]
    apex = (0.0, 0.0, height)
    return [(a, c, b),                                    # base (normal points down)
            (a, b, apex), (b, c, apex), (c, a, apex)]     # three sides


if __name__ == '__main__':
    mesh_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'meshes')
    os.makedirs(mesh_dir, exist_ok=True)
    write_stl(os.path.join(mesh_dir, 'hemisphere.stl'), 'hemisphere', hemisphere())
    write_stl(os.path.join(mesh_dir, 'pyramid.stl'), 'pyramid', pyramid())
    print('Written meshes/hemisphere.stl and meshes/pyramid.stl')
