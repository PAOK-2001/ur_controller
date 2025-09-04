import yaml
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Point
from shapely.ops import unary_union
from shapely.geometry import Polygon, box
from scipy.spatial.distance import cdist

def make_workspace_cut_circle(
    radius: float,
    center: tuple[float, float] = (0.0, 0.0),
    cutoff: float = 0.0,
    direction: str = 'above',
    resolution: int = 128
) -> Polygon:
    """
    Create a circular workspace (2D) and “cut” it at an arbitrary horizontal line.

    Args:
      radius     : circle radius
      center     : (cx, cy) center of the circle
      cutoff     : the y-value at which to cut the circle
      direction  : 'above' → keep y ≥ cutoff,
                   'below' → keep y ≤ cutoff
      resolution : number of segments used to approximate the circle

    Returns:
      A Shapely Polygon representing the portion of the circle
      on the specified side of the horizontal line y = cutoff.
    """
    cx, cy = center
    # full circle
    circle = Point(cx, cy).buffer(radius, resolution=resolution)
    # build cutter box
    minx, miny, maxx, maxy = circle.bounds
    if direction == 'positive':
        cutter = box(minx, cutoff, maxx, maxy)
    else:  # 'negative'
        cutter = box(minx, miny, maxx, cutoff)
    # intersect to get the cut circle
    return circle.intersection(cutter)

def load_geometry_from_yaml(yaml_path: str):
    """
    Load workspace & self‐collision barrier from a YAML file, and compute the safe‐zone.

    The YAML must look like:

      self_collision_barrier:
        x_min: <float>
        x_max: <float>
        y_min: <float>
        y_max: <float>

      workspace_min:
        x: <float>
        y: <float>
        z: <float>

      workspace_max:
        x: <float>
        y: <float>
        z: <float>

    Returns:
      workspace_poly   : shapely.geometry.Polygon  # 2D footprint of workspace
      barrier_poly     : shapely.geometry.Polygon  # 2D footprint of self‐collision barrier
      safe_zone_poly   : shapely.geometry.Polygon  # workspace_poly minus barrier_poly
      z_min, z_max     : floats                    # Z‐limits of the workspace
    """
    # 1) load YAML
    with open(yaml_path, 'r') as f:
        cfg = yaml.safe_load(f)

    # 2) parse barrier and ensure min/max ordering
    sb = cfg['self_collision_barrier']
    bx0, bx1 = sb['x_min'], sb['x_max']
    by0, by1 = sb['y_min'], sb['y_max']
    bxmin, bxmax = sorted((bx0, bx1))
    bymin, bymax = sorted((by0, by1))
    barrier_poly = Polygon([
        (bxmin, bymin),
        (bxmin, bymax),
        (bxmax, bymax),
        (bxmax, bymin),
    ])

    # 3) parse workspace (XY footprint) + Z–limits
    wmin = cfg['workspace_min']
    wmax = cfg['workspace_max']
    
    zmin = wmin['z']
    zmax = wmax['z']
    
    workspace_poly = make_workspace_cut_circle(
        radius=cfg['workspace_circle']['radius'],
        center=cfg['workspace_circle']['center'],
        cutoff=cfg['workspace_circle']['cutoff'],
        direction=cfg['workspace_circle']['direction']
    )
    

    # 4) compute safe zone = workspace minus barrier
    #    (unary_union in case you ever pass a list of barriers)
    barrier_union = unary_union(barrier_poly)
    safe_zone_poly = workspace_poly.difference(barrier_union)

    return workspace_poly, barrier_poly, safe_zone_poly, zmin, zmax

def plot_geometry(workspace: Polygon, barrier: Polygon, safe_zone: Polygon,
                  ax=None, figsize=(8,5)):
    """
    Plots workspace (green), barrier (red), and safe_zone (light green).
    """
    if ax is None:
        fig, ax = plt.subplots(figsize=figsize)
    def _draw(poly, **kwargs):
        if poly.is_empty:
            return
        if poly.geom_type == 'Polygon':
            xs, ys = poly.exterior.xy
            ax.fill(xs, ys, **kwargs)
        else:  # MultiPolygon
            for p in poly:
                _draw(p, **kwargs)

    _draw(workspace, color='green', alpha=0.3, ec='darkgreen', label='Workspace')
    _draw(safe_zone, color='blue', alpha=0.8, ec='green', label='Safe Zone')
    _draw(barrier,   color='red',   alpha=0.5, ec='darkred',   label='Barrier')
    ax.set_aspect('equal', 'box')
    ax.legend()
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Workspace with Self-Collision Barrier & Safe-Zone')
    plt.savefig('workspace_geometry.png', dpi=300)
    
def clamp_point_to_safe_zone(
        pt: tuple[float, float, float],
        safe_poly,
        z_min: float,
        z_max: float
    ) -> tuple[float, float, float]:
        """
        Clamp a 3D point (x, y, z) into the safe zone.

        - XY is projected into the 2D safe_poly (Shapely Polygon).
        - Z is clamped between [z_min, z_max].

        Args:
        pt        : (x, y, z)
        safe_poly : shapely Polygon defining XY-safe region
        z_min     : float, minimum Z
        z_max     : float, maximum Z

        Returns:
        (x_clamped, y_clamped, z_clamped)
        """
        x, y, z = pt
        p2d = Point(x, y)

        if safe_poly.contains(p2d):
            cx, cy = x, y
        else:
            boundary = safe_poly.exterior
            nearest = boundary.interpolate(boundary.project(p2d))
            cx, cy = nearest.x, nearest.y

        cz = min(max(z, z_min), z_max)
        return (cx, cy, cz)
    
import numpy as np

import matplotlib.pyplot as plt

def order_points_by_closeness(points: np.ndarray) -> np.ndarray:
    n = len(points)
    ordered = [0]
    unvisited = set(range(1, n))

    while unvisited:
        last = ordered[-1]
        dists = cdist([points[last]], points[list(unvisited)])
        next_idx = list(unvisited)[np.argmin(dists)]
        ordered.append(next_idx)
        unvisited.remove(next_idx)

    return points[ordered]

def plot_ordered_points(points: np.ndarray, ax: plt.Axes):
    ordered_points = order_points_by_closeness(points)
    ax.plot(ordered_points[:, 0], ordered_points[:, 1], 'o-', label='Ordered Path')
    for i, p in enumerate(ordered_points):
        ax.text(p[0], p[1], str(i), fontsize=8, ha='right')
    ax.set_title("Ordered Points by Closeness")
    ax.axis("equal")
    ax.grid(True)
    ax.legend()

    # Example usage:
if __name__ == '__main__':
    workspace, barrier, safe_zone, zmin, zmax = load_geometry_from_yaml('/home/paok/Documents/MOTORCORTEX/motor_cortex/benchmarks/real_life/ur_control/src/ur_control/config/arm_parms.yaml')
    print(f"Z limits: {zmin} to {zmax}")
    plot_geometry(workspace, barrier, safe_zone)
    
    pt = (0.04, -0.16, 0.06)  # Example point
    clamped_pt = clamp_point_to_safe_zone(pt, safe_zone, zmin, zmax)
    print(f"Clamped point: {clamped_pt}")

    # plot the points, the safe zone, AND BARRIER
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.set_aspect('equal', 'box')

    # Plot safe zone
    if not safe_zone.is_empty:
        if safe_zone.geom_type == 'Polygon':
            xs, ys = safe_zone.exterior.xy
            ax.fill(xs, ys, color='blue', alpha=0.2, label='Safe Zone')
        else:  # MultiPolygon
            for poly in safe_zone:
                xs, ys = poly.exterior.xy
                ax.fill(xs, ys, color='blue', alpha=0.2, label='Safe Zone')

    # Plot barrier
    if not barrier.is_empty:
        if barrier.geom_type == 'Polygon':
            xs, ys = barrier.exterior.xy
            ax.fill(xs, ys, color='red', alpha=0.5, label='Barrier')
        else:  # MultiPolygon
            for poly in barrier:
                xs, ys = poly.exterior.xy
                ax.fill(xs, ys, color='red', alpha=0.5, label='Barrier')

    # Plot points
    ax.plot(pt[0], pt[1], 'ro', label='Original Point')
    ax.plot(clamped_pt[0], clamped_pt[1], 'bo', label='Clamped Point')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Point Clamping to Safe Zone and Barrier')
    ax.legend()
    plt.savefig('clamped_point.png', dpi=300)