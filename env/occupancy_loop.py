"""Deterministic, clearance-checked perimeter loop over an occupancy image."""
import math

import cv2
import networkx as nx
import numpy as np
from scipy.ndimage import distance_transform_edt, label


def plan_loop(image, scale, origin, clearance=0.2, simplify_tolerance=0.15):
    if scale <= 0 or clearance <= 0:
        raise ValueError('scale and clearance must be positive')
    if simplify_tolerance < 0:
        raise ValueError('simplify_tolerance cannot be negative')
    free = np.asarray(image) == 255
    # Outside the map is unknown, not unbounded free space.
    distance = distance_transform_edt(np.pad(free, 1, constant_values=False))[1:-1, 1:-1] * scale
    start = tuple(int(v) for v in np.unravel_index(np.argmax(distance), distance.shape))
    safe = free & (distance + 1e-9 >= clearance)
    if not safe[start]:
        raise ValueError('No free pixel meets the requested clearance')
    labels, _ = label(safe)
    safe = labels == labels[start]
    # One half-cell inset makes diagonal contour edges safe under conservative
    # supercover checking while retaining the largest feasible obstacle loop.
    inset = safe & (distance + 1e-9 >= clearance + scale / 2)
    contours, hierarchy = cv2.findContours(inset.astype(np.uint8), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    if hierarchy is None:
        raise ValueError('No perimeter loop: inset safe region is empty')
    holes = [i for i in range(len(contours)) if hierarchy[0][i][3] >= 0]
    if not holes:
        raise ValueError('No closed perimeter around an internal obstacle at this clearance')
    hole = max(holes, key=lambda i: (cv2.contourArea(contours[i]), -i))
    loop = [(int(p[0][1]), int(p[0][0])) for p in contours[hole]]
    if len(loop) < 3:
        raise ValueError('Perimeter contour is too short')

    raw_loop = loop[:]

    def edge_ok(a, b):
        r, c = a; rr, cc = b
        return (safe[r, c] and safe[rr, cc] and
                (r == rr or c == cc or (safe[r, cc] and safe[rr, c])))

    def segment_ok(a, b):
        # Sample at quarter-pixel intervals so simplified chords cannot clip a wall.
        steps = max(1, int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) * 4)))
        previous = None
        for i in range(steps + 1):
            t = i / steps
            point = (int(round(a[0] + t * (b[0] - a[0]))),
                     int(round(a[1] + t * (b[1] - a[1]))))
            if not (0 <= point[0] < safe.shape[0] and 0 <= point[1] < safe.shape[1] and safe[point]):
                return False
            if previous is not None and not edge_ok(previous, point):
                return False
            previous = point
        return True

    contour_xy = np.asarray([[[col, row]] for row, col in loop], dtype=np.int32)
    # Reduce tolerance until every shortcut remains inside the clearance mask.
    actual_tolerance = float(simplify_tolerance)
    while True:
        approximation = cv2.approxPolyDP(contour_xy, actual_tolerance / scale, True)
        simplified_loop = [(int(point[0][1]), int(point[0][0])) for point in approximation]
        valid_loop = len(simplified_loop) >= 3 and all(
            segment_ok(a, b) for a, b in zip(simplified_loop, simplified_loop[1:] + simplified_loop[:1])
        )
        if valid_loop:
            break
        actual_tolerance *= 0.5
        if actual_tolerance < scale * 0.05:
            simplified_loop = loop[:]
            actual_tolerance = 0.0
            break
    loop = simplified_loop

    if not all(segment_ok(a, b) for a, b in zip(loop, loop[1:] + loop[:1])):
        raise ValueError('Perimeter contour violates clearance at a diagonal edge')

    graph = nx.Graph()
    for r, c in np.argwhere(safe):
        a = (int(r), int(c))
        graph.add_node(a)
        for dr, dc in ((0, 1), (1, -1), (1, 0), (1, 1)):
            b = (int(r + dr), int(c + dc))
            if 0 <= b[0] < safe.shape[0] and 0 <= b[1] < safe.shape[1] and edge_ok(a, b):
                graph.add_edge(a, b, weight=math.hypot(dr, dc))
    lengths, paths = nx.single_source_dijkstra(graph, start, weight='weight')
    candidates = [(lengths[p], i) for i, p in enumerate(loop) if p in lengths]
    if not candidates:
        raise ValueError('The safest start cannot reach the perimeter loop')
    _, entry_index = min(candidates)
    loop = loop[entry_index:] + loop[:entry_index]
    approach = paths[loop[0]]
    pixels = approach + loop[1:] + [loop[0]] + list(reversed(approach[:-1]))
    if not all(segment_ok(a, b) for a, b in zip(pixels, pixels[1:])):
        raise ValueError('Generated route has an unsafe segment')

    def world(point):
        r, c = point
        return (float(origin[0] + (image.shape[1] - c - 0.5) * scale),
                float(origin[1] + (r + 0.5) * scale))

    # Retain corners and samples no more than 0.25 m apart; do not shortcut
    # around obstacles. Collapse only collinear runs.
    selected = [pixels[0]]
    stride = max(1, int(0.25 / scale))
    last_direction = None
    for i in range(1, len(pixels) - 1):
        before, point, after = pixels[i - 1], pixels[i], pixels[i + 1]
        direction = (point[0] - before[0], point[1] - before[1])
        next_direction = (after[0] - point[0], after[1] - point[1])
        if direction != next_direction or max(abs(point[0] - selected[-1][0]), abs(point[1] - selected[-1][1])) >= stride:
            if point != selected[-1]: selected.append(point)
        last_direction = direction
    if pixels[-1] != selected[-1]: selected.append(pixels[-1])
    points = [world(p) for p in selected]
    poses = []
    for i, (x, y) in enumerate(points):
        j = min(i + 1, len(points) - 1)
        if j == i: j = max(0, i - 1)
        dx, dy = points[j][0] - x, points[j][1] - y
        if i == len(points) - 1: dx, dy = -dx, -dy
        poses.append({'x': x, 'y': y, 'yaw': float(math.atan2(dy, dx))})
    def length(seq):
        return sum(math.hypot(b[0]-a[0], b[1]-a[1]) * scale for a,b in zip(seq, seq[1:]))
    return {'frame_id': 'scene', 'loop': False, 'source': 'occupancy perimeter loop', 'poses': poses,
            'planning': {'clearance_m': float(clearance), 'simplify_tolerance_requested_m': float(simplify_tolerance),
                         'simplify_tolerance_used_m': float(actual_tolerance),
                         'raw_perimeter_vertices': len(raw_loop), 'simplified_perimeter_vertices': len(loop),
                         'start_pixel_rc': list(start),
                         'start_clearance_m': float(distance[start]),
                         'minimum_route_clearance_m': float(min(distance[p] for p in pixels)),
                         'safe_component_pixels': int(safe.sum()), 'perimeter_length_m': length(loop + loop[:1]),
                         'approach_length_m': length(approach), 'route_length_m': length(pixels),
                         'hole_area_m2': float(cv2.contourArea(contours[hole]) * scale * scale),
                         'route_pixel_rc': [list(p) for p in pixels], 'approach_pixel_rc': [list(p) for p in approach],
                         'raw_perimeter_pixel_rc': [list(p) for p in raw_loop],
                         'perimeter_pixel_rc': [list(p) for p in loop]}}
