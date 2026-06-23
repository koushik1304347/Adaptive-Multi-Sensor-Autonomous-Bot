import math
import heapq

# ==========================================
# PART 2 & 3: WAYPOINT AND EDGE GRAPH DATA (5x5 Grid)
# ==========================================
WAYPOINTS = {
    1: {'x': -200.0, 'y': -200.0, 'z': 0.0, 'segment_id': 'c1'},
    2: {'x': -100.0, 'y': -200.0, 'z': 0.0, 'segment_id': 'c2'},
    3: {'x':    0.0, 'y': -200.0, 'z': 0.0, 'segment_id': 'c3'},
    4: {'x':  100.0, 'y': -200.0, 'z': 0.0, 'segment_id': 'c4'},
    5: {'x':  200.0, 'y': -200.0, 'z': 0.0, 'segment_id': 'c5'},
    6: {'x': -200.0, 'y': -100.0, 'z': 0.0, 'segment_id': 'c6'},
    7: {'x': -100.0, 'y': -100.0, 'z': 0.0, 'segment_id': 'c7'},
    8: {'x':    0.0, 'y': -100.0, 'z': 0.0, 'segment_id': 'c8'},
    9: {'x':  100.0, 'y': -100.0, 'z': 0.0, 'segment_id': 'c9'},
   10: {'x':  200.0, 'y': -100.0, 'z': 0.0, 'segment_id': 'c10'},
   11: {'x': -200.0, 'y':    0.0, 'z': 0.0, 'segment_id': 'c11'},
   12: {'x': -100.0, 'y':    0.0, 'z': 0.0, 'segment_id': 'c12'},
   13: {'x':    0.0, 'y':    0.0, 'z': 0.0, 'segment_id': 'c13'},
   14: {'x':  100.0, 'y':    0.0, 'z': 0.0, 'segment_id': 'c14'},
   15: {'x':  200.0, 'y':    0.0, 'z': 0.0, 'segment_id': 'c15'},
   16: {'x': -200.0, 'y':  100.0, 'z': 0.0, 'segment_id': 'c16'},
   17: {'x': -100.0, 'y':  100.0, 'z': 0.0, 'segment_id': 'c17'},
   18: {'x':    0.0, 'y':  100.0, 'z': 0.0, 'segment_id': 'c18'},
   19: {'x':  100.0, 'y':  100.0, 'z': 0.0, 'segment_id': 'c19'},
   20: {'x':  200.0, 'y':  100.0, 'z': 0.0, 'segment_id': 'c20'},
   21: {'x': -200.0, 'y':  200.0, 'z': 0.0, 'segment_id': 'c21'},
   22: {'x': -100.0, 'y':  200.0, 'z': 0.0, 'segment_id': 'c22'},
   23: {'x':    0.0, 'y':  200.0, 'z': 0.0, 'segment_id': 'c23'},
   24: {'x':  100.0, 'y':  200.0, 'z': 0.0, 'segment_id': 'c24'},
   25: {'x':  200.0, 'y':  200.0, 'z': 0.0, 'segment_id': 'c25'},
}

EDGES = {
    1: {2: 100.0, 6: 100.0},
    2: {1: 100.0, 3: 100.0, 7: 100.0},
    3: {2: 100.0, 4: 100.0, 8: 100.0},
    4: {3: 100.0, 5: 100.0, 9: 100.0},
    5: {4: 100.0, 10: 100.0},
    6: {1: 100.0, 7: 100.0, 11: 100.0},
    7: {2: 100.0, 6: 100.0, 8: 100.0, 12: 100.0},
    8: {3: 100.0, 7: 100.0, 9: 100.0, 13: 100.0},
    9: {4: 100.0, 8: 100.0, 10: 100.0, 14: 100.0},
   10: {5: 100.0, 9: 100.0, 15: 100.0},
   11: {6: 100.0, 12: 100.0, 16: 100.0},
   12: {7: 100.0, 11: 100.0, 13: 100.0, 17: 100.0},
   13: {8: 100.0, 12: 100.0, 14: 100.0, 18: 100.0},
   14: {9: 100.0, 13: 100.0, 15: 100.0, 19: 100.0},
   15: {10: 100.0, 14: 100.0, 20: 100.0},
   16: {11: 100.0, 17: 100.0, 21: 100.0},
   17: {12: 100.0, 16: 100.0, 18: 100.0, 22: 100.0},
   18: {13: 100.0, 17: 100.0, 19: 100.0, 23: 100.0},
   19: {14: 100.0, 18: 100.0, 20: 100.0, 24: 100.0},
   20: {15: 100.0, 19: 100.0, 25: 100.0},
   21: {16: 100.0, 22: 100.0},
   22: {17: 100.0, 21: 100.0, 23: 100.0},
   23: {18: 100.0, 22: 100.0, 24: 100.0},
   24: {19: 100.0, 23: 100.0, 25: 100.0},
   25: {20: 100.0, 24: 100.0}
}

SEGMENT_MAP = {
    'h1_1': (1, 2), 'h1_2': (2, 3), 'h1_3': (3, 4), 'h1_4': (4, 5),
    'h2_1': (6, 7), 'h2_2': (7, 8), 'h2_3': (8, 9), 'h2_4': (9, 10),
    'h3_1': (11, 12), 'h3_2': (12, 13), 'h3_3': (13, 14), 'h3_4': (14, 15),
    'h4_1': (16, 17), 'h4_2': (17, 18), 'h4_3': (18, 19), 'h4_4': (19, 20),
    'h5_1': (21, 22), 'h5_2': (22, 23), 'h5_3': (23, 24), 'h5_4': (24, 25),
    'v1_1': (1, 6), 'v1_2': (6, 11), 'v1_3': (11, 16), 'v1_4': (16, 21),
    'v2_1': (2, 7), 'v2_2': (7, 12), 'v2_3': (12, 17), 'v2_4': (17, 22),
    'v3_1': (3, 8), 'v3_2': (8, 13), 'v3_3': (13, 18), 'v3_4': (18, 23),
    'v4_1': (4, 9), 'v4_2': (9, 14), 'v4_3': (14, 19), 'v4_4': (19, 24),
    'v5_1': (5, 10), 'v5_2': (10, 15), 'v5_3': (15, 20), 'v5_4': (20, 25)
}

EDGE_TO_SEGMENT = {}
for _seg_id, (_u, _v) in SEGMENT_MAP.items():
    EDGE_TO_SEGMENT[(_u, _v)] = _seg_id
    EDGE_TO_SEGMENT[(_v, _u)] = _seg_id

BLOCKED_EDGES = set()

# ==========================================
# PART 4: PATH PLANNING & HELPER FUNCTIONS
# ==========================================

def nearest_waypoint(x, y, z=0.0):
    best_id = None
    min_dist = float('inf')
    for w_id, data in WAYPOINTS.items():
        dist = math.hypot(data['x'] - x, data['y'] - y)
        if dist < min_dist:
            min_dist = dist
            best_id = w_id

    wp = WAYPOINTS[best_id].copy()
    wp['id'] = best_id
    return wp

def segment_of_nearest_waypoint(x, y, z=0.0):
    wp = nearest_waypoint(x, y, z)
    return wp.get('segment_id', 'unknown')

def get_segment_id_from_waypoints(u, v):
    return EDGE_TO_SEGMENT.get((u, v), None)

def block_segment(segment_id):
    count = 0
    if segment_id in SEGMENT_MAP:
        u, v = SEGMENT_MAP[segment_id]
        BLOCKED_EDGES.add((u, v))
        BLOCKED_EDGES.add((v, u))
        count += 1
    return count

def block_edges_near(ox, oy, dist_thresh=20.0):
    pass

def blocked_edge_count():
    return len(BLOCKED_EDGES) // 2

def plan_path(start_id, goal_id):
    if start_id not in WAYPOINTS or goal_id not in WAYPOINTS:
        return []

    queue = [(0.0, start_id, [])]
    visited = set()
    distances = {w_id: float('inf') for w_id in WAYPOINTS}
    distances[start_id] = 0.0

    while queue:
        cost, current, path = heapq.heappop(queue)

        if current in visited:
            continue
        visited.add(current)
        path = path + [current]

        if current == goal_id:
            result = []
            for p in path:
                wp = WAYPOINTS[p].copy()
                wp['id'] = p
                result.append(wp)
            return result

        for neighbor, weight in EDGES[current].items():
            if (current, neighbor) in BLOCKED_EDGES:
                continue

            new_cost = cost + weight
            if new_cost < distances[neighbor]:
                distances[neighbor] = new_cost
                heapq.heappush(queue, (new_cost, neighbor, path))

    return []