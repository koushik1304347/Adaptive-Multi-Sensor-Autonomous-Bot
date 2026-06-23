import math
import numpy as np
from controller import Robot
import dijkstra_nav

# ================= CONFIG =================
WAYPOINT_RADIUS = 0.2
BASE_SPEED = 7.0
MAX_SPEED  = 10.0
DEPTH_DETECT_THRESH = 5.0
OBSTACLE_CONFIRM_STEPS = 2
TURN_SPEED = 3.5
BRAKE_STEPS = 25
SMALL_ANGLE = 0.05
BIG_ANGLE   = 0.5

# STATES
FOLLOW_PATH   = "FOLLOW_PATH"
TURN_IN_PLACE = "TURN_IN_PLACE"
REPLANNING    = "REPLANNING"

# DYNAMIC TARGETS (Will be set based on robot name)
START_WP_ID = 1
GOAL_WP_ID  = 18

# ================= HELPERS =================
def angle_diff(a, b):
    d = a - b
    while d > math.pi: d -= 2*math.pi
    while d < -math.pi: d += 2*math.pi
    return d

def heading_from_compass(cv):
    return math.atan2(cv[0], cv[1])

def get_depth_front_min(depth_data, width, height):
    depth = np.array(depth_data).reshape((height, width))
    h_start = int(height * 0.3)
    h_end   = int(height * 0.7)
    w_start = int(width * 0.3)
    w_end   = int(width * 0.7)
    region = depth[h_start:h_end, w_start:w_end]
    region = np.nan_to_num(region, nan=100, posinf=100, neginf=100)
    return np.min(region)

# ================= MAIN =================
def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    robot_name = robot.getName()

    # Dynamic Route Assignment
    global START_WP_ID, GOAL_WP_ID
    if robot_name == "moose":
        START_WP_ID, GOAL_WP_ID = 1, 18
    elif robot_name == "moose_2":
        START_WP_ID, GOAL_WP_ID = 16, 20
    else:
        START_WP_ID, GOAL_WP_ID = 1, 25

    # Sensors
    depth   = robot.getDevice("depth");   depth.enable(timestep)
    gps     = robot.getDevice("gps");     gps.enable(timestep)
    compass = robot.getDevice("compass"); compass.enable(timestep)
    
    # Communication Radios
    emitter = robot.getDevice("emitter")
    receiver = robot.getDevice("receiver")
    receiver.enable(timestep)

    motors = []
    for name in [
        "left motor 1","left motor 2","left motor 3","left motor 4",
        "right motor 1","right motor 2","right motor 3","right motor 4"
    ]:
        m = robot.getDevice(name)
        m.setPosition(float("inf"))
        m.setVelocity(0)
        motors.append(m)

    def set_speed(l, r):
        l = max(-MAX_SPEED, min(MAX_SPEED, l))
        r = max(-MAX_SPEED, min(MAX_SPEED, r))
        for i in range(4):
            motors[i].setVelocity(l)
            motors[i+4].setVelocity(r)

    path = dijkstra_nav.plan_path(START_WP_ID, GOAL_WP_ID)
    wp_index = 0

    state = FOLLOW_PATH
    obstacle_count = 0
    brake_counter = 0

    print(f"[{robot_name}] INITIAL PATH: {[p['id'] for p in path]}")

    while robot.step(timestep) != -1:

        # --- 1. LISTEN FOR RADIO MESSAGES ---
        new_blocks_found = False
        while receiver.getQueueLength() > 0:
            message = receiver.getString()
            receiver.nextPacket()
            try:
                parts = message.split(',')
                u, v = int(parts[0]), int(parts[1])
                if (u, v) not in dijkstra_nav.BLOCKED_EDGES:
                    dijkstra_nav.BLOCKED_EDGES.add((u, v))
                    dijkstra_nav.BLOCKED_EDGES.add((v, u))
                    new_blocks_found = True
                    print(f"[{robot_name}] Radio alert received! Edge {u}-{v} blocked globally.")
            except Exception:
                pass

        # --- 2. SENSOR READING ---
        gps_v = gps.getValues()
        comp_v = compass.getValues()
        rx, ry = gps_v[0], gps_v[1]
        heading = heading_from_compass(comp_v)

        depth_data = depth.getRangeImage()
        front_min = get_depth_front_min(depth_data, depth.getWidth(), depth.getHeight())

        # --- 3. STATE MACHINE ---
        if state == FOLLOW_PATH:
            
            # Dynamic Replanning due to remote obstacle
            if new_blocks_found and wp_index < len(path):
                path_compromised = False
                for i in range(max(0, wp_index - 1), len(path) - 1):
                    edge_u = path[i]["id"]
                    edge_v = path[i+1]["id"]
                    if (edge_u, edge_v) in dijkstra_nav.BLOCKED_EDGES:
                        path_compromised = True
                        break
                
                if path_compromised:
                    print(f"[{robot_name}] Path compromised by remote obstacle! Replanning...")
                    state = REPLANNING
                    continue

            if wp_index >= len(path):
                set_speed(0,0)
                print(f"[{robot_name}] DONE - Arrived at destination.")
                break

            target = path[wp_index]
            tx, ty = target["x"], target["y"]
            dist = math.hypot(tx - rx, ty - ry)

            if dist < WAYPOINT_RADIUS:
                wp_index += 1
                obstacle_count = 0
                continue

            desired = math.atan2(ty - ry, tx - rx)
            err = angle_diff(desired, heading)

            # Local Obstacle Detection
            if front_min < DEPTH_DETECT_THRESH:
                obstacle_count += 1
            else:
                obstacle_count = 0

            if obstacle_count >= OBSTACLE_CONFIRM_STEPS:
                print(f"[{robot_name}] OBSTACLE DETECTED at {front_min:.2f}m")
                set_speed(0,0)
                
                if wp_index > 0 and wp_index < len(path):
                    u = path[wp_index - 1]["id"]
                    v = path[wp_index]["id"]
                    
                    # Block locally
                    dijkstra_nav.BLOCKED_EDGES.add((u, v))
                    dijkstra_nav.BLOCKED_EDGES.add((v, u))
                    
                    # Broadcast to others
                    msg = f"{u},{v}"
                    emitter.send(msg.encode('utf-8'))
                    print(f"[{robot_name}] Broadcasted block: {msg}")
                
                state = REPLANNING
                continue

            # Turn Logic
            err_abs = abs(err)
            if err_abs < SMALL_ANGLE:
                set_speed(MAX_SPEED, MAX_SPEED)
            elif err_abs < BIG_ANGLE:
                turn = 5.0 * err
                turn = max(-4.0, min(4.0, turn))
                speed_scale = max(0.4, 1.0 - 1.5 * err_abs)
                fwd = BASE_SPEED * speed_scale
                set_speed(fwd - turn, fwd + turn)
            else:
                set_speed(0,0)
                brake_counter = BRAKE_STEPS
                state = TURN_IN_PLACE

        elif state == TURN_IN_PLACE:
            if brake_counter > 0:
                set_speed(0,0)
                brake_counter -= 1
                continue

            target = path[wp_index]
            tx, ty = target["x"], target["y"]
            desired = math.atan2(ty - ry, tx - rx)
            err = angle_diff(desired, heading)

            if abs(err) > SMALL_ANGLE:
                turn_vel = 5.0 * err 
                turn_vel = max(-TURN_SPEED, min(TURN_SPEED, turn_vel))
                set_speed(-turn_vel, turn_vel)
            else:
                set_speed(0, 0)
                #print(f"[{robot_name}] ALIGNED -> RESUMING PATH")
                state = FOLLOW_PATH

        elif state == REPLANNING:
            set_speed(0,0)
            nearest = dijkstra_nav.nearest_waypoint(rx, ry)
            start_id = nearest["id"]

            new_path = dijkstra_nav.plan_path(start_id, GOAL_WP_ID)

            if new_path:
                path = new_path
                wp_index = 0
                state = TURN_IN_PLACE
                brake_counter = BRAKE_STEPS
                print(f"[{robot_name}] NEW PATH: {[p['id'] for p in path]}")
            else:
                print(f"[{robot_name}] STUCK - No available path.")

if __name__ == "__main__":
    main()