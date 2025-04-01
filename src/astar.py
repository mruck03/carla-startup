import heapq
import math
import numpy as np
import matplotlib.pyplot as plt


class Node:
    def __init__(self, x, y, cost, pind):
        self.x = x  # x position of node
        self.y = y  # y position of node
        self.cost = cost  # g cost of node
        self.pind = pind  # parent index of node

class Para:
    def __init__(self, minx, miny, maxx, maxy, xw, yw, reso, motion):
        self.minx = minx
        self.miny = miny
        self.maxx = maxx
        self.maxy = maxy
        self.xw = xw
        self.yw = yw
        self.reso = reso  # resolution of grid world
        self.motion = motion  # motion set


def astar_planning(sx, sy, gx, gy, ox, oy, reso, rr):
    """
    return path of A*.
    :param sx: starting node x [m]
    :param sy: starting node y [m]
    :param gx: goal node x [m]
    :param gy: goal node y [m]
    :param ox: obstacles x positions [m]
    :param oy: obstacles y positions [m]
    :param reso: xy grid resolution
    :param rr: robot radius
    :return: path
    """

    n_start = Node(round(sx / reso), round(sy / reso), 0.0, -1)
    n_goal = Node(round(gx / reso), round(gy / reso), 0.0, -1)

    ox = [x / reso for x in ox]
    oy = [y / reso for y in oy]

    P, obsmap = calc_parameters(ox, oy, rr, reso)

    # open_set = nodes that are yet to be explored
    # REROUTING: In our application, the car might reroute back to the same node 
    #            e.g if there's no car avaialble, then we might need to remove closed_set in that case
    # closed_set = nodes that are explored
    open_set, closed_set = dict(), dict()
    open_set[calc_index(n_start, P)] = n_start

    q_priority = []
    # fvalue =  g-cost + heuristic cost
    heapq.heappush(q_priority,
                   (fvalue(n_start, n_goal), calc_index(n_start, P)))

    while True:
        if not open_set:
            break

        _, ind = heapq.heappop(q_priority)
        n_curr = open_set[ind]
        closed_set[ind] = n_curr
        open_set.pop(ind)

        for i in range(len(P.motion)):
            node = Node(n_curr.x + P.motion[i][0],
                        n_curr.y + P.motion[i][1],
                        n_curr.cost + u_cost(P.motion[i]), ind)

            # If out of bound or hit something, we return 
            if not check_node(node, P, obsmap):
                continue
            
            n_ind = calc_index(node, P)
            if n_ind not in closed_set:
                if n_ind in open_set:
                    if open_set[n_ind].cost > node.cost:
                        open_set[n_ind].cost = node.cost
                        open_set[n_ind].pind = ind
                else:
                    open_set[n_ind] = node
                    heapq.heappush(q_priority,
                                   # calc_index(node, P) here repeated?
                                   (fvalue(node, n_goal), calc_index(node, P)))

    pathx, pathy = extract_path(closed_set, n_start, n_goal, P)

    return pathx, pathy


def optimized_u_cost(motion):
    return abs(motion[0]) + abs(motion[1])  # Example heuristic

def calc_holonomic_heuristic_with_obstacle(node, ox, oy, xmin, xmax, ymin, ymax, reso, rr):
    n_goal = Node(round(node.x[-1] / reso), round(node.y[-1] / reso), 0.0, -1)

    ox = np.array(ox) / reso
    oy = np.array(oy) / reso

    P, obsmap = calc_parameters(ox, oy, xmin, xmax, ymin, ymax, reso, rr)

    open_set = {}
    closed_set = set()  # Use a set for fast lookups
    closed_nodes = {} 
    n_goal_index = calc_index(n_goal, P)
    open_set[n_goal_index] = n_goal

    q_priority = []
    heapq.heappush(q_priority, (n_goal.cost, n_goal_index))

    while open_set:
        _, ind = heapq.heappop(q_priority)
        n_curr = open_set.pop(ind)
        closed_set.add(ind)
        closed_nodes[ind] = n_curr 

        for motion in P.motion:
            new_x, new_y = n_curr.x + motion[0], n_curr.y + motion[1]
            cost = n_curr.cost + optimized_u_cost(motion)

            node = Node(new_x, new_y, cost, ind)

            if not check_node(node, P, obsmap):
                continue

            n_ind = calc_index(node, P)
            if n_ind in closed_set:
                continue

            if n_ind not in open_set or open_set[n_ind].cost > cost:
                open_set[n_ind] = node
                heapq.heappush(q_priority, (cost, n_ind))

    hmap = np.full((P.xw, P.yw), np.inf)  # Faster than list comprehension

    indices = np.array(list(closed_nodes.keys()))  # Get all indices
    nodes = np.array([closed_nodes[ind] for ind in indices])  # Extract all Node objects

    x_indices = np.array([node.x for node in nodes]) - P.minx
    y_indices = np.array([node.y for node in nodes]) - P.miny
    costs = np.array([node.cost for node in nodes])

    hmap[x_indices, y_indices] = costs

    return hmap


def check_node(node, P, obsmap):
    if node.x <= P.minx or node.x >= P.maxx or \
            node.y <= P.miny or node.y >= P.maxy:
        return False

    if obsmap[node.x - P.minx][node.y - P.miny]:
        return False

    return True


def u_cost(u):
    """
    
                u
               /|
return ->     / | u[1]
             /  |              
        start ---
              u[0]
    """
    return math.hypot(u[0], u[1])


def fvalue(node, n_goal):
    return node.cost + h(node, n_goal)


# This heuristic is extremely simple, it just checks the distance 
# between the current node and the end node
# TODO: maybe explore a better heuristic for our application.
def h(node, n_goal):
    return math.hypot(node.x - n_goal.x, node.y - n_goal.y)


def calc_index(node, P):
    return (node.y - P.miny) * P.xw + (node.x - P.minx)


def calc_parameters(ox, oy, minx, maxx, miny, maxy, rr, reso):
    # Might need to remove round in CARLA...
    # minx, miny = round(min(ox)), round(min(oy))
    # maxx, maxy = round(max(ox)), round(max(oy))
    
    xw, yw = maxx - minx, maxy - miny

    motion = get_motion()
    P = Para(minx, miny, maxx, maxy, xw, yw, reso, motion)
    # Calcualte the obstacles around the car given the radius
    # This might need be called constantly in CARLA
    # OR, this obsmap will be updated by our lidar ONLINE
    obsmap = calc_obsmap(ox, oy, rr, P)

    return P, obsmap


def calc_obsmap(ox, oy, rr, P):
    # obsmap = [[False for _ in range(P.yw)] for _ in range(P.xw)]

    # for x in range(P.xw):
    #     xx = x + P.minx
    #     for y in range(P.yw):
    #         yy = y + P.miny
    #         for oxx, oyy in zip(ox, oy):
    #             if math.hypot(oxx - xx, oyy - yy) <= rr / P.reso:
    #                 obsmap[x][y] = True
    #                 break
    # Create grid coordinates
    x_range = np.arange(P.xw) + P.minx  # Shape: (P.xw,)
    y_range = np.arange(P.yw) + P.miny  # Shape: (P.yw,)
    
    xx, yy = np.meshgrid(x_range, y_range, indexing="ij")  # Shape: (P.xw, P.yw)

    # Compute distances from all obstacles
    ox = np.array(ox)[:, None, None]  # Shape: (num_obstacles, 1, 1)
    oy = np.array(oy)[:, None, None]  # Shape: (num_obstacles, 1, 1)

    distances = np.sqrt((ox - xx) ** 2 + (oy - yy) ** 2)  # Shape: (num_obstacles, P.xw, P.yw)

    # Check if any obstacle is within range
    obsmap = np.any(distances <= (rr / P.reso), axis=0)  # Shape: (P.xw, P.yw)

    return obsmap


def extract_path(closed_set, n_start, n_goal, P):
    pathx, pathy = [n_goal.x], [n_goal.y]
    n_ind = calc_index(n_goal, P)

    while True:
        node = closed_set[n_ind]
        pathx.append(node.x)
        pathy.append(node.y)
        n_ind = node.pind

        if node == n_start:
            break

    pathx = [x * P.reso for x in reversed(pathx)]
    pathy = [y * P.reso for y in reversed(pathy)]

    return pathx, pathy


def get_motion():
    # Might need to figure out how to change this in CARLA
    # We might also need to figure out how fine it should be 
    motion = [[-1, 0], [-1, 1], [0, 1], [1, 1],
              [1, 0], [1, -1], [0, -1], [-1, -1]]

    return motion


def get_env():
    ox, oy = [], []

    for i in range(60):
        ox.append(i)
        oy.append(0.0)
    for i in range(60):
        ox.append(60.0)
        oy.append(i)
    for i in range(61):
        ox.append(i)
        oy.append(60.0)
    for i in range(61):
        ox.append(0.0)
        oy.append(i)
    for i in range(40):
        ox.append(20.0)
        oy.append(i)
    for i in range(40):
        ox.append(40.0)
        oy.append(60.0 - i)

    return ox, oy

"""
In CARLA we can assume we only need to model (x, y, theta)
The car doesn't elevate in a parking lot, so z is not needed
The car also doesn't rotate in the z and y axis, so we don't need 
pitch and roll 

"""

"""
Qs:
1. What if the parking spot is not known in advance?
2. What is the start point is not known in advance?
3. How do we update ox, oy if this is an online problem? 
    - I think hybrid A* already handle this 
4. How do we know if we hit something? 
5. How does an obstacle look like for CARLA programmatically?

"""
def main():
    # This corresonds to the starting point of Jake
    sx = 10.0  # [m]
    sy = 10.0  # [m]

    # This corresponds to the vacant parking spot 
    gx = 50.0  # [m]
    gy = 50.0  # [m]

    robot_radius = 2.0
    # Decrease this = increased accuracy and increase computational costs
    grid_resolution = 1.0
    ox, oy = get_env()

    pathx, pathy = astar_planning(sx, sy, gx, gy, ox, oy, grid_resolution, robot_radius)

    plt.plot(ox, oy, 'sk')
    plt.plot(pathx, pathy, '-r')
    plt.plot(sx, sy, 'sg')
    plt.plot(gx, gy, 'sb')
    plt.axis("equal")
    plt.show()


if __name__ == '__main__':
    main()
