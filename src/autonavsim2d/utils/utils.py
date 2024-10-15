import time
import math
import numpy as np

from queue import PriorityQueue
from autonavsim2d.utils.pose import Pose, Point, Orientation
from autonavsim2d.utils.pose_stamped import PoseStamped, Header


pq = PriorityQueue()

# colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (111, 0, 0)
GREEN = (0, 255, 0)
GREY = (192, 192, 192)
ORANGE = (255, 128, 0)
BLUE = (0, 0, 102)
RED_LIGHT = (255, 0, 0)
GREY_LIGHT = (220, 220, 220)
GOLD = (255, 215, 0)

def generate_path_custom(custom_planner, grid, matrix, start, end):
     # params
    GRID_LEN = len(grid)
    GRID_WIDTH = len(grid[0])

    # inflate map
    for i in range(GRID_LEN):
        for j in range(GRID_WIDTH):
            # generate node neighbors
            row, col = grid[i][j][2]
            node_color = grid[i][j][1]
            

            if node_color == GREY or node_color == RED or node_color == GREEN or node_color == BLUE or node_color == ORANGE:
                pass

            else:
                neighbors = [
                    (row - 1, col),
                    (row + 1, col),
                    (row, col - 1),
                    (row, col + 1)
                ]
                valid_neighbors = []

                # convert free space to obstacle by one cell
                for neighbor in neighbors:
                    n_row, n_col = neighbor

                    # check valid neighbors
                    if 0 <= n_row < GRID_LEN and 0 <= n_col < GRID_WIDTH:
                        valid_neighbors.append(neighbor)

                        if matrix[n_row][n_col] == 0:
                            pass

                        elif matrix[n_row][n_col] == 1:
                            # convert free space to obstacle
                            matrix[n_row][n_col] = 0
                
                # generate neighbors of valid neighbors of current node
                for neighbor_val in valid_neighbors:
                    row_val, col_val = neighbor_val

                    valid_neighbor_neighbors = [
                        (row_val - 1, col_val),
                        (row_val + 1, col_val),
                        (row_val, col_val - 1),
                        (row_val, col_val + 1)
                    ]

                    # convert free space to obstacle by one cell
                    for neighbor_ in valid_neighbor_neighbors:
                        n_row, n_col = neighbor_

                        # check valid neighbors
                        if 0 <= n_row < GRID_LEN and 0 <= n_col < GRID_WIDTH:
                            if matrix[n_row][n_col] == 0:
                                pass

                            elif matrix[n_row][n_col] == 1:
                                # convert free space to obstacle
                                matrix[n_row][n_col] = 0
                                
    shortest_path, runtime = custom_planner(grid, matrix, start, end)

    # return shortest path
    return (shortest_path, runtime)


def generate_path(grid, matrix, start, end):
    # params
    GRID_LEN = len(grid)
    GRID_WIDTH = len(grid[0])

    # inflate map
    for i in range(GRID_LEN):
        for j in range(GRID_WIDTH):
            # generate node neighbors
            row, col = grid[i][j][2]
            node_color = grid[i][j][1]
            

            if node_color == GREY or node_color == WHITE or node_color == RED or node_color == GREEN or node_color == BLUE or node_color == ORANGE:
                pass

            else:
                neighbors = [
                    (row - 10, col),
                    (row + 10, col),
                    (row, col - 10),
                    (row, col + 10)
                ]

                angled_neighbors = [
                    (row - 10, col - 10),
                    (row - 10, col + 10),
                    (row + 10, col - 10),
                    (row + 10, col + 10)
                ]

                valid_neighbors = []
                valid_angled_neighbors = []

                # convert adjacent neighbors to obstacles by 10 cells
                for neighbor in neighbors:
                    n_row, n_col = neighbor

                    # check valid neighbors
                    if 0 <= n_row < GRID_LEN and 0 <= n_col < GRID_WIDTH:
                        valid_neighbors.append(neighbor)

                        if matrix[n_row][n_col] == 0:
                            pass

                        elif matrix[n_row][n_col] == 1:
                            # convert free space to obstacle
                            matrix[n_row][n_col] = 0
                
                # convert angled neighbors to obstacles by 10 cells
                for angled_neighbor in angled_neighbors:
                    n_row, n_col = angled_neighbor

                    # check valid neighbors
                    if 0 <= n_row < GRID_LEN and 0 <= n_col < GRID_WIDTH:
                        valid_angled_neighbors.append(angled_neighbor)

                        if matrix[n_row][n_col] == 0:
                            pass

                        elif matrix[n_row][n_col] == 1:
                            # convert free space to obstacle
                            matrix[n_row][n_col] = 0
                
                # generate neighbors of valid neighbors of current node
                for neighbor_val in valid_neighbors:
                    row_val, col_val = neighbor_val

                    valid_neighbor_neighbors = [
                        (row_val - 1, col_val),
                        (row_val + 1, col_val),
                        (row_val, col_val - 1),
                        (row_val, col_val + 1)
                    ]

                    # convert free space to obstacle by one cell
                    for neighbor_ in valid_neighbor_neighbors:
                        n_row, n_col = neighbor_

                        # check valid neighbors
                        if 0 <= n_row < GRID_LEN and 0 <= n_col < GRID_WIDTH:
                            if matrix[n_row][n_col] == 0:
                                pass

                            elif matrix[n_row][n_col] == 1:
                                # convert free space to obstacle
                                matrix[n_row][n_col] = 0
                
                # generate angled neighbors of valid neighbors of current node
                for angled_neighbor_val in valid_angled_neighbors:
                    row_val, col_val = angled_neighbor_val

                    valid_angled_neighbor_neighbors = [
                        (row_val - 1, col_val),
                        (row_val + 1, col_val),
                        (row_val, col_val - 1),
                        (row_val, col_val + 1)
                    ]

                    # convert free space to obstacle by one cell
                    for neighbor_ in valid_angled_neighbor_neighbors:
                        n_row, n_col = neighbor_

                        # check valid neighbors
                        if 0 <= n_row < GRID_LEN and 0 <= n_col < GRID_WIDTH:
                            if matrix[n_row][n_col] == 0:
                                pass

                            elif matrix[n_row][n_col] == 1:
                                # convert free space to obstacle
                                matrix[n_row][n_col] = 0
                
                                
    
    shortest_path, runtime = dijkstra_v2(grid, matrix, start, end)

    # return shortest path
    return (shortest_path, runtime)


def generate_waypoints_v4(grid, matrix, path, start, end, WIN_WIDTH, WIN_HEIGHT):

    # generated posestamps
    waypoints = [None for _ in range(len(path) - 1)]

    # params
    visted = set()
    counter = 0
    prev_angle = None
    current_angle = 0

    # robot pose
    start_cell = start[0]
    rect_x = start_cell.x
    rect_y = start_cell.y

    rect_center_x = rect_x + start_cell.width // 2
    rect_center_y = rect_y + start_cell.height // 2

    robot_position = Point(x=rect_center_x, y=rect_center_y, z=0)
    robot_orientation = Orientation(0, 0, 0, math.pi/2)
    robot_pose = Pose(position=robot_position, orientation=robot_orientation)

    # generate pose stamp of waypoints
    PATH_LEN = len(path)
    for i in range(1, PATH_LEN, 1):

        if i < PATH_LEN-1:
            if i+1 < PATH_LEN:
                if waypoints[counter] == None:
                    # current waypoint
                    waypoint = path[counter]

                    # next waypoint
                    waypoint_next = path[counter+1]

                    # generate current waypoint neighbors
                    neighbors = [
                        (waypoint[0], waypoint[1]+1, 0),   # 0deg
                        (waypoint[0], waypoint[1]-1, math.pi),   # 180deg
                        (waypoint[0]+1, waypoint[1], math.pi*1.5),   # 270deg
                        (waypoint[0]-1, waypoint[1], math.pi/2)    # 90
                    ]

                    # find current waypoint heading based on next waypoint
                    for i in range(len(neighbors)):
                        neighbor = (neighbors[i][0], neighbors[i][1])
                        angle = neighbors[i][2]

                        if neighbor not in visted:
                            if neighbor == waypoint_next:   # found
                                current_angle = angle
                                visted.add(neighbor)
                                break
                    
                    
                    if prev_angle == None:
                        # current waypoint
                        cell = grid[waypoint[0]][waypoint[1]][0]
                        waypoint_center_x = cell.x + cell.width // 2
                        waypoint_center_y = cell.y +  cell.height // 2
                        
                        pt = Point(x=waypoint_center_x, y=waypoint_center_y, z=0)
                        orientation = Orientation(x=0, y=0, z=0, w=current_angle)
                        waypoint_pose = Pose(position=pt, orientation=orientation)
                        header = Header(stamp='0', frame_id='map')
                        waypoint_ps = PoseStamped(header=header, pose=waypoint_pose)
                        waypoints[counter] = waypoint_ps
                    
                    else:
                        # current waypoint
                        cell = grid[waypoint[0]][waypoint[1]][0]
                        waypoint_center_x = cell.x + cell.width // 2
                        waypoint_center_y = cell.y +  cell.height // 2
                        
                        pt = Point(x=waypoint_center_x, y=waypoint_center_y, z=0)
                        orientation = Orientation(x=0, y=0, z=0, w=prev_angle)
                        waypoint_pose = Pose(position=pt, orientation=orientation)
                        header = Header(stamp='0', frame_id='map')
                        waypoint_ps = PoseStamped(header=header, pose=waypoint_pose)
                        waypoints[counter] = waypoint_ps
                    
                    prev_angle = current_angle

            else:
                pass

        else:
            
            # last waypoint reached
            waypoint_lt = path[counter]
            cell_lt = grid[waypoint_lt[0]][waypoint_lt[1]][0]
            waypoint_center_x_lt = cell_lt.x + cell_lt.width // 2
            waypoint_center_y_lt = cell_lt.y +  cell_lt.height // 2
            
            # set pose stamp of last waypoint
            angle_lt = prev_angle
            pt_lt = Point(x=waypoint_center_x_lt, y=waypoint_center_y_lt, z=0)
            orientation_lt = Orientation(x=0, y=0, z=0, w=angle_lt)
            waypoint_pose_lt = Pose(position=pt_lt, orientation=orientation_lt)
            header_lt = Header(stamp='0', frame_id='map')
            waypoint_ps_lt = PoseStamped(header=header_lt, pose=waypoint_pose_lt)

            # add last waypoint to waypoint list
            waypoints[counter] = waypoint_ps_lt

        # increment counter
        counter += 1

    # add goal location to waypoints
    end_cell = end[0]
    rect_x_end = end_cell.x
    rect_y_end = end_cell.y

    rect_center_x_end = rect_x_end + end_cell.width // 2
    rect_center_y_end = rect_y_end + end_cell.height // 2
    
    # set pose stamp of last waypoint
    angle_lt = prev_angle
    pt_lt = Point(x=rect_center_x_end, y=rect_center_y_end, z=0)
    orientation_lt = Orientation(x=0, y=0, z=0, w=angle_lt)
    waypoint_pose_lt = Pose(position=pt_lt, orientation=orientation_lt)
    header_lt = Header(stamp='0', frame_id='map')
    waypoint_ps_lt = PoseStamped(header=header_lt, pose=waypoint_pose_lt)
    waypoints.append(waypoint_ps_lt)

    return(robot_pose, waypoints)


def compare_waypoints(point1, point2):
    # point 1
    point1_x = point1[0]
    point1_y = point1[1]

    # point 2
    point2_x = point2[0]
    point2_y = point2[1]

    if abs(point1_x - point2_x) <= 2 and abs(point1_y - point2_y) <= 2:
        return True
    else:
        return False


def parse_arrow_angle(angle):
    angle_ = math.degrees(angle)
    true_angle = 0

    if angle_ == 0:
        true_angle = math.pi*1.5

    elif angle_ == 90:
        true_angle = 0

    elif angle_ == 180:
        true_angle = math.pi/2

    elif angle_ == 270:
        true_angle = math.pi

    return true_angle


def dijkstra_v2(grid, matrix, start, end):
    # start timer
    start_time = time.time()
    
    n = len(matrix)
    m = len(matrix[0])

    distances = [[float('inf')] * m for _ in range(n)]
    distances[start[0]][start[1]] = 0

    prev_nodes = [[None] * m for _ in range(n)]
    visited = set()

    pq.put((0, start))

    while not pq.empty():
        dist, node = pq.get()

        if node == end:
            break

        if node in visited:
            continue

        visited.add(node)

        row, col = node
        neighbors = [(row - 1, col), (row + 1, col),
                     (row, col - 1), (row, col + 1)]
        for neighbor in neighbors:
            n_row, n_col = neighbor
            if 0 <= n_row < n and 0 <= n_col < m:
                if matrix[n_row][n_col] == 1:
                    distance = dist + 1
                    if distance < distances[n_row][n_col]:
                        distances[n_row][n_col] = distance
                        prev_nodes[n_row][n_col] = node
                        pq.put((distance, (n_row, n_col)))

    path = []
    current = end
    while current is not None:
        path.append(current)
        current = prev_nodes[current[0]][current[1]]

    # end timer
    end_time = time.time()
    runtime = end_time - start_time

    # Reverse the path and return
    return (path[::-1], runtime)


def generate_curve_points(p0, p1, p2, num_points):
    points = []
    for t in range(num_points):
        t /= num_points - 1
        x = int((1 - t) ** 2 * p0[0] + 2 * (1 - t) * t * p1[0] + t ** 2 * p2[0])
        y = int((1 - t) ** 2 * p0[1] + 2 * (1 - t) * t * p1[1] + t ** 2 * p2[1])
        points.append((x, y))
    return points


def generate_cubic_curve_points(p0, p1, p2, p3, num_points):
    points = []
    for t in range(num_points):
        t /= num_points - 1
        x = int((1 - t) ** 3 * p0[0] + 3 * (1 - t) ** 2 * t * p1[0] + 3 * (1 - t) * t ** 2 * p2[0] + t ** 3 * p3[0])
        y = int((1 - t) ** 3 * p0[1] + 3 * (1 - t) ** 2 * t * p1[1] + 3 * (1 - t) * t ** 2 * p2[1] + t ** 3 * p3[1])
        points.append((x, y))
    return points


def generate_line_points(p0, p1, num_points):
    points = []
    for t in range(num_points):
        t /= num_points - 1
        x = int((1 - t) * p0[0] + t * p1[0])
        y = int((1 - t) * p0[1] + t * p1[1])
        points.append((x, y))
    return points


def sdv_track_1():
    # control points
    p1 = (700, 900)  # Start point
    p2 = (700, 500)  # End point
    ca = (900, 600)  # Control point

    p3 = (700, 500)  # Start point
    p4 = (650, 350)  # End point
    cb = (600, 450)  # Control point
    
    p5 = (695, 260)  # Start point
    p6 = (560, 100)  # End point
    cc = (800, 10)  # Control point

    p9 = (560, 100)  # Start point
    p10 = (400, 500)  # End point
    ce = (350, 150)  # Control point

    p7 = (400, 500)  # Start point
    p8 = (700, 900)  # End point
    cd = (450, 750)  # Control point

    # lines
    pl1 = (650, 350)
    pl2 = (695, 260)
    cla = (450, 150)
    num_points = 100

    curve_points1 = generate_curve_points(p1, ca, p2, num_points)
    curve_points2 = generate_curve_points(p3, cb, p4, num_points)
    curve_points3 = generate_curve_points(p5, cc, p6, num_points)
    curve_points4 = generate_curve_points(p7, cd, p8, num_points)
    curve_points5 = generate_curve_points(p9, ce, p10, num_points)
    line_points1 = generate_line_points(pl1, pl2, 100)

    # add first curve to list
    curve_points = [pt for pt in curve_points1]

    # add second curve to points list
    for i in range(len(curve_points2)):
        if i == 0:
            pass

        else:
            curve_points.append(curve_points2[i])

    # add line
    for i in range(len(line_points1)):
            if i == 0:
                pass

            else:
                curve_points.append(line_points1[i])

    # add third curve to points list
    for i in range(len(curve_points3)):
        if i == 0:
            pass

        else:
            curve_points.append(curve_points3[i])

    # add fourth curve to points list
    for i in range(len(curve_points4)):
        if i == 0:
            pass

        else:
            curve_points.append(curve_points4[i])

    # add fifth curve to points list
    for i in range(len(curve_points5)):
        if i == 0:
            pass

        else:
            curve_points.append(curve_points5[i])

    return curve_points, curve_points1, curve_points2, curve_points3, curve_points4, curve_points5, line_points1