from queue import PriorityQueue
import time
from autonavsim2d.utils.pose import Pose, Point, Orientation
from autonavsim2d.utils.pose_stamped import PoseStamped, Header
import math
import numpy as np


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



