import numpy as np

class PurePursuitController:
    def __init__(self, lookahead_distance):
        self.lookahead_distance = lookahead_distance

    def compute_steering_angle(self, current_position, path):
        # find the point on the path closest to the vehicle
        closest_point_index = self.find_closest_point(current_position, path)

        # find the point on the path that is lookahead_distance ahead of the vehicle
        target_point_index = self.find_target_point(current_position, path, closest_point_index)

        # compute the steering angle to reach the target point
        steering_angle = self.compute_steering_angle_to_point(current_position, path[target_point_index])

        return steering_angle

    def find_closest_point(self, current_position, path):
        # convert current_position to a 2D array
        current_position = np.array(current_position)[:2]  # consider only x, y coordinates

        # compute distances between current_position and each point in the path
        distances = np.linalg.norm(path - current_position, axis=1)

        # find the index of the closest point
        return np.argmin(distances)

    def find_target_point(self, current_position, path, closest_point_index):
        path = np.array(path)
        remaining_path = path[closest_point_index:]
        current_position = np.array(current_position)[:2]  # convert current position to 2D array

        for i, point in enumerate(remaining_path):
            if np.linalg.norm(current_position - point[:2]) > self.lookahead_distance:
                return closest_point_index + i

        # if no target point found within lookahead distance, return the last point
        return len(path) - 1
    
    def compute_steering_angle_to_point(self, current_position, target_point):
        dx = target_point[0] - current_position[0]
        dy = target_point[1] - current_position[1]
        desired_heading = np.arctan2(dy, dx)
        steering_angle = desired_heading - current_position[2]

        # normalize the steering angle to be within [-pi, pi]
        while steering_angle > np.pi:
            steering_angle -= 2 * np.pi
            
        while steering_angle < -np.pi:
            steering_angle += 2 * np.pi

        return steering_angle