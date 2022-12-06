import numpy as np
import math


class ObstacleAvoidance:
    def __init__(self):
        # Robot dimensions
        self.width = 0.38 # meters
        self.length = 0.44 # meters

        self.attractive_factor = 1.5 / 2
        self.repulsion_factor = 0.005 / 2

      
    def obstacle_avoidance(self, goal, distances, thetas):
        """ Avoid obstacles using artificial potential fields. The goal provides
        an attractive force on the robot, while obstacles create a repulsive
        force. These forces combine to create a total force on the robot, which
        is used to calculate forward and rotational speeds.
        """

        # Calculate attractive force
        goal_vector = goal
        goal_distance = np.linalg.norm(goal_vector)
        goal_direction = goal_vector / goal_distance
        attractive_force = goal_direction * goal_distance * self.attractive_factor

        # Calculate repulsive forces
        repulsive_forces = np.zeros(2)
        for i in range(len(distances)):
            # Skip distances that are too far away to affect the robot
            if distances[i] > 1:
                continue

            # Calculate direction and magnitude of repulsive force
            obstacle_vector = np.array([distances[i] * np.cos(thetas[i]), distances[i] * np.sin(thetas[i])])
            obstacle_distance = np.linalg.norm(obstacle_vector)
            obstacle_direction = -obstacle_vector / obstacle_distance
            repulsive_force = obstacle_direction / (obstacle_distance ** 2) * self.repulsion_factor

            # Add repulsive force to total repulsive forces
            repulsive_forces += repulsive_force

        # Calculate final force and desired direction
        total_force = attractive_force + repulsive_forces
        force_distance = np.linalg.norm(total_force)
        desired_direction = total_force / force_distance

        # Calculate forward and rotational speeds
        forward_speed = np.clip(force_distance, -0.75, 0.75) # Max forward speed of 1
        rotational_speed = np.arctan2(desired_direction[1], desired_direction[0]) # Max rotational speed of 0.6
        rotational_speed = np.clip(rotational_speed, -0.75, 0.75)

        x = np.abs(distances * np.sin(thetas))

        # Find indecies less than half the robot width (19cm) +25% buffer
        x_infront = x <= 0.19 * 1.15

        # Get thetas and scan distances
        thetas_infront = thetas[x_infront]
        scans_infront = distances[x_infront]

        # Find closest index
        idx_closest = np.argmin(scans_infront)

        # Get shortest scan distance and angle
        nearest_obstacle = scans_infront[idx_closest]
        nearest_obstacle_angle = thetas_infront[idx_closest] * 180/math.pi

        # TODO: and goal is not closer than obstacle check
        if nearest_obstacle < 0.5:
            forward_speed = 0

        return forward_speed, rotational_speed