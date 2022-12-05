import numpy as np
import math

# def obstacle_avoidance(goal, laser_scan, thetas):
#     goal_x = goal[0]
#     goal_y = goal[1]

#     # Initialize forward and rotational speeds to zero
#     forward_speed = 0
#     rotational_speed = 0
    
#     # Calculate the vector from the robot to the goal
#     goal_vector = np.array([goal_x, goal_y])
    
#     # Calculate the magnitude of the goal vector
#     goal_magnitude = np.linalg.norm(goal_vector)
    
#     # Calculate the unit vector of the goal vector
#     goal_unit = goal_vector / goal_magnitude
    
#     # Loop through the laser scan and calculate the repulsive vectors
#     for idx, distance in enumerate(laser_scan):
#         # Calculate the angle of the obstacle from the robot's perspective
#         angle = thetas[idx]
        
#         # Calculate the repulsive vector
#         repulsive_vector = np.array([np.cos(angle), np.sin(angle)])
        
#         # Calculate the magnitude of the repulsive vector
#         repulsive_magnitude = 1 / distance
        
#         # Calculate the unit vector of the repulsive vector
#         repulsive_unit = repulsive_vector / repulsive_magnitude
        
#         # Add the repulsive vector to the goal vector
#         goal_vector += repulsive_unit
    
#     # Calculate the magnitude of the updated goal vector
#     goal_magnitude = np.linalg.norm(goal_vector)
    
#     # Calculate the unit vector of the updated goal vector
#     goal_unit = goal_vector / goal_magnitude
    
#     # Set the forward speed to the magnitude of the updated goal vector
#     forward_speed = goal_magnitude / 1000
    
#     # Set the rotational speed to the angle of the updated goal vector
#     rotational_speed = np.arctan2(goal_unit[1], goal_unit[0])
    
#     return forward_speed, rotational_speed

class ObstacleAvoidance:
    def __init__(self):
        # Robot dimensions
        self.width = 0.38 # meters
        self.length = 0.44 # meters

        self.max_speed = 0.7

        self.attractive_factor = 1.2
        self.repulsion_factor = 0.005

      
    def obstacle_avoidance(self, goal, distances, thetas):
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
        forward_speed = min(force_distance, 1) # Max forward speed of 1
        rotational_speed = np.arctan2(desired_direction[1], desired_direction[0]) # Max rotational speed of 0.6
        rotational_speed = min(rotational_speed, 0.6)

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