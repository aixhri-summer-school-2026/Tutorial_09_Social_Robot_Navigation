#!/usr/bin/env python3
import math
import numpy as np
from geometry_msgs.msg import TwistStamped
import socket
import struct
import threading
import rclpy
from rclpy.node import Node
from socnav_msgs.msg import TrackedAgents


class SocialMPPINode(object):
    def __init__(self):
        """
        Initializes the parameters and sets the weights for MPPI
        """
        
        self.global_plan = None
        self.costmap = None
        self.pose = None
        self.twist = None
        self.cmd_vel = None
        ## TODO: Define tracked agents and their predictions
        self.tracked_agents = None
        self.agent_predictions = None

        ### Robot Parameters ###
        self.max_v = 1.0         # Max linear velocity of the robot
        self.max_w = 2.0         # Max angular velocity of the robot
        self.robot_radius = 0.2  # Radius of the robot

        ### MPPI Hyperparameters ###
        self.N = 50                         # Horizon of the rollout
        self.dt = 0.1                       # time step for rollouts
        self.ds = 0.1                       # ds = dt * max_vel -> distance step for rollouts
        self.K = 500                        # Number of parallel trajectory rollouts
        self.lambda_ = 2.0                  # Temperature parameter (lower = more aggressive tracking)
        self.sigma = np.array([0.3, 0.7])   # Standard deviation of control noise [v, w]

        # Weight for Different Constraints
        self.Q = np.array([10.0, 10.0, 2.0])    # Tracking weight for [x, y, theta]
        
        ## TODO: Part 1 -> Define a penalty for entering costmap  
        ## Define costmap penalty weight  
        self.w_costmap = 1.0                   
       
        
        ################### Part 2 ######################        
        ## TODO: Part 2-> Define tracked agents and their predictions
        ## Define tracked agents and their predictions 
        self.tracked_agents = None
        self.agent_predictions = None
        
        ## TODO: Part 2 -> Define a penalty for going very close to humans or violating the proxemics
        ## Define human proxemics penalty weight 
        self.w_human_proxem = 50.0              


        ## TODO: Part 2 -> Initialize human safety parameters
        ## 3. Define proxemics distance for human (in metres), 
        ## 4. Define radius of circumcircle for human (in metres)
        self.proxemic_dist = 0.6       
        self.human_radius = 0.3

        ## Visualization Initialization
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.viz_address = ("127.0.0.1", 9876)  # Stream locally to port 9876

    def setPath(self, global_plan):
        """
        Get the global plan from Nav2 Planner and store it in a class variable
        """
        self.global_plan = global_plan

    ## TODO: Part 2 -> Complete Tracked Agents callback
    def setTrackedAgents(self, tracked_agents):
        """
        Get the data from the tracked_agents topic and store it in a class variable
        """
        
        try:
            self.tracked_agents = tracked_agents
            try:
                self.predict_agent_paths()
            except Exception as e:
                print(f"Warning: predict_agent_paths failed: {e}", flush=True)
        except Exception as e:
            print(f"Error storing tracked_agents: {e}", flush=True)
            self.tracked_agents = None
            
            
    ## TODO: Part 2 -> Complete constant velocity prediction for humans
    def predict_agent_paths(self):
        """
        Class function to predict the motion of the tracked humans using a constant velocity model
        """
        
        if self.tracked_agents is None or not hasattr(self.tracked_agents, "agents"):
            self.agent_predictions = None
            return

        # Initialize structured list of coordinates tracking length of horizon steps N
        self.agent_predictions = [[] for _ in range(self.N)]

        for agent in self.tracked_agents.agents:
            # Skip stationary agents
            if (
                abs(agent.velocity.linear.x) < 0.01
                and abs(agent.velocity.linear.y) < 0.01
            ):
                continue

            init_x = agent.pose.position.x
            init_y = agent.pose.position.y
            vel_x = agent.velocity.linear.x
            vel_y = agent.velocity.linear.y

            # Linear extrapolation matching our rollouts time increments
            for step in range(self.N):
                future_time = step * self.dt
                pred_x = init_x + vel_x * future_time
                pred_y = init_y + vel_y * future_time
                self.agent_predictions[step].append([pred_x, pred_y])
                
                
    ## TODO: Part 2 -> Compute a vectorized human avoidance cost for the 'K' predictions at a time step 't'
    def compute_human_avoidance_cost(self, state_x, state_y, agent_predictions):
        """
        Vectorized cost computation for human avoidance
        """

        # If no agents are predicted or tracked, exit with zero cost instantly
        if agent_predictions is None or len(agent_predictions) == 0:
            return np.zeros_like(state_x)

        # Convert agent predictions at current timeline step to array [Num_Agents, 2]
        # Coordinates shape requirements: [K, 1] and [1, Num_Agents] for broadcasting matrix calculation
        agents_arr = np.array(agent_predictions)  # Assumed tracking active x,y frames
        hx = agents_arr[:, 0][np.newaxis, :]  # Shape: (1, Num_Agents)
        hy = agents_arr[:, 1][np.newaxis, :]  # Shape: (1, Num_Agents)

        sx = state_x[:, np.newaxis]  # Shape: (K, 1)
        sy = state_y[:, np.newaxis]  # Shape: (K, 1)

        # Broadmatrix Distance computation between all K samples and all tracked agents
        # Resulting shape: (K, Num_Agents)
        dist = np.sqrt((sx - hx) ** 2 + (sy - hy) ** 2) + 1e-6
        radius_sum = self.robot_radius + self.human_radius
        eff_dist = dist - radius_sum

        # Vectorized Condition Evaluation Masks
        cond_collision = eff_dist < 0
        cond_proxemic = (eff_dist >= 0) & (eff_dist < self.proxemic_dist)
        cond_safe = eff_dist >= self.proxemic_dist

        # Apply piece-wise functional equivalents matching your exact scaling mathematical intents
        choice_collision = 10.0 * np.abs(eff_dist + self.proxemic_dist)
        choice_proxemic = 10.0 * np.abs(eff_dist)
        choice_safe = eff_dist / 10.0

        # Select matching equations per cell element
        pair_costs = np.select(
            [cond_collision, cond_proxemic, cond_safe],
            [choice_collision, choice_proxemic, choice_safe],
        )

        # Sum penalties across all agents to get single total cost scalar per sample trajectory
        return np.sum(pair_costs, axis=1)

    def world_to_grid(self, x, y):
        """
        Helper function to convert world coordinates to costmap grid points
        """
        
        origin_x = self.costmap.info.origin.position.x
        origin_y = self.costmap.info.origin.position.y
        resolution = self.costmap.info.resolution

        gx = (x - origin_x) / resolution
        gy = (y - origin_y) / resolution
        return gx, gy
    
    def get_out_of_bounds_mask(self, x0, y0, x1, y1):
        """
        Helper function to check if the grid coordinates are out of bounds
        """
        out_of_bounds = (x0 < 0) | (x1 >= self.cost_map_width) | (y0 < 0) | (y1 >= self.cost_map_height)
        return out_of_bounds
    
    def get_cell_cost(self, x, y):
        """
        Helper function to get the cost of a cell in the costmap
        """
        # Convert to integer indices for flat indexing
        x0 = np.clip(x, 0, self.cost_map_width - 1)
        y0 = np.clip(y, 0, self.cost_map_height - 1)
        
        cost_data = np.asarray(self.costmap.data)

        c00 = cost_data[y0_clipped * width + x0_clipped].astype(np.uint8)
        c10 = cost_data[y0_clipped * width + x1_clipped].astype(np.uint8)
        c01 = cost_data[y1_clipped * width + x0_clipped].astype(np.uint8)
        c11 = cost_data[y1_clipped * width + x1_clipped].astype(np.uint8)

        # Perform batch bilinear interpolation math
        c0 = c00 * (1.0 - sx) + c10 * sx
        c1 = c01 * (1.0 - sx) + c11 * sx
        c = c0 * (1.0 - sy) + c1 * sy

        # Apply lethal cost override to any out-of-bounds samples
        c[out_of_bounds] = 255.0

        # If inputs were scalars, return a clean float; otherwise, return the array
        return c if len(c) > 1 else float(c[0])

    def computeVelocityCommands(self, costmap, pose, twist):
        """
        Main velocity function executed by Nav2
        """
        
        self.cmd_vel = TwistStamped()
        self.cmd_vel.header = pose.header
        self.costmap = costmap

        if self.global_plan is None or not self.global_plan.poses:
            return self.cmd_vel

        curr_yaw = self.get_yaw_from_pose(pose.pose)
        
        # Get current pose
        x0 = np.array([pose.pose.position.x, pose.pose.position.y, curr_yaw])

        # Generate random perturbations around nominal control sequence
        # Shape: [K, N, 2]
        noise = np.random.normal(0, self.sigma, size=(self.K, self.N, 2))
        U_samples = self.U_nominal + noise

        # Clamp samples to actual actuator limits [K, N, 2]
        U_samples[..., 0] = np.clip(U_samples[..., 0], 0.0, self.max_v)
        U_samples[..., 1] = np.clip(U_samples[..., 1], -self.max_w, self.max_w)

        # Vectorized Rollout Simulation across all K samples simultaneously
        # States shape: [K, 3] initialized to x0
        states = np.tile(x0, (self.K, 1))
        costs = np.zeros(self.K)
        predicted_trajectories = np.zeros((self.K, self.N, 3))

        # Get the global references for tracking
        g_refs = self.get_global_references(pose.pose)

        # Perform Rollouts
        for k in range(self.N):
            v = U_samples[:, k, 0]
            w = U_samples[:, k, 1]

            # Unicycle integration rolled out in parallel across all samples
            next_x = states[:, 0] + v * np.cos(states[:, 2]) * self.dt
            next_y = states[:, 1] + v * np.sin(states[:, 2]) * self.dt
            next_th = states[:, 2] + w * self.dt
            states = np.stack([next_x, next_y, next_th], axis=1)

            predicted_trajectories[:, k, :] = states

            # Compute vectorized error against reference step k
            err = states - g_refs[k]

            # Fast vectorized orientation wrap-around tracking
            err[:, 2] = np.arctan2(np.sin(err[:, 2]), np.cos(err[:, 2]))
            
            ################### Part 1 ######################
            ## TODO: Part 1: Get the interpolated costmap value (needs iterpolation to deal with discretization)
            ## 1. Complete the get_interpolated_costmap_value function
            ## 2. Get the cost associated with the robot entering the costmap
            ## 3. Store the costmap penalty in a variable called "costmap_penalty"
            
            costmap_penalties = self.get_interpolated_costmap_value(states[:, 0], states[:, 1])

            ## TODO: Part 2
            # Extract where all agents are predicted to be at exactly this future time-step k
            step_agent_coords = (self.agent_predictions[k] if self.agent_predictions else None)

            ## TODO: Part 2
            # Get the social penalty associated with the human proxemics violations
            social_penalties = self.compute_human_avoidance_cost(states[:, 0], states[:, 1], step_agent_coords)

            # Define the combined cost
            costs += (
                np.sum(err * err * self.Q, axis=1)           ## Quadratic tracking cost
                + (self.w_costmap * costmap_penalties)       ## Costmap penalty for entering costmap (weight * penalty)                        
                + (self.w_human_proxem * social_penalties)   ## Social penalty for human proxemics violations (weight * penalty)                          
            )

        # Softmax weighting of trajectories based on performance score
        costs_min = np.min(costs)
        weights = np.exp(-(costs - costs_min) / self.lambda_)
        weights /= np.sum(weights) + 1e-10

        # Elite Top-K Selection (Top 25 low cost trajectories out of 500 samples)
        K_elite = 25
        elite_indices = np.argsort(costs)[:K_elite]

        # Re-normalize weights only for those top 25 paths
        elite_weights = weights[elite_indices]
        elite_weights /= np.sum(elite_weights) + 1e-10

        # Average only the elite group for a smooth, decisive path
        self.U_nominal = np.sum(U_samples[elite_indices] * elite_weights[:, np.newaxis, np.newaxis], axis=0)

        # Extract the single best trajectory by calculating the weighted average path
        best_trajectory = np.sum(predicted_trajectories[elite_indices] * elite_weights[:, np.newaxis, np.newaxis], axis=0)

        ## Send data for visualization 
        try:
            g_refs_clean = g_refs[:, :2].astype(np.float64)
            best_traj_clean = best_trajectory[:, :2].astype(np.float64)
            samples_clean = predicted_trajectories[::20, :, :2].astype(
                np.float64
            )  # Every 20th sample
            robot_pose_clean = np.array(
                [pose.pose.position.x, pose.pose.position.y, curr_yaw]
            ).astype(np.float64)

            # We flatten the arrays so they can be packed into a fast, continuous memory stream
            data_payload = bytes()
            data_payload += struct.pack(
                "!I", len(g_refs_clean)
            )  # Send size of references
            data_payload += g_refs_clean.tobytes()

            data_payload += struct.pack(
                "!I", len(best_traj_clean)
            )  # Send size of optimal track
            data_payload += best_traj_clean.tobytes()

            # Send the sample rollouts shape [K_subsampled, N, 2]
            data_payload += struct.pack(
                "!II", samples_clean.shape[0], samples_clean.shape[1]
            )
            data_payload += samples_clean.tobytes()

            # Send the robot pose
            data_payload += robot_pose_clean.tobytes()

            # Non-blocking fire-and-forget network emit
            self.udp_sock.sendto(data_payload, self.viz_address)

        except Exception as e:
            print(
                f"============== VISUALIZATION STREAMING ERROR: {e} ==============",
                flush=True,
            )

        # Receding Horizon action
        v_cmd = float(self.U_nominal[0, 0])
        w_cmd = float(self.U_nominal[0, 1])

        # Shift timeline forward for the next iteration (warm start behavior)
        self.U_nominal = np.roll(self.U_nominal, -1, axis=0)
        self.U_nominal[-1] = np.array([0.0, 0.0])

        self.cmd_vel.twist.linear.x = v_cmd
        self.cmd_vel.twist.angular.z = w_cmd
        return self.cmd_vel

    ################### Part 1 ######################        
    def get_interpolated_costmap_value(self, x, y):
        """
        Vectorized bilinear interpolation for costmap lookup.
        Accepts both scalar coordinates and NumPy arrays of shape (K,).
        """
        ## TODO: Part 1 -> Compute a vectorized cost for entering costmap for the 'K' predictions at a time step 't'
        ## 1. Convert x and y to NumPy arrays for vectorized processing.
        ## 2. Transform world coordinates into costmap grid coordinates. Use the world_to_grid helper function.
        ## 3. Compute the surrounding grid cell indices. Make sure they are integer values.
        ## 4. Check for out-of-bounds points and create a mask using the "get_out_of_bounds_mask" helper function. Store it in "out_of_bounds". 
        ## 5. Calculate fractional interpolation weights across the batch.
        ## 6. Read the four neighboring cost values using the grid cells above and bilinearly interpolate them. Use the helper funtion "get_cell_cost".
        ## 7. Store the interpolated cost values in a variable called "costs".
        ## 7. Apply the out-of-bounds mask to "costs" to set any out-of-bounds point's cost to a high penalty value (e.g., 255).
        ## 8. Return a scalar for a single point or an array for batched coordinates of shape (K,).

         # Ensure inputs are numpy arrays for batch processing
        x = np.atleast_1d(x)
        y = np.atleast_1d(y)

        # Transform entire arrays from world coordinates to grid index floats
        gx, gy = self.world_to_grid(x, y)

        # Compute bounding cell coordinates across the entire batch
        x0 = np.floor(gx).astype(np.int32)
        x1 = x0 + 1
        y0 = np.floor(gy).astype(np.int32)
        y1 = y0 + 1

        # Vectorized Boundary Check
        # Create a boolean mask of any points that fall outside the costmap boundaries
        out_of_bounds = self.get_out_of_bounds_mask(x0, y0, x1, y1)
     

        # Calculate fractional interpolation weights across the batch
        sx = gx - x0
        sy = gy - y0

        # Extract neighbor cell costs using vectorized 1D array indexing
        cost_data = np.asarray(self.costmap.data)


        width = self.costmap.info.width
        height = self.costmap.info.height
        
        # Clip indices to prevent out-of-bounds access
        x0_clipped = np.clip(x0, 0, width - 1)
        x1_clipped = np.clip(x1, 0, width - 1)
        y0_clipped = np.clip(y0, 0, height - 1)
        y1_clipped = np.clip(y1, 0, height - 1)


        c00 = cost_data[y0_clipped * width + x0_clipped].astype(np.uint8)
        c10 = cost_data[y0_clipped * width + x1_clipped].astype(np.uint8)
        c01 = cost_data[y1_clipped * width + x0_clipped].astype(np.uint8)
        c11 = cost_data[y1_clipped * width + x1_clipped].astype(np.uint8)

        # Perform batch bilinear interpolation math
        c0 = c00 * (1.0 - sx) + c10 * sx
        c1 = c01 * (1.0 - sx) + c11 * sx
        costs = c0 * (1.0 - sy) + c1 * sy

        # Apply lethal cost override to any out-of-bounds samples
        costs[out_of_bounds] = 255.0

        # If inputs were scalars, return a clean float; otherwise, return the array
        return costs if len(costs) > 1 else float(costs[0])
    
    ################### Part 2 ######################         
    def setTrackedAgents(self, tracked_agents):
        """
        Get the data from the tracked_agents topic and store it in a class variable
        """
        ## TODO: Part 2 -> Complete Tracked Agents callback
        ## 1. Store the incoming tracked_agents message in a class variable.
        ## 2. Make sure the message contains valid agent data before continuing.
        ## 3. Trigger the human prediction step so future positions are available.
                        
        try:
            self.tracked_agents = tracked_agents
            try:
                self.predict_agent_paths()
            except Exception as e:
                print(f"Warning: predict_agent_paths failed: {e}", flush=True)
        except Exception as e:
            print(f"Error storing tracked_agents: {e}", flush=True)
            self.tracked_agents = None          
    
    ################### Part 2 ######################                 
    def predict_agent_paths(self):
        """
        Class function to predict the motion of the tracked humans using a constant velocity model
        """
        ## TODO: Part 2 -> Complete constant velocity prediction for humans
        ## 1. Check whether tracked_agents exists and contains agent data.
        ## 2. Initialize an empty prediction structure for the rollout horizon steps (size = self.N).
        ## 3. Loop over each tracked agent (e.g, like 'agent in self.tracked_agents.agents') and skip stationary ones as they are not needed.
        ## 4. Use the current position and velocity to predict future positions over time.
        ## 5. Store the predicted positions for each future time step.
        
        if self.tracked_agents is None or not hasattr(self.tracked_agents, "agents"):
            self.agent_predictions = None
            return

        # Initialize structured list of coordinates tracking length of horizon steps N
        self.agent_predictions = [[] for _ in range(self.N)]

        for agent in self.tracked_agents.agents:
            # Skip stationary agents
            if (
                abs(agent.velocity.linear.x) < 0.01
                and abs(agent.velocity.linear.y) < 0.01
            ):
                continue

            init_x = agent.pose.position.x
            init_y = agent.pose.position.y
            vel_x = agent.velocity.linear.x
            vel_y = agent.velocity.linear.y

            # Linear extrapolation matching our rollouts time increments
            for step in range(self.N):
                future_time = step * self.dt
                pred_x = init_x + vel_x * future_time
                pred_y = init_y + vel_y * future_time
                self.agent_predictions[step].append([pred_x, pred_y])
                
    ################### Part 2 ######################                     
    def compute_human_avoidance_cost(self, state_x, state_y, agent_predictions):
        """
        Vectorized cost computation for human avoidance
        """
        ## TODO: Part 2 -> Compute a vectorized human avoidance cost for the 'K' predictions at a time step 't'
        ## 1. Check whether there are any predicted agent positions to evaluate.
        ## 2. Convert agent predictions at current timeline step to array [Num_Agents, 2]. Get the x and y positions of all agents at this time step (shape = (1, Num_Agents)).
        ## 3. Get the robot's current rollout state positions (state_x, state_y) and reshape them for broadcasting (shape = (K, 1)). 
        #  4. Note: Shape requirements: [K, 1] and [1, Num_Agents] for broadcasting matrix calculation.
        ## 5. Compute the pairwise distance from each rollout state to each predicted human position. Resulting shape: (K, Num_Agents)
        ## 6. Get the effective distance by subtracting the human radius and robot radius from the pairwise distances.
        ## 7. Apply collision, proxemics, and safe-distance cost logic.
        ##      for collision: if effective distance < 0, assign a high penalty
        ##      for proxemics: if effective distance < proxemics distance, assign a penalty that increases as the distance decreases with high weight
        ##      for safe-distance: if effective distance < safe distance, assign a penalty that increases as the distance decreases with low weight
        ## 8. Sum the penalties across agents to produce one cost per rollout state.
        ## 9. Return the computed cost array of shape (K,).
        
         # If no agents are predicted or tracked, exit with zero cost instantly
        if agent_predictions is None or len(agent_predictions) == 0:
            return np.zeros_like(state_x)

        # Convert agent predictions at current timeline step to array [Num_Agents, 2]
        # Coordinates shape requirements: [K, 1] and [1, Num_Agents] for broadcasting matrix calculation
        agents_arr = np.array(agent_predictions)  # Assumed tracking active x,y frames
        hx = agents_arr[:, 0][np.newaxis, :]  # Shape: (1, Num_Agents)
        hy = agents_arr[:, 1][np.newaxis, :]  # Shape: (1, Num_Agents)

        sx = state_x[:, np.newaxis]  # Shape: (K, 1)
        sy = state_y[:, np.newaxis]  # Shape: (K, 1)

        # Broadmatrix Distance computation between all K samples and all tracked agents
        # Resulting shape: (K, Num_Agents)
        dist = np.sqrt((sx - hx) ** 2 + (sy - hy) ** 2) + 1e-6
        radius_sum = self.robot_radius + self.human_radius
        eff_dist = dist - radius_sum

        # Vectorized Condition Evaluation Masks
        cond_collision = eff_dist < 0
        cond_proxemic = (eff_dist >= 0) & (eff_dist < self.proxemic_dist)
        cond_safe = eff_dist >= self.proxemic_dist

        # Apply piece-wise functional equivalents matching your exact scaling mathematical intents
        choice_collision = 100.0 * np.abs((1./eff_dist) + self.proxemic_dist)
        choice_proxemic = 10.0 * np.abs(1./eff_dist)
        choice_safe = 1. / (eff_dist* 10.0)

        # Select matching equations per cell element
        pair_costs = np.select(
            [cond_collision, cond_proxemic, cond_safe],
            [choice_collision, choice_proxemic, choice_safe],
        )
        return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


class ROS2BridgeNode(Node):
    """
    Dedicated background ROS 2 node that listens to the simulator topics
    and pipes data directly into the algorithm node instance.
    """

    def __init__(self, algo_node_instance):
        super().__init__("mppi_ros2_bridge_node")
        self.algo_node = algo_node_instance

        # Subscribe to tracked agents.
        self.subscription = self.create_subscription(
            TrackedAgents, "/tracked_agents", self.listener_callback, 10
        )
        self.get_logger().info("Background ROS 2 listener node initialized.")

    def listener_callback(self, msg):
        # Pipes incoming ROS 2 message directly into the algorithm instance
        self.algo_node.setTrackedAgents(msg)


# Global Runtime Registration Hooks
socnav_controller_node = SocialMPPINode()

# Define the virtual functions for PYIF
def computeVelocityCommands(costmap, pose, twist):
    return socnav_controller_node.computeVelocityCommands(costmap, pose, twist)


def setPath(global_plan):
    socnav_controller_node.setPath(global_plan)


def setSpeedLimit(speed_limit, is_percentage):
    return

# Initialize rclpy once globally when this module loads
if not rclpy.ok():
    rclpy.init()

# Create the subscriber bridge and pass it the active MPPI algorithm core
bridge_node = ROS2BridgeNode(socnav_controller_node)

# Run rclpy.spin in a separate non-blocking thread so Nav2/the plugin loop can run freely
ros_thread = threading.Thread(target=lambda: rclpy.spin(bridge_node), daemon=True)
ros_thread.start()


