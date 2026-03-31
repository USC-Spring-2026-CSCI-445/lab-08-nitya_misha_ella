#!/usr/bin/env python3
from typing import Optional, Tuple, List, Dict
from argparse import ArgumentParser
from math import inf, sqrt, atan2, pi
from time import sleep, time
import queue
import json
import math
from random import uniform
import copy

import scipy
import numpy as np
import rospy
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Twist, Point32, PoseStamped, Pose, Vector3, Quaternion, Point, PoseArray
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan, PointCloud, ChannelFloat32
from visualization_msgs.msg import MarkerArray, Marker
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import scipy.stats
from numpy.random import choice

np.set_printoptions(linewidth=200)

# AABB format: (x_min, x_max, y_min, y_max)
OBS_TYPE = Tuple[float, float, float, float]
# Position format: {"x": x, "y": y, "theta": theta}
POSITION_TYPE = Dict[str, float]

# don't change this
GOAL_THRESHOLD = 0.1


def angle_to_0_to_2pi(angle: float) -> float:
    while angle < 0:
        angle += 2 * pi
    while angle > 2 * pi:
        angle -= 2 * pi
    return angle


def angle_to_neg_pi_to_pi(angle: float) -> float:
    while angle < -pi:
        angle += 2 * pi
    while angle > pi:
        angle -= 2 * pi
    return angle


# see https://rootllama.wordpress.com/2014/06/20/ray-line-segment-intersection-test-in-2d/
def ray_line_intersection(ray_origin, ray_direction_rad, point1, point2):
    # Convert to numpy arrays
    ray_origin = np.array(ray_origin, dtype=np.float32)
    ray_direction = np.array([math.cos(ray_direction_rad), math.sin(ray_direction_rad)])
    point1 = np.array(point1, dtype=np.float32)
    point2 = np.array(point2, dtype=np.float32)

    # Ray-Line Segment Intersection Test in 2D
    v1 = ray_origin - point1
    v2 = point2 - point1
    v3 = np.array([-ray_direction[1], ray_direction[0]])
    denominator = np.dot(v2, v3)
    if denominator == 0:
        return None
    t1 = np.cross(v2, v1) / denominator
    t2 = np.dot(v1, v3) / denominator
    if t1 >= 0.0 and 0.0 <= t2 <= 1.0:
        return [ray_origin + t1 * ray_direction]
    return None


class Map:
    def __init__(self, obstacles: List[OBS_TYPE], map_aabb: Tuple):
        self.obstacles = obstacles
        self.map_aabb = map_aabb

    @property
    def top_right(self) -> Tuple[float, float]:
        return self.map_aabb[1], self.map_aabb[3]

    @property
    def bottom_left(self) -> Tuple[float, float]:
        return self.map_aabb[0], self.map_aabb[2]

    def draw_distances(self, origins: List[Tuple[float, float]]):
        """Example usage:
        map_ = Map(obstacles, map_aabb)
        map_.draw_distances([(0.0, 0.0), (3, 3), (1.5, 1.5)])
        """

        # Draw scene
        import matplotlib.pyplot as plt
        import matplotlib.patches as patches

        fig, ax = plt.subplots(figsize=(10, 10))
        fig.tight_layout()
        x_min_global, x_max_global, y_min_global, y_max_global = self.map_aabb
        for aabb in self.obstacles:
            width = aabb[1] - aabb[0]
            height = aabb[3] - aabb[2]
            rect = patches.Rectangle(
                (aabb[0], aabb[2]), width, height, linewidth=2, edgecolor="r", facecolor="r", alpha=0.4
            )
            ax.add_patch(rect)
        ax.set_xlim(x_min_global, x_max_global)
        ax.set_ylim(y_min_global, y_max_global)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_title("2D Plot of Obstacles")
        ax.set_aspect("equal", "box")
        plt.grid(True)

        # Draw rays
        angles = np.linspace(0, 2 * math.pi, 10, endpoint=False)
        for origin in origins:
            for angle in angles:
                closest_distance = self.closest_distance(origin, angle)
                if closest_distance is not None:
                    x = origin[0] + closest_distance * math.cos(angle)
                    y = origin[1] + closest_distance * math.sin(angle)
                    plt.plot([origin[0], x], [origin[1], y], "b-")
        plt.show()

    def closest_distance(self, origin: Tuple[float, float], angle: float) -> Optional[float]:
        """Returns the closest distance to an obstacle from the given origin in the given direction `angle`. If no
        intersection is found, returns `None`.
        """

        def lines_from_obstacle(obstacle: OBS_TYPE):
            """Returns the four lines of the given AABB format obstacle.
            Example usage: `point0, point1 = lines_from_obstacle(self.obstacles[0])`
            """
            x_min, x_max, y_min, y_max = obstacle
            return [
                [(x_min, y_min), (x_max, y_min)],
                [(x_max, y_min), (x_max, y_max)],
                [(x_max, y_max), (x_min, y_max)],
                [(x_min, y_max), (x_min, y_min)],
            ]

        # Iterate over the obstacles in the map to find the closest distance (if there is one). Remember that the
        # obstacles are represented as a list of AABBs (Axis-Aligned Bounding Boxes) with the format
        # (x_min, x_max, y_min, y_max).
        result = None
        origin = np.array(origin)

        for obstacle in self.obstacles:
            for line in lines_from_obstacle(obstacle):
                p = ray_line_intersection(origin, angle, line[0], line[1])
                if p is None:
                    continue

                dist = np.linalg.norm(np.array(p) - origin)
                if result is None:
                    result = dist
                else:
                    result = min(result, dist)
        return result

# PID controller class
######### Your code starts here #########
class PIDController:
    """
    Generates control action taking into account instantaneous error (proportional action),
    accumulated error (integral action) and rate of change of error (derivative action).
    """

    def __init__(self, kP, kI, kD, kS, u_min, u_max):
        assert u_min < u_max, "u_min should be less than u_max"
        # initialize PID variables here
        ######### Your code starts here #########
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.kS = kS
        self.u_min = u_min
        self.u_max = u_max
        #more?
        self.t_prev = 0.0
        self.e_prev = 0.0
        self.integral = 0.0
        ######### Your code ends here #########

    def control(self, err, t):
        # compute PID control action here
        ######### Your code starts here #########
        dt = t - self.t_prev
        
        #compute derivative
        if self.t_prev == 0.0 or dt <= 0:
            derivative = 0.0
        else:
            derivative = (err - self.e_prev)/dt

        
        #compute integral
        if dt > 0:
            self.integral += err * dt

        #clamp integral
        self.integral = max(-self.kS, min(self.integral, self.kS))

        #compute u 
        u = self.kP*err + self.kI*self.integral + self.kD*derivative
        u = max(self.u_min, min(u, self.u_max))

        self.t_prev = t
        self.e_prev = err

        return u
        ######### Your code ends here #########

######### Your code ends here #########


class Particle:
    def __init__(self, x: float, y: float, theta: float, log_p: float):
        self.x = x
        self.y = y
        self.theta = theta
        self.log_p = log_p

    def __str__(self) -> str:
        return f"Particle<pose: {self.x, self.y, self.theta}, log_p: {self.log_p}>"


class ParticleFilter:

    def __init__(
        self,
        map_: Map,
        n_particles: int,
        translation_variance: float,
        rotation_variance: float,
        measurement_variance: float,
    ):
        self.particles_visualization_pub = rospy.Publisher("/pf_particles", PoseArray, queue_size=10)
        self.estimate_visualization_pub = rospy.Publisher("/pf_estimate", PoseStamped, queue_size=10)

        # Initialize uniformly-distributed particles
        ######### Your code starts here #########
        self.map_ = map_                                                                  
        self.n_particles = n_particles
        self.translation_variance = translation_variance                                  
        self.rotation_variance = rotation_variance                                        
        self.measurement_variance = measurement_variance

        x_min, x_max, y_min, y_max = map_.map_aabb                                        
        self._particles = [
            Particle(
                x=uniform(x_min, x_max),
                y=uniform(y_min, y_max),
                theta=uniform(0, 2 * pi),                                             
                log_p=-math.log(n_particles)
            )                                                                         
            for _ in range(n_particles)
        ]

        ######### Your code ends here #########

    def visualize_particles(self):
        pa = PoseArray()
        pa.header.frame_id = "odom"
        pa.header.stamp = rospy.Time.now()
        for particle in self._particles:
            pose = Pose()
            pose.position = Point(particle.x, particle.y, 0.01)
            q_np = quaternion_from_euler(0, 0, float(particle.theta))
            pose.orientation = Quaternion(*q_np.tolist())
            pa.poses.append(pose)
        self.particles_visualization_pub.publish(pa)

    def visualize_estimate(self):
        ps = PoseStamped()
        ps.header.frame_id = "odom"
        ps.header.stamp = rospy.Time.now()
        x, y, theta = self.get_estimate()
        pose = Pose()
        pose.position = Point(x, y, 0.01)
        q_np = quaternion_from_euler(0, 0, float(theta))
        pose.orientation = Quaternion(*q_np.tolist())
        ps.pose = pose
        self.estimate_visualization_pub.publish(ps)

    def move_by(self, delta_x, delta_y, delta_theta):
        delta_theta = angle_to_neg_pi_to_pi(delta_theta)

        # Propagate motion of each particle
        ######### Your code starts here #########
        d = math.sqrt(delta_x*delta_x + delta_y*delta_y)
        for p in self._particles:
            # Add noise to the commanded motion step
            d_prime = d + np.random.normal(0, self.translation_variance)
            theta_prime = delta_theta + np.random.normal(0, self.rotation_variance)

            # Move in the local frame of the particle
            p.x += (d_prime * math.cos(p.theta))
            p.y += (d_prime * math.sin(p.theta))
            p.theta = angle_to_neg_pi_to_pi(p.theta + theta_prime)
        ######### Your code ends here #########

    def measure(self, z: float, scan_angle_in_rad: float):
        """Update the particles based on the measurement `z` at the given `scan_angle_in_rad`.

        Args:
            z: distance to an obstacle
            scan_angle_in_rad: Angle in the robots frame where the scan was taken
        """

        # Calculate posterior probabilities and resample
                ######### Your code starts here #########

        # Step 1: Update each particle's log weight based on how well the expected
        # sensor reading matches the actual reading z.
        for p in self._particles:
            expected = self.map_.closest_distance((p.x, p.y), p.theta + scan_angle_in_rad)
            # If no wall found in that direction, particle is in a bad state — penalize it heavily
            if expected is None:
                p.log_p += math.log(1e-300)
            else:
                likelihood = scipy.stats.norm(expected, self.measurement_variance).pdf(z)
                # Avoid log(0) if likelihood rounds to zero
                p.log_p += math.log(max(likelihood, 1e-300))

        # Step 2: Normalize log weights for numerical stability, then convert to probabilities.
        # Subtracting the max log weight before exponentiating prevents underflow/overflow.
        log_ps = np.array([p.log_p for p in self._particles])
        log_ps -= np.max(log_ps)
        probs = np.exp(log_ps)
        probs /= np.sum(probs)

        # Step 3: Resample — draw N particles with replacement according to weights.
        indices = choice(self.n_particles, self.n_particles, p=probs) # "Give me N indices drawn from [0, 1, ..., N-1], weighted by probs"
        new_particles = []
        for i in indices:
            old = self._particles[i]
            # Reset the probabilities to uniform after resampling step
            new_particles.append(Particle(old.x, old.y, old.theta, -math.log(self.n_particles)))
        self._particles = new_particles

        ######### Your code ends here #########

    def get_estimate(self) -> Tuple[float, float, float]:
        # Estimate robot's location using particle weights
        ######### Your code starts here #########
        log_weights = np.array([p.log_p for p in self._particles])
        # use sox_est, y_est, theta_estights to normalized weights 
        log_weights -= np.max(log_weights)
        weights = np.exp(log_weights)
        weights /= np.sum(weights)

        xs = np.array([p.x for p in self._particles])
        ys = np.array([p.y for p in self._particles])
        thetas = np.array([p.theta for p in self._particles])

        x_est = np.dot(weights, xs)
        y_est = np.dot(weights, ys)
        # use circular mean for theta
        theta_est = math.atan2(np.dot(weights, np.sin(thetas)), np.dot(weights, np.cos(thetas)))

        return x_est, y_est, theta_est

        ######### Your code ends here #########


class Controller:
    def __init__(self, particle_filter: ParticleFilter):
        rospy.init_node("particle_filter_controller", anonymous=True)
        self._particle_filter = particle_filter
        self._particle_filter.visualize_particles()

        #
        self.current_position = None
        self.laserscan = None
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.laserscan_sub = rospy.Subscriber("/scan", LaserScan, self.robot_laserscan_callback)
        self.robot_ctrl_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.pointcloud_pub = rospy.Publisher("/scan_pointcloud", PointCloud, queue_size=10)
        self.target_position_pub = rospy.Publisher("/waypoints", MarkerArray, queue_size=10)

        while ((self.current_position is None) or (self.laserscan is None)) and (not rospy.is_shutdown()):
            rospy.loginfo("waiting for odom and laserscan")
            rospy.sleep(0.1)

    def odom_callback(self, msg):
        pose = msg.pose.pose
        orientation = pose.orientation
        _, _, theta = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.current_position = {"x": pose.position.x, "y": pose.position.y, "theta": theta}

    def robot_laserscan_callback(self, msg: LaserScan):
        self.laserscan = msg

    def visualize_laserscan_ranges(self, idx_groups: List[Tuple[int, int]]):
        """Helper function to visualize ranges of sensor readings from the laserscan lidar.

        Example usage for visualizing the first 10 and last 10 degrees of the laserscan:
            `self.visualize_laserscan_ranges([(0, 10), (350, 360)])`
        """
        pcd = PointCloud()
        pcd.header.frame_id = "odom"
        pcd.header.stamp = rospy.Time.now()
        for idx_low, idx_high in idx_groups:
            for idx, d in enumerate(self.laserscan.ranges[idx_low:idx_high]):
                if d == inf:
                    continue
                angle = math.radians(idx) + self.current_position["theta"]
                x = d * math.cos(angle) + self.current_position["x"]
                y = d * math.sin(angle) + self.current_position["y"]
                z = 0.1
                pcd.points.append(Point32(x=x, y=y, z=z))
                pcd.channels.append(ChannelFloat32(name="rgb", values=(0.0, 1.0, 0.0)))
        self.pointcloud_pub.publish(pcd)

    def visualize_position(self, x: float, y: float):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "waypoints"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position = Point(x, y, 0.0)
        marker.pose.orientation = Quaternion(0, 0, 0, 1)
        marker.scale = Vector3(0.075, 0.075, 0.1)
        marker.color = ColorRGBA(0.0, 0.0, 1.0, 0.5)
        marker_array.markers.append(marker)
        self.target_position_pub.publish(marker_array)

    def take_measurements(self):
        # Take measurement using LIDAR
        ######### Your code starts here #########
        # NOTE: with more than 2 angles the particle filter will converge too quickly, so with high likelihood the
        # correct neighborhood won't be found.
        if self.laserscan is None:
            return
            
        # Sample 2 angles (0 and 90 degrees) to prevent converging incorrectly
        angles_to_test = [0, 90]
        
        for angle_deg in angles_to_test:
            idx = int(angle_deg) % len(self.laserscan.ranges)
            z = self.laserscan.ranges[idx]
            
            # Avoid measuring infinities or NaNs from the lidar simulation
            if z == inf or math.isnan(z):
                continue
                
            scan_angle_in_rad = math.radians(angle_deg)
            self._particle_filter.measure(z, scan_angle_in_rad)
            
        self._particle_filter.visualize_estimate()
        self._particle_filter.visualize_particles()
        ######### Your code ends here #########

    def autonomous_exploration(self):
        """Randomly explore the environment here, while making sure to call `take_measurements()` and
        `_particle_filter.move_by()`. The particle filter should converge on the robots position eventually.

        Note that the following visualizations functions are available:
            visualize_position(...)
            visualize_laserscan_ranges(...)
        """
        # Robot autonomously explores environment while it localizes itself
        ######### Your code starts here #########
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            # 1. Determine confidence based on particle distribution spread
            xs = [p.x for p in self._particle_filter._particles]
            ys = [p.y for p in self._particle_filter._particles]
            
            # If standard deviation of particle grouping is very tight, we're confident
            if np.std(xs) < 0.15 and np.std(ys) < 0.15:
                rospy.loginfo("Confident in location. Stopping exploration.")
                break

            # 2. Basic obstacle avoidance logic
            front_ranges = self.laserscan.ranges[0:15] + self.laserscan.ranges[-15:]
            min_front_dist = min(front_ranges) if front_ranges else inf
            
            if min_front_dist < 0.5:
                # Rotate 90 degrees left to avoid wall
                goal_theta = angle_to_neg_pi_to_pi(self.current_position["theta"] + pi/2)
                self.rotate_action(goal_theta)
                self._particle_filter.move_by(0.0, 0.0, pi/2)
            else:
                # Move forward 0.3 meters if path is clear
                self.forward_action(0.3)
                self._particle_filter.move_by(0.3, 0.0, 0.0)
            
            # 3. Always take measurements after moving
            self.take_measurements()
            rate.sleep()
        ######### Your code ends here #########

    def forward_action(self, distance: float):
        # Robot moves forward by a set amount during manual control
        ######### Your code starts here #########
        
        start_x = self.current_position["x"]
        start_y = self.current_position["y"]
        
        while not rospy.is_shutdown():
            current_x = self.current_position["x"]
            current_y = self.current_position["y"]

            distance_traveled = sqrt((current_x - start_x)**2 + (current_y - start_y)**2)
            
            if distance_traveled >= distance:
                break
            
            cmd = Twist()
            cmd.linear.x = 0.2
            self.robot_ctrl_pub.publish(cmd)        
                
            rospy.sleep(0.05)

        # stop once you're done
        cmd = Twist()
        cmd.linear.x = 0
        self.robot_ctrl_pub.publish(cmd)
            
        ######### Your code ends here #########

    def rotate_action(self, goal_theta: float):
        # Robot turns by a set amount during manual control
        ######### Your code starts here #########
        
        while not rospy.is_shutdown():
            #get current angle and error from goal angle
            current_theta = self.current_position["theta"]
            error = angle_to_neg_pi_to_pi(goal_theta - current_theta)

            if abs(error) < 0.1:
                break

            cmd = Twist()
            if error > 0:
                #turn right
                cmd.angular.z = 0.8
            else:
                #turn left
                cmd.angular.z = -0.8

            self.robot_ctrl_pub.publish(cmd)
            rospy.sleep(0.05)

        # stop once you're done
        cmd = Twist()
        cmd.angular.z = 0
        self.robot_ctrl_pub.publish(cmd)
            
        ######### Your code ends here #########


""" Example usage

rosrun development lab8_9.py --map_filepath src/csci455l/scripts/lab8_9_map.json
"""


if __name__ == "__main__":

    parser = ArgumentParser()
    parser.add_argument("--map_filepath", type=str, required=True)
    args = parser.parse_args()
    with open(args.map_filepath, "r") as f:
        map_ = json.load(f)
        obstacles = map_["obstacles"]
        map_aabb = map_["map_aabb"]

    map_ = Map(obstacles, map_aabb)
    num_particles = 200
    translation_variance = 0.1
    rotation_variance = 0.05
    measurement_variance = 0.1
    particle_filter = ParticleFilter(map_, num_particles, translation_variance, rotation_variance, measurement_variance)
    controller = Controller(particle_filter)

    try:
        # Manual control
        goal_theta = 0
        controller.take_measurements()
        while not rospy.is_shutdown():
            print("\nEnter 'a', 'w', 's', 'd' to move the robot:")
            uinput = input("")
            if uinput == "w": # forward
                ######### Your code starts here #########
                controller.forward_action(0.5)
                particle_filter.move_by(0.5, 0.0, 0.0)
                ######### Your code ends here #########
            elif uinput == "a": # left
                ######### Your code starts here #########
                goal_theta = angle_to_neg_pi_to_pi(goal_theta + pi / 2)
                controller.rotate_action(goal_theta)
                particle_filter.move_by(0.0, 0.0, pi / 2)
                ######### Your code ends here #########
            elif uinput == "d": #right
                ######### Your code starts here #########
                goal_theta = angle_to_neg_pi_to_pi(goal_theta - pi / 2)
                controller.rotate_action(goal_theta)
                particle_filter.move_by(0.0, 0.0, -pi / 2)
                ######### Your code ends here #########
            elif uinput == "s": # backwards
                ######### Your code starts here #########
                goal_theta = angle_to_neg_pi_to_pi(goal_theta + pi)
                controller.rotate_action(goal_theta)
                particle_filter.move_by(0.0, 0.0, pi)
                ######### Your code ends here #########
            else:
                print("Invalid input")
            ######### Your code starts here #########
            controller.take_measurements()
            ######### Your code ends here #########

        # Autonomous exploration
        ######### Your code starts here #########
        controller.autonomous_exploration()
        ######### Your code ends here #########

    except rospy.ROSInterruptException:
        print("Shutting down...")