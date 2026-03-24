#!/usr/bin/env python3
from typing import Optional, Tuple, List, Dict
from argparse import ArgumentParser
from math import radians, inf, sqrt, atan2, pi, isinf, cos, sin, degrees
from time import sleep, time
import queue

import rospy
from turtlebot3_msgs.msg import SensorState
from geometry_msgs.msg import Twist, Point32, Vector3, Quaternion, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud, ChannelFloat32
from visualization_msgs.msg import MarkerArray, Marker
from tf.transformations import euler_from_quaternion
from std_msgs.msg import ColorRGBA


OBS_FREE_WAYPOINTS = [
    {"x": 1, "y": 1},
    {"x": 2, "y": 1},
    {"x": 1, "y": 0},
]

W_OBS_WAYPOINTS = [
    {"x": 3.0, "y": 0.0},
    # {"x": 4, "y": 1},
    # {"x": 0, "y": 3.0},
]


def angle_to_0_to_2pi(angle: float) -> float:
    while angle < 0:
        angle += 2 * pi
    while angle > 2 * pi:
        angle -= 2 * pi
    return angle


def map_to_new_range(x: float, a_low: float, a_high: float, b_low: float, b_high: float):
    """Helper function to map a value from range [a_low, a_high] to [b_low, b_high]"""
    y = (x - a_low) / (a_high - a_low) * (b_high - b_low) + b_low
    return y


class PIDController:
    """
    Generates control action taking into account instantaneous error (proportional action),
    accumulated error (integral action) and rate of change of error (derivative action).
    """

    def __init__(self, kP, kI, kD, kS, u_min, u_max):
        assert u_min < u_max, "u_min should be less than u_max"
        # Initializa PID variables here
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


class PDController:
    """
    Generates control action taking into account instantaneous error (proportional action)
    and rate of change of error (derivative action).
    """

    def __init__(self, kP, kD, kS, u_min, u_max):
        assert u_min < u_max, "u_min should be less than u_max"
        # Initialize PD variables here
        ######### Your code starts here #########
        self.kP = kP
        #self.kI = kI
        self.kD = kD
        self.kS = kS
        self.u_min = u_min
        self.u_max = u_max
        self.t_prev = 0.0
        self.e_prev = 0.0
        ######### Your code ends here #########

    def control(self, err, t):
        dt = t - self.t_prev
        # Compute PD control action here
        ######### Your code starts here #########
        if self.t_prev == 0.0 or dt <= 0:
            derivative = 0.0
        else:
            derivative = (err - self.e_prev)/dt

        
        u = self.kP * err + self.kD * derivative
        u = max(self.u_min, min(u, self.u_max))
        self.t_prev = t
        self.e_prev = err
        return u
        ######### Your code ends here #########


def publish_waypoints(waypoints: List[Dict], publisher: rospy.Publisher):
    marker_array = MarkerArray()
    for i, waypoint in enumerate(waypoints):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "waypoints"
        marker.id = i
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position = Point(waypoint["x"], waypoint["y"], 0.0)
        marker.pose.orientation = Quaternion(0, 0, 0, 1)
        marker.scale = Vector3(0.1, 0.1, 0.1)
        marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.5)
        marker_array.markers.append(marker)
    publisher.publish(marker_array)


# Class for controlling the robot to reach a goal position
class ObstacleFreeWaypointController:
    def __init__(self, waypoints: List[Dict]):
        rospy.init_node("waypoint_follower", anonymous=True)
        self.waypoints = waypoints
        # Subscriber to the robot's current position (assuming you have Odometry data)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.robot_ctrl_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.waypoint_pub = rospy.Publisher("/waypoints", MarkerArray, queue_size=10)
        sleep(0.5)  # sleep to give time for rviz to subscribe to /waypoints
        publish_waypoints(self.waypoints, self.waypoint_pub)

        self.current_position = None

        # define linear and angular PID controllers here
        ######### Your code starts here #########
        kP = 2.0
        kD = 0.03
        kI = 0.01
        kS = 0.4
        u_min = -1.5
        u_max = 1.5

        self.angular_controller = PIDController(kP, kI, kD, kS, u_min, u_max)
       
        self.v0 = 0.2 #base forward velocity
        ######### Your code ends here #########

    def odom_callback(self, msg):
        # Extracting current position from Odometry message
        pose = msg.pose.pose
        orientation = pose.orientation
        _, _, theta = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.current_position = {"x": pose.position.x, "y": pose.position.y, "theta": theta}

    def calculate_error(self, goal_position: Dict) -> Optional[Tuple]:
        """Return distance and angle error between the current position and the provided goal_position. Returns None if
        the current position is not available.
        """
        if self.current_position is None:
            return None

        # Calculate error in position and orientation
        ######### Your code starts here #########
        ######### CODE FROM LAB 5 BELOW #########

        ## update self.goal position to be from the parameter goal position
        dx = goal_position["x"] - self.current_position["x"]
        dy = goal_position["y"] - self.current_position["y"]
        
        #dist = sqrt(dx^2 + dy^2)
        distance_error = sqrt(dx**2+dy**2)
        
        #angle = arctan(dy/dx)
        goal_angle = atan2(dy, dx)
        current_angle = self.current_position["theta"]

        angle_error = goal_angle - current_angle

        # Ensure angle error is within -pi to pi range
        if angle_error > pi:
            angle_error -= 2 * pi
        elif angle_error < -pi:
            angle_error += 2 * pi

        ######### Your code ends here #########

        return distance_error, angle_error

    def control_robot(self):
        rate = rospy.Rate(20)  # 20 Hz
        ctrl_msg = Twist()

        current_waypoint_idx = 0

        while not rospy.is_shutdown():

            # Travel through waypoints one at a time, checking if robot is close enough
            ######### Your code starts here #########
            if current_waypoint_idx >= len(self.waypoints):
                # stop robot 
                ctrl_msg.linear.x = 0.0
                ctrl_msg.angular.z = 0.0
                self.robot_ctrl_pub.publish(ctrl_msg)
                rospy.loginfo("All waypoints reached!")
                break
            

            ## select the goal 
            goal = self.waypoints[current_waypoint_idx]
            error = self.calculate_error(goal)

            if error is None:
                rate.sleep()
                continue
            
            distance_error, angle_error = error
            
            #if error is small
            if abs(distance_error) < 0.15:
                rospy.loginfo(f"Reached waypoint {current_waypoint_idx}: {goal}")
                current_waypoint_idx += 1
                continue
            
            t = rospy.get_time()
            omega = self.angular_controller.control(angle_error, t) #calls the control method in the PID controller class that internally computes ω = kP*error + kI*integral + kD*derivative
            ctrl_msg.angular.z = omega
            ctrl_msg.linear.x = self.v0
            
            #publish
            self.robot_ctrl_pub.publish(ctrl_msg)

            ######### Your code ends here #########
            rate.sleep()


# Class for controlling the robot to reach a goal position
class ObstacleAvoidingWaypointController:
    def __init__(self, waypoints: List[Dict]):
        rospy.init_node("waypoint_follower", anonymous=True)
        self.waypoints = waypoints

        self.current_position = None
        self.laserscan: Optional[LaserScan] = None
        self.laserscan_angles: Optional[List[float]] = None
        self.ir_distance = None
        self.wall_following_desired_distance = 0.5  # set this to whatever you want

        # Subscriber to the robot's current position (assuming you have Odometry data)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.laserscan_sub = rospy.Subscriber("/scan", LaserScan, self.robot_laserscan_callback)
        self.cliff_sub = rospy.Subscriber("/sensor_state", SensorState, self.sensor_state_callback, queue_size=1)
        self.robot_ctrl_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.waypoint_pub = rospy.Publisher("/waypoints", MarkerArray, queue_size=10)
        self.pointcloud_pub = rospy.Publisher("/scan_pointcloud", PointCloud, queue_size=10)

        sleep(0.5)  # sleep to give time for rviz to subscribe to /waypoints
        publish_waypoints(self.waypoints, self.waypoint_pub)

        # Add PID controllers here for obstacle avoidance and waypoint following
        ######### Your code starts here #########

        kP = 2.0
        kD = 0.03
        kI = 0.01
        kS = 0.4
        u_min = -1.5
        u_max = 1.5

        self.wall_follow_controller = PDController(kP=1.0, kD=0.25, kS=kS, u_min=-2, u_max=2)
        self.goal_angular_controller = PIDController(kP=kP, kI=kI, kD=kD, kS=kS, u_min=u_min, u_max=u_max)

        self.v0 = 0.1 #base forward velocity

        ######### Your code ends here #########

    def sensor_state_callback(self, state: SensorState):
        raw = state.cliff
        # Calculation from raw sensor readings to distance (use equation from Lab 2)
        ######### Your code starts here #########
        distance = 3116.522296 * (raw ** -1.594097)

        ######### Your code ends here #########
        self.ir_distance = distance

    def robot_laserscan_callback(self, msg: LaserScan):
        self.laserscan = msg
        if self.laserscan_angles is None:
            self.laserscan_angles = [
                self.laserscan.angle_min + i * self.laserscan.angle_increment for i in range(len(self.laserscan.ranges))
            ]
            # sanity check the angles
            assert (abs(self.laserscan.angle_min) < 1e-4) and (
                abs(self.laserscan_angles[0]) < 1e-4
            ), f"laserscan.angle_min: {self.laserscan.angle_min}, laserscan_angles[0]: {self.laserscan_angles[0]}. Both should be 0"
            assert abs(self.laserscan.angle_max - 2 * pi) < 1e-4 and (abs(self.laserscan_angles[-1] - 2 * pi) < 1e-4)

    def odom_callback(self, msg):
        pose = msg.pose.pose
        orientation = pose.orientation
        _, _, theta = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.current_position = {"x": pose.position.x, "y": pose.position.y, "theta": theta}

    def waypoint_tracking_control(self, goal_position: Dict):

        if self.current_position is None:
            return None

        ######### Your code starts here #########

        dx = goal_position["x"] - self.current_position["x"]
        dy = goal_position["y"] - self.current_position["y"]

        distance_error = sqrt(dx**2 + dy**2)

        goal_angle = atan2(dy, dx)
        current_angle = self.current_position["theta"]

        angle_error = goal_angle - current_angle

        # Ensure angle error is within -pi to pi range
        if angle_error > pi:
            angle_error -= 2 * pi
        elif angle_error < -pi:
            angle_error += 2 * pi

        ctrl_msg = Twist()

        t = rospy.get_time()
        omega = self.goal_angular_controller.control(angle_error, t)

        cmd_linear_vel = self.v0
        cmd_angular_vel = omega
        ctrl_msg.angular.z = cmd_angular_vel
        ctrl_msg.linear.x = cmd_linear_vel

        self.robot_ctrl_pub.publish(ctrl_msg)

        ######### Your code ends here #########

        rospy.loginfo(
            f"distance to target: {distance_error:.2f}\tangle error: {angle_error:.2f}\tcommanded linear vel: {cmd_linear_vel:.2f}\tcommanded angular vel: {cmd_angular_vel:.2f}"
        )

        ## added this as well: 
        return distance_error

    def obstacle_avoiding_control(self, visualize: bool = True):

        ctrl_msg = Twist()

        ######### Your code starts here #########
        if self.laserscan_angles is not None:
            n = len(self.laserscan.ranges)
            rospy.loginfo_once(f"Total scan ranges: {n}, angle at index 90: {degrees(self.laserscan_angles[90]):.1f} deg")

        if self.laserscan is not None:
            raw_left = list(self.laserscan.ranges[80:100])
            print(f"ir_dist={self.ir_distance}  raw[80:100]={[round(x,2) for x in raw_left[:5]]}...")

        if self.ir_distance is None or self.ir_distance > 1.5:
            # wall not on left yet — move forward while turning right to find it
            ctrl_msg.angular.z = -0.3
            ctrl_msg.linear.x = self.v0
            self.robot_ctrl_pub.publish(ctrl_msg)
            print("Turning right to find wall on left side...")
            return

        err = self.wall_following_desired_distance - self.ir_distance
        t = rospy.get_time()
        u = self.wall_follow_controller.control(err, t)
        ctrl_msg.linear.x = self.v0
        ctrl_msg.angular.z = u  # don't negate because u is already a negative value !

        ######### Your code ends here #########

        self.robot_ctrl_pub.publish(ctrl_msg)
        print(
            f"dist: {round(self.ir_distance, 4)}\ttgt: {round(self.wall_following_desired_distance, 4)}\tu: {round(u, 4)}"
        )

    def laserscan_distances_to_point(self, point: Dict, cone_angle: float, visualize: bool = False):
        """Returns the laserscan distances within the cone of angle `cone_angle` centered about the line pointing from
        the robots current position to the given point. Angles are in radians.

        Notes:
            1. Distances that are outside of the laserscan's minimum and maximum range are filterered out
        """

        curr_pos = self.current_position
        # angle to point in the local frame. this is the same as the lidar frame. Not neccessarily in [-pi, pi] because
        # of the theta subtraction
        angle_to_point_local = angle_to_0_to_2pi(
            atan2(point["y"] - curr_pos["y"], point["x"] - curr_pos["x"]) - curr_pos["theta"]
        )
        angle_low = angle_to_0_to_2pi(angle_to_point_local - cone_angle)
        angle_high = angle_to_0_to_2pi(angle_to_point_local + cone_angle)

        # This is the so called 'danger zone', because either the high or low angle has wrapped around. For example,
        # when low = 355 deg, and high = 20 deg. The solution is to set the low to 0 and use the high when angle is > 0,
        # or set the high to 2*pi and use the low when angle is < 2*pi
        if angle_to_point_local < cone_angle or angle_to_point_local > 2 * pi - cone_angle:
            if angle_to_point_local < cone_angle:
                angle_low = 0
                idx_low = 0
                idx_high = int(
                    map_to_new_range(
                        angle_high, self.laserscan.angle_min, self.laserscan.angle_max, 0, len(self.laserscan.ranges)
                    )
                )
            elif angle_to_point_local > 2 * pi - cone_angle:
                angle_high = 2 * pi
                idx_high = len(self.laserscan.ranges) - 1
                idx_low = int(
                    map_to_new_range(
                        angle_low, self.laserscan.angle_min, self.laserscan.angle_max, 0, len(self.laserscan.ranges)
                    )
                )
            else:
                assert False, "should not reach here"
        else:
            idx_low = int(
                map_to_new_range(
                    angle_low, self.laserscan.angle_min, self.laserscan.angle_max, 0, len(self.laserscan.ranges)
                )
            )
            idx_high = int(
                map_to_new_range(
                    angle_high, self.laserscan.angle_min, self.laserscan.angle_max, 0, len(self.laserscan.ranges)
                )
            )
        assert angle_low < angle_high, f"angle_low: {angle_low}, angle_high: {angle_high}"
        if idx_low > idx_high:
            idx_low, idx_high = idx_high, idx_low
        assert idx_low < idx_high, f"idx_low: {idx_low}, idx_high: {idx_high}"

        raw = self.laserscan.ranges[idx_low:idx_high]
        filtered = [r for r in raw if (r > self.laserscan.range_min and r < self.laserscan.range_max)]

        if visualize:
            # raw should include all ranges, even if they are inf, in the specified cone
            #   i.e. something like `raw = self.laserscan.ranges[idx_low:idx_high]`
            # `angle_low` and `angle_high` are the angles in the robots local frame
            pcd = PointCloud()
            pcd.header.frame_id = "odom"
            pcd.header.stamp = rospy.Time.now()
            for i, p in enumerate(raw):
                if isinf(p):
                    continue
                angle_local = map_to_new_range(i, 0, len(raw), angle_low, angle_high)
                angle = angle_local + curr_pos["theta"]
                x = p * cos(angle) + curr_pos["x"]
                y = p * sin(angle) + curr_pos["y"]
                z = 0.1
                pcd.points.append(Point32(x=x, y=y, z=z))
                pcd.channels.append(ChannelFloat32(name="rgb", values=(0.0, 1.0, 0.0)))
            self.pointcloud_pub.publish(pcd)
        return filtered

    def control_robot(self):
        rate = rospy.Rate(10)  # 20 Hz

        current_waypoint_idx = 0
        distance_from_wall_safety = 1.0
        cone_angle = radians(5)
        in_obstacle_avoidance = False
        obstacle_clear_count = 0

        while not rospy.is_shutdown():

            if self.current_position is None or self.laserscan is None:
                sleep(0.01)
                continue

            # Travel through waypoints, checking if there is an obstacle in the way. Transition to obstacle avoidance if necessary
            ######### Your code starts here #########
            if current_waypoint_idx >= len(self.waypoints):
                # stop robot
                ctrl_msg = Twist()
                ctrl_msg.linear.x = 0.0
                ctrl_msg.angular.z = 0.0
                self.robot_ctrl_pub.publish(ctrl_msg)
                rospy.loginfo("All waypoints reached!")
                break


            ## select the goal
            goal = self.waypoints[current_waypoint_idx]

            distances = self.laserscan_distances_to_point(goal, cone_angle)
            obstacle_detected = len(distances) > 0 and min(distances) < distance_from_wall_safety

            # enter obstacle avoidance mode
            if obstacle_detected:
                in_obstacle_avoidance = True
                obstacle_clear_count = 0
                rospy.loginfo("Obstacle detected! Switching to wall following.")
                self.obstacle_avoiding_control()
            elif in_obstacle_avoidance:
                obstacle_clear_count += 1
                if obstacle_clear_count >= 20:
                    in_obstacle_avoidance = False
                    rospy.loginfo("Obstacle cleared! Resuming waypoint tracking.")
                else:
                    self.obstacle_avoiding_control()
            else:
                result = self.waypoint_tracking_control(goal)

                if result is None:
                    rate.sleep()
                    continue

                if result < 0.15:
                    rospy.loginfo(f"Reached waypoint {current_waypoint_idx}: {goal}")
                    current_waypoint_idx += 1

            ######### Your code ends here #########
            rate.sleep()


""" Example usage

rosrun development lab6_7.py --mode obstacle_free
rosrun development lab6_7.py --mode obstacle_avoiding
"""


if __name__ == "__main__":

    parser = ArgumentParser()
    parser.add_argument(
        "--mode", type=str, required=True, help="Mode of operation: 'obstacle_free' or 'obstacle_avoiding'"
    )
    args = parser.parse_args()
    assert args.mode in {"obstacle_free", "obstacle_avoiding"}

    if args.mode == "obstacle_free":
        controller = ObstacleFreeWaypointController(OBS_FREE_WAYPOINTS)
    else:
        controller = ObstacleAvoidingWaypointController(W_OBS_WAYPOINTS)

    try:
        controller.control_robot()
    except rospy.ROSInterruptException:
        print("Shutting down...")
