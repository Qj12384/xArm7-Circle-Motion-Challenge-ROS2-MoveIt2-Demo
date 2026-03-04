#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np

class CircleMotionNode(Node):
    def __init__(self):
        super().__init__('circle_motion_node')
        
        #Match the 'circle' state in your start.launch.py SRDF exactly
        self.target_pose = [0.0, -0.4, 0.0, 0.7, 0.0, 1.1, 0.0]
        self.joint_names = [f'joint{i}' for i in range(1, 8)]
        self.is_triggered = False
        
        self.sub = self.create_subscription(JointState, '/joint_states', self.check_pose, 10)
        
        # Action client guarantees the move will be processed by fake execution in RViz
        self.action_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        
        self.get_logger().info("Circle node initialized. Select 'circle' Goal State in RViz to begin.")

    def check_pose(self, msg):
        if self.is_triggered: return

        try:
            current_pos = [msg.position[msg.name.index(name)] for name in self.joint_names]
            # Verify if joints are at the 'circle' pose (Tolerance 0.05 rad)
            if np.all(np.abs(np.array(current_pos) - np.array(self.target_pose)) < 0.05):
                self.get_logger().info("Target pose detected! Executing circle in 2 seconds...")
                self.is_triggered = True
                self.create_timer(2.0, self.publish_circle)
        except (ValueError, IndexError):
            pass

    def publish_circle(self):
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("MoveIt Action server not available.")
            return
            
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        
        points = 100
        duration = 10.0
        
        for i in range(points + 1):
            t = (i / points) * 2 * np.pi
            p = JointTrajectoryPoint()
            # Coordinate joints to extend the arm and draw a large circle with Joint 7
            p.positions = [
                0.0, 
                -0.4, 
                0.0, 
                0.7 + 0.3 * np.cos(t), 
                0.4 * np.sin(t), 
                1.1 + 0.3 * np.sin(t), 
                t # Joint 7 rotating 360 degrees
            ]
            p.time_from_start = Duration(sec=int((i/points)*duration), 
                                         nanosec=int(((i/points)*duration % 1)*1e9))
            traj.points.append(p)
            
        robot_traj = RobotTrajectory()
        robot_traj.joint_trajectory = traj
        
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = robot_traj
        
        self.action_client.send_goal_async(goal_msg)
        self.get_logger().info("Circle trajectory command dispatched successfully.")

def main():
    rclpy.init()
    node = CircleMotionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
