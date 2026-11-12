#!/usr/bin/env python3
import math
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, Spin
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Int32


class MissionCommander(Node):
    def __init__(self):
        super().__init__('aruco_mission_commander')
        self.waypoints = [
            (-4.5, 3.2), (-4.7, -2.3), (1.8, -3.5),
            (5.4, -5.2), (12.6, -5.2), (16.5, 0.1),
            (10.7, -1.1), (10.6, 3.7), (3.7, 4.8),
            (5.0, -2.1), (1.2, -0.2), (0.0, 0.0)
        ]
        self.index = 0
        self.mode = 'nav'
        self.state = 'idle'
        self.found = set()
        self.goal_handle = None
        self.pending = None
        self.motion_finished = False
        self.still_since = None

        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin = ActionClient(self, Spin, 'spin')
        self.nav.wait_for_server()
        self.spin.wait_for_server()
        self.capture = self.create_publisher(Int32, '/aruco_capture', 10)
        self.create_subscription(Int32, '/aruco_candidate', self.on_candidate, 10)
        self.create_subscription(Int32, '/aruco_found', self.on_found, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.start_timer = self.create_timer(1.0, self.begin)

    def begin(self):
        self.start_timer.cancel()
        self.start_motion()

    def start_motion(self):
        self.state = 'moving'
        self.goal_handle = None
        if self.mode == 'nav':
            x, y = self.waypoints[self.index]
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            goal = NavigateToPose.Goal(pose=pose)
            client = self.nav
            self.get_logger().info(f'Cel {self.index + 1}: ({x}, {y})')
        else:
            goal = Spin.Goal(target_yaw=2.0 * math.pi)
            goal.time_allowance.sec = 120
            client = self.spin
        client.send_goal_async(goal).add_done_callback(self.on_goal)

    def on_goal(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.advance(False)
            return
        self.goal_handle.get_result_async().add_done_callback(self.on_motion_end)

    def on_candidate(self, msg):
        if (self.state != 'moving' or self.goal_handle is None
                or msg.data in self.found or len(self.found) == 5):
            return
        self.pending = msg.data
        self.state = 'stopping'
        self.goal_handle.cancel_goal_async()

    def on_motion_end(self, future):
        self.goal_handle = None
        self.motion_finished = future.result().status == GoalStatus.STATUS_SUCCEEDED
        if self.state == 'stopping':
            self.state = 'settling'
            self.still_since = None
        else:
            self.advance(self.motion_finished)

    def on_odom(self, msg):
        if self.state != 'settling':
            return
        v = msg.twist.twist
        now = self.get_clock().now()
        if math.hypot(v.linear.x, v.linear.y) > 0.01 or abs(v.angular.z) > 0.02:
            self.still_since = None
        elif self.still_since is None:
            self.still_since = now
        elif (now - self.still_since).nanoseconds >= 500_000_000:
            self.state = 'capturing'
            self.capture.publish(Int32(data=self.pending))

    def on_found(self, msg):
        if self.state != 'capturing':
            return

        if msg.data == self.pending:
            self.found.add(msg.data)

        self.pending = None

        if len(self.found) == 5 or self.motion_finished:
            self.advance()
        else:
            self.start_motion()

    def advance(self, success=True):
        if len(self.found) == 5:
            self.state = 'finished'
            self.get_logger().info('Koniec misji: 5/5 kostek.')
            return
        if self.mode == 'nav' and success:
            self.mode = 'spin'
        else:
            self.mode = 'nav'
            self.index = (self.index + 1) % len(self.waypoints)
        self.start_motion()


def main():
    rclpy.init()
    node = MissionCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
