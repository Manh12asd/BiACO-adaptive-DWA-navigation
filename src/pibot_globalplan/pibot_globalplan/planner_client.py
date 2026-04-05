#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Point
from pibot_msg.action import AcoPlan 

from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException

class ACOActionClient(Node):
    def __init__(self):
        super().__init__('aco_action_client')

        # 1. Action Client & Subscriber
        self._action_client = ActionClient(self, AcoPlan, 'aco_plan')
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            10
        )

        # 2. Publishers cho RViz
        self.marker_pub = self.create_publisher(MarkerArray, '/planning_markers', 10)
        self.trajectory_pub = self.create_publisher(Marker, '/actual_trajectory', 10)

        # 3. TF Buffer & Listener để lấy vị trí
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 4. Các biến lưu trữ quỹ đạo thực tế
        self.actual_path = []
        self.min_record_dist = 0.05  # Chỉ ghi nhận điểm mới nếu robot di chuyển > 5cm
        
        # 5. Timer cập nhật quỹ đạo (chu kỳ 0.2s)
        self.traj_timer = self.create_timer(0.2, self.record_and_publish_trajectory)

        self.get_logger().info("ACO Action Client đã sẵn sàng. Hãy chọn Goal trên RViz...")

    def get_current_pose(self):
        """Lấy vị trí hiện tại của robot từ TF"""
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except TransformException:
            return None, None

    def record_and_publish_trajectory(self):
        """Hàm Timer callback để ghi lại và hiển thị quỹ đạo thực tế"""
        x, y = self.get_current_pose()
        if x is None or y is None:
            return

        # Chỉ lưu điểm mới nếu robot di chuyển đủ xa so với điểm lưu cuối cùng
        if len(self.actual_path) > 0:
            last_p = self.actual_path[-1]
            dist = math.hypot(x - last_p.x, y - last_p.y)
            if dist < self.min_record_dist:
                return

        # Thêm tọa độ mới vào danh sách
        p = Point()
        p.x = x
        p.y = y
        p.z = 0.05 # Nâng lên cao một chút để không bị che bởi bản đồ
        self.actual_path.append(p)

        # Tạo Marker dạng LINE_STRIP (Đường kẻ liền)
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "robot_trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        
        # Độ rộng của nét vẽ
        marker.scale.x = 0.03 
        
        # Màu sắc (màu Xanh Dương)
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.points = self.actual_path

        # Xuất Marker lên RViz
        self.trajectory_pub.publish(marker)

    def publish_markers(self, start_x, start_y, goal_x, goal_y, frame_id):
        marker_array = MarkerArray()

        # Marker START (Xanh Lá)
        start_marker = Marker()
        start_marker.header.frame_id = frame_id
        start_marker.header.stamp = self.get_clock().now().to_msg()
        start_marker.ns = "start_goal"
        start_marker.id = 0
        start_marker.type = Marker.CYLINDER
        start_marker.action = Marker.ADD
        start_marker.pose.position.x = start_x
        start_marker.pose.position.y = start_y
        start_marker.pose.position.z = 0.05
        start_marker.pose.orientation.w = 1.0
        start_marker.scale.x = 0.3
        start_marker.scale.y = 0.3
        start_marker.scale.z = 0.02
        start_marker.color.r = 0.0; start_marker.color.g = 1.0; start_marker.color.b = 0.0; start_marker.color.a = 0.8
        marker_array.markers.append(start_marker)

        # Marker GOAL (Đỏ)
        goal_marker = Marker()
        goal_marker.header.frame_id = frame_id
        goal_marker.header.stamp = self.get_clock().now().to_msg()
        goal_marker.ns = "start_goal"
        goal_marker.id = 1
        goal_marker.type = Marker.CYLINDER
        goal_marker.action = Marker.ADD
        goal_marker.pose.position.x = goal_x
        goal_marker.pose.position.y = goal_y
        goal_marker.pose.position.z = 0.05
        goal_marker.pose.orientation.w = 1.0
        goal_marker.scale.x = 0.3
        goal_marker.scale.y = 0.3
        goal_marker.scale.z = 0.02
        goal_marker.color.r = 1.0; goal_marker.color.g = 0.0; goal_marker.color.b = 0.0; goal_marker.color.a = 0.8
        marker_array.markers.append(goal_marker)

        self.marker_pub.publish(marker_array)

    def goal_pose_callback(self, msg):
        self.get_logger().info(f"Đã nhận mục tiêu từ topic: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}")
        
        # Xóa quỹ đạo cũ để bắt đầu vẽ quỹ đạo mới cho lần chạy này
        self.actual_path.clear()

        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        frame_id = msg.header.frame_id if msg.header.frame_id else 'map'
        
        start_x, start_y = self.get_current_pose()
        
        if start_x is not None and start_y is not None:
            self.publish_markers(start_x, start_y, goal_x, goal_y, frame_id)
            # Thêm ngay điểm xuất phát vào quỹ đạo
            p = Point()
            p.x = start_x
            p.y = start_y
            p.z = 0.05
            self.actual_path.append(p)

        self.send_goal(msg)

    def send_goal(self, pose_msg):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action Server không phản hồi!")
            return

        goal_msg = AcoPlan.Goal()
        goal_msg.goal_pose = pose_msg
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Mục tiêu bị Server từ chối.")
            return
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        pass 

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == 4: 
            self.get_logger().info(f"THÀNH CÔNG! Đã tìm thấy đường đi dài {result.total_length:.2f}m")

def main(args=None):
    rclpy.init(args=args)
    action_client = ACOActionClient()
    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        pass
    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()