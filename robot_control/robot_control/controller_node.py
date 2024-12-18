import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame
import threading
import time
import subprocess

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.k = self.declare_parameter('k', 0.4).value
        self.v = self.declare_parameter('v', 1.0).value
        self.bag_dir = self.declare_parameter('bag_dir', os.path.join(os.path.expanduser("~"), "Desktop")).value
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10) #Buffer 10
        self.is_running = False
        self.bag_process = None  # rosbagプロセスを管理する変数
        self.thread = threading.Thread(target=self.joystick_loop)
        self.thread.start()

        # rosbagを開始
        self.start_rosbag()

    def start_rosbag(self):
        # rosbag記録プロセスを開始
        bag_path = os.path.join(self.bag_dir, "robot_data_bag")
        
        self.bag_process = subprocess.Popen(
            ['ros2', 'bag', 'record', '-o', 'bag_path', '/cmd_vel', '/camera/image_raw', '/wheel_speeds'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        self.get_logger().info("rosbag recording started")

    def stop_rosbag(self):
        # rosbag記録プロセスを終了
        if self.bag_process:
            self.bag_process.terminate()
            self.bag_process.wait()
            self.get_logger().info("rosbag recording stopped")

    def joystick_loop(self):
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() < 1:
            self.get_logger().error("Joystick not found")
            return
        joystick = pygame.joystick.Joystick(0)
        joystick.init()

        while rclpy.ok():
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == 3:  # □ボタンで停止
                        self.is_running = False
                    elif event.button == 0:  # ×ボタンでrosbag停止 & ノード終了
                        self.get_logger().info("×ボタンが押されました。プログラムを終了します。")
                        self.stop_rosbag()
                        rclpy.shutdown()
                        return
                    elif event.button == 1:  # ○ボタンで開始
                        self.is_running = True

            if self.is_running:
                w = -joystick.get_axis(0)
                w = round(w, 2)
                Vr = self.v + self.k * w
                Vl = self.v - self.k * w
                Vr = min(Vr, 1.0)
                Vl = min(Vl, 1.0)

                msg = Twist()
                msg.linear.x = (Vl + Vr) / 2.0
                msg.angular.z = w
                self.publisher_.publish(msg)

            time.sleep(0.05)  # 少し待つ

    def destroy_node(self):
        self.stop_rosbag()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
