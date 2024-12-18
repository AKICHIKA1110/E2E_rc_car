import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame
import threading
import time

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.is_running = False
        self.thread = threading.Thread(target=self.joystick_loop)
        self.thread.start()

    def joystick_loop(self):
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() < 1:
            self.get_logger().error("Joystick not found")
            return
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        
        v = 1.0
        k = 0.4

        while rclpy.ok():
            # イベント取得
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    # ボタン割り当てはコントローラによる
                    if event.button == 3:  # e.g. stop
                        self.is_running = False
                    elif event.button == 0:  # start
                        self.is_running = True
                    elif event.button == 1:  # exit or shutdown
                        rclpy.shutdown()
                        return

            if self.is_running:
                w = -joystick.get_axis(0)
                w = round(w, 2)
                Vr = v + k * w
                Vl = v - k * w
                Vr = min(Vr, 1.0)
                Vl = min(Vl, 1.0)

                msg = Twist()
                # linear.xを両車輪の平均速度に、angular.zを左右差から計算した値として割り当てる例
                msg.linear.x = (Vl + Vr) / 2.0
                msg.angular.z = w
                self.publisher_.publish(msg)

            time.sleep(0.05) # 少し待つ

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
