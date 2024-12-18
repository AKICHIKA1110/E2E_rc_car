import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import tensorflow as tf
import numpy as np

class TensorFlowNode(Node):
    def __init__(self):
        super().__init__('tensorflow_node')
        self.bridge = CvBridge()

        # TensorFlowモデルを読み込み
        self.model_path = self.declare_parameter('model_path', '/path/to/your/model.h5').value
        self.model = tf.keras.models.load_model(self.model_path)
        self.get_logger().info(f"Loaded TensorFlow model from {self.model_path}")

        # サブスクライバーとパブリッシャーの設定
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def preprocess_image(self, image):
        """画像をモデル入力形式に前処理"""
        # RGBに変換し、モデルの入力サイズにリサイズ
        input_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        input_image = cv2.resize(input_image, (160, 120))  # モデルの入力サイズに合わせる
        input_image = input_image.astype(np.float32) / 255.0  # 正規化
        input_image = np.expand_dims(input_image, axis=0)  # バッチ次元を追加
        return input_image

    def image_callback(self, msg):
        """カメラ画像のコールバック"""
        try:
            # ROSメッセージをOpenCV画像に変換
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # 前処理してモデルに入力
            input_image = self.preprocess_image(cv_image)
            predictions = self.model.predict(input_image)

            # モデルの出力を解析 (例: [Vl, Vr]の形式)
            Vl, Vr = predictions[0]
            Vl = max(0.0, min(Vl, 1.0))  # 0.0 ~ 1.0 の範囲にクリップ
            Vr = max(0.0, min(Vr, 1.0))

            # 結果を/cmd_velトピックに送信
            twist_msg = Twist()
            twist_msg.linear.x = (Vl + Vr) / 2.0  # 前進速度
            twist_msg.angular.z = (Vr - Vl)  # 左右の速度差を旋回速度に変換
            self.publisher.publish(twist_msg)

            self.get_logger().info(f"Predicted speeds -> Vl: {Vl:.2f}, Vr: {Vr:.2f}")
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TensorFlowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
