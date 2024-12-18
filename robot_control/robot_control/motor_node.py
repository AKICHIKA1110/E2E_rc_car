import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685

class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')

        # PCA9685設定
        i2c_bus = busio.I2C(SCL, SDA)
        self.pwm = PCA9685(i2c_bus)
        self.pwm.frequency = 60

        # GPIO設定
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self.IN1 = 23
        self.IN2 = 24
        self.IN3 = 27
        self.IN4 = 22
        GPIO.setup(self.IN1, GPIO.OUT)
        GPIO.setup(self.IN2, GPIO.OUT)
        GPIO.setup(self.IN3, GPIO.OUT)
        GPIO.setup(self.IN4, GPIO.OUT)

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Vr, Vlを出力するパブリッシャ
        self.wheel_speed_pub = self.create_publisher(Twist, '/wheel_speeds', 10)

    def set_speed(self, left_speed, right_speed):
        left_duty_cycle = int(min(abs(left_speed), 1.0) * 65535)
        right_duty_cycle = int(min(abs(right_speed), 1.0) * 65535)
        self.pwm.channels[0].duty_cycle = left_duty_cycle
        self.pwm.channels[1].duty_cycle = right_duty_cycle

        if left_speed > 0:
            GPIO.output(self.IN2, True)
            GPIO.output(self.IN1, False)
        elif left_speed < 0:
            GPIO.output(self.IN2, False)
            GPIO.output(self.IN1, True)
        else:
            GPIO.output(self.IN1, False)
            GPIO.output(self.IN2, False)

        if right_speed > 0:
            GPIO.output(self.IN4, True)
            GPIO.output(self.IN3, False)
        elif right_speed < 0:
            GPIO.output(self.IN4, False)
            GPIO.output(self.IN3, True)
        else:
            GPIO.output(self.IN3, False)
            GPIO.output(self.IN4, False)

    def cmd_vel_callback(self, msg):
        # msg.linear.xが前進速度v、msg.angular.zが旋回指令wと仮定
        v = msg.linear.x
        w = msg.angular.z
        k = 0.4
        Vr = v + k * w
        Vl = v - k * w

        # モータ速度設定
        self.set_speed(Vl, Vr)

        # Vr, VlをPublish
        wheel_msg = Twist()
        # linear.x = Vl, linear.y = Vrとして格納
        wheel_msg.linear.x = Vl
        wheel_msg.linear.y = Vr
        self.wheel_speed_pub.publish(wheel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    GPIO.cleanup()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
