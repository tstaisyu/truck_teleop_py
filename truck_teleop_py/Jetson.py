import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import Jetson.GPIO as GPIO
import time

R = 15
L = 18
ENABLE_R = 22
ENABLE_L = 13

GPIO.setmode(GPIO.BOARD)
GPIO.setup(R, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(L, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ENABLE_R, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ENABLE_L, GPIO.OUT, initial=GPIO.LOW)
PWM_R = GPIO.PWM(R, 50)
PWM_L = GPIO.PWM(L, 50)
v_R = 0
v_L = 0
PWM_R.start(v_R)
PWM_L.start(v_L)

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber')
        self.joy_r = 0
        self.joy_l = 0

        print("Starting SubscriberNode initialization...")

        self.get_logger().info("Setting up GPIO using JetsonGPIO...")



        self.get_logger().info("GPIO setup completed.")

        self.subscription = self.create_subscription(
            Int32MultiArray,
            'velocity',
            self.to_gpio,
            10)
        self.get_logger().info("Subscription created successfully.")

        print("SubscriberNode initialization completed.")

    def to_gpio(self, msg):
        print("Received message in to_gpio callback.")
        self.get_logger().info("toGpio callback called.")

        if not msg:
            self.get_logger().error("Received null pointer in callback")
            return

        if len(msg.data) < 2:
            self.get_logger().error(f"Invalid joystick data size: {len(msg.data)}")
            return

        self.joy_r = msg.data[0]
        self.joy_l = msg.data[1]

        time.sleep(0.1)
        v_R = self.joy_r
        v_L = self.joy_l
        PWM_R.ChangeDutyCycle(v_R)
        PWM_L.ChangeDutyCycle(v_L)
        self.get_logger().info(f"Right Joystick: {self.joy_r}, Left Joystick: {self.joy_l}")

def main():
    print("Starting ROS2 Node.")
    rclpy.init()
    node = SubscriberNode()
    rclpy.spin(node)
    GPIO.cleanup()
    rclpy.shutdown()
    print("ROS2 Node has been shutdown.")

if __name__ == '__main__':
    main()
