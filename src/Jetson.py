import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import Jetson.GPIO as GPIO
import time

R = 15
L = 18
ENABLE_R = 22
ENABLE_L = 13

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber')
        self.joy_r = 0
        self.joy_l = 0

        self.get_logger().info("Setting up GPIO using JetsonGPIO...")

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(R, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(L, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(ENABLE_R, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(ENABLE_L, GPIO.OUT, initial=GPIO.LOW)

        self.get_logger().info("GPIO setup completed.")

        self.subscription = self.create_subscription(
            Int32MultiArray,
            'velocity',
            self.to_gpio,
            10)
        self.get_logger().info("Subscription created successfully.")

    def to_gpio(self, msg):
        self.get_logger().info("toGpio callback called.")

        if not msg:
            self.get_logger().error("Received null pointer in callback")
            return

        if len(msg.data) < 2:
            self.get_logger().error(f"Invalid joystick data size: {len(msg.data)}")
            return

        self.joy_r = msg.data[0]
        self.joy_l = msg.data[1]

        self.update_pwm(R, self.joy_r)
        self.update_pwm(L, self.joy_l)

        self.get_logger().info(f"Right Joystick: {self.joy_r}, Left Joystick: {self.joy_l}")

    def update_pwm(self, pin, duty_cycle):
        duty_cycle = max(0, min(duty_cycle, 100))

        if duty_cycle > 0:
            GPIO.output(pin, GPIO.HIGH)
            time.sleep(duty_cycle / 1000.0)

        if duty_cycle < 100:
            GPIO.output(pin, GPIO.LOW)
            time.sleep((100 - duty_cycle) / 1000.0)

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()
    rclpy.spin(node)
    GPIO.cleanup()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

