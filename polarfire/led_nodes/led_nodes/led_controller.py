import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class LEDController:
        def __init__(self, node, led_id):
                self.node = node
                self.led_id = led_id
                self.led_path = f"/sys/class/leds/led{led_id}/brightness"

                self.mode = 0
                self.state = 0
                self.timer = None

        def write(self, value):
                try:
                        with open(self.led_path, "w") as f:
                                f.write(str(value))
                except Exception as e:
                        self.node.get_logger().error(f"LED {self.led_id} error: {e}")

        def set_mode(self, mode):
                self.mode = mode

                if self.timer is not None:
                        self.timer.cancel()
                        self.timer = None

                if mode == 0:
                        self.write(0)

                elif mode == 1:
                        self.write(1)

                elif mode == 2:
                        self.timer = self.node.create_timer(0.5, self._blink_callback)

                else:
                        self.node.get_logger().warn(f"Invalid mode {mode}")

        def _blink_callback(self):
                self.state = 1 - self.state
                self.write(self.state)

class LEDNode(Node):
        def __init__(self):
                super().__init__('led_controller_node')

                self.controllers = {
                        i: LEDController(self, i) for i in range(1, 9)
                }
                #subscriber
                self.subscription = self.create_subscription(
                        Int32MultiArray,
                        'led_command',
                        self.callback,
                        10
                )
                self.startup_rainbow()
                self.get_logger().info("LED Controller Node Ready")

        def callback(self, msg):
                if len(msg.data) < 2:
                        self.get_logger().warn("Invalid message")
                        return

                led_id = msg.data[0]
                mode = msg.data[1]

                if led_id not in self.controllers:
                        self.get_logger().warn(f"Invalid LED ID: {led_id}")
                        return

                self.controllers[led_id].set_mode(mode)

                self.get_logger().info(f"LED {led_id} --> Mode {mode}")

        def startup_rainbow(self, delay = 0.04, width = 3, cycles = 2):

                import time

                leds = list(self.controllers.keys())

                def set_all_off():
                        for i in leds:
                                self.controllers[i].write(0)
                for _ in range(cycles):
                        for i in range(len(leds) + width):
                                set_all_off()

                                for j in range(width):
                                        idx = i - j
                                        if 0 <= idx < len(leds):
                                                self.controllers[leds[idx]].write(1)
                                time.sleep(delay)

                        for i in range(len(leds) + width):
                                set_all_off()

                                for j in range(width):
                                        idx = (len(leds) - 1 - i) + j
                                        if 0 <= idx < len(leds):
                                                self.controllers[leds[idx]].write(1)

                                time.sleep(delay)

                set_all_off()


def main():
        rclpy.init()
        node = LEDNode()
        rclpy.spin(node)
        rclpy.shutdown()

if __name__ == '__main__':
        main()
