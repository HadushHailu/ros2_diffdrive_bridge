#!/usr/bin/env python3
import time
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

class DiffDriveSerialBridge(Node):
    def __init__(self):
        super().__init__('diffdrive_serial_bridge')

        # ---- Parameters ----
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('track_width', 0.09)   # meters (your 9 cm)
        self.declare_parameter('max_v', 0.4)          # m/s (tune)
        self.declare_parameter('max_w', 2.0)          # rad/s (tune)
        self.declare_parameter('pwm_max', 200)        # <=255 (tune)
        self.declare_parameter('send_hz', 30.0)       # command send rate

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.track = self.get_parameter('track_width').get_parameter_value().double_value
        self.max_v = self.get_parameter('max_v').get_parameter_value().double_value
        self.max_w = self.get_parameter('max_w').get_parameter_value().double_value
        self.pwm_max = int(self.get_parameter('pwm_max').get_parameter_value().integer_value)
        self.send_hz = self.get_parameter('send_hz').get_parameter_value().double_value

        self.get_logger().info(f"Opening serial {self.port} @ {self.baud} ...")
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.02)
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise

        time.sleep(1.0)  # let Arduino reset
        try:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        except Exception:
            pass

        self.target_L = 0
        self.target_R = 0
        self.last_cmd_time = time.time()

        self.last_sent = None
        self.last_sent_time = time.time()
        self.heartbeat_period = 0.1  # seconds


        self.sub = self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.timer = self.create_timer(1.0 / self.send_hz, self.send_loop)

        self.get_logger().info("Bridge ready. Listening to /cmd_vel and sending M,L,R to Arduino.")

    def on_cmd(self, msg: Twist):
        v = float(msg.linear.x)
        w = float(msg.angular.z)

        # Clamp to expected ranges
        v = clamp(v, -self.max_v, self.max_v)
        w = clamp(w, -self.max_w, self.max_w)

        # Differential-drive wheel linear velocities (m/s)
        v_l = v - (w * self.track / 2.0)
        v_r = v + (w * self.track / 2.0)

        # Map wheel velocities to PWM (simple linear map for encoderless)
        # scale so max(|v|) -> pwm_max
        # Use absolute max_v for mapping
        scale = self.pwm_max / max(1e-6, self.max_v)
        pwm_l = int(clamp(v_l * scale, -255, 255))
        pwm_r = int(clamp(v_r * scale, -255, 255))

        self.target_L = pwm_l
        self.target_R = pwm_r
        self.last_cmd_time = time.time()

    def send_loop(self):
        now = time.time()

        # Stop if no cmd_vel recently
        if now - self.last_cmd_time > 0.3:
            target = (0, 0)
        else:
            target = (self.target_L, self.target_R)

        # Only send if changed OR heartbeat elapsed
        if target != self.last_sent or (now - self.last_sent_time) > self.heartbeat_period:
            line = f"M,{target[0]},{target[1]}\n"
            try:
                self.ser.write(line.encode('ascii'))
                self.last_sent = target
                self.last_sent_time = now
            except Exception:
                pass


def main():
    rclpy.init()
    node = None
    try:
        node = DiffDriveSerialBridge()
        rclpy.spin(node)
    except Exception as e:
        print(f"Exception: {e}")
    finally:
        if node is not None:
            try:
                node.ser.write(b"M,0,0\n")
                node.ser.close()
            except Exception:
                pass
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

