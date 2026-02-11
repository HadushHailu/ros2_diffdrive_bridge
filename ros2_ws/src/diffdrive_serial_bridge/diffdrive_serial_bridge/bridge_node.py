#!/usr/bin/env python3
import time
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

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
        self.declare_parameter('imu_frame_id', 'imu_link')

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.track = self.get_parameter('track_width').get_parameter_value().double_value
        self.max_v = self.get_parameter('max_v').get_parameter_value().double_value
        self.max_w = self.get_parameter('max_w').get_parameter_value().double_value
        self.pwm_max = int(self.get_parameter('pwm_max').get_parameter_value().integer_value)
        self.send_hz = self.get_parameter('send_hz').get_parameter_value().double_value
        self.imu_frame_id = self.get_parameter('imu_frame_id').get_parameter_value().string_value

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
        self.rx_buf = bytearray()

        # Time sync: offset = ROS_time - Arduino_time (in seconds)
        self.time_offset = None
        self.last_arduino_time = None  # for detecting Arduino resets (micros rollover)

        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
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

        self.read_serial()

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

    def read_serial(self):
        try:
            waiting = self.ser.in_waiting
        except Exception:
            return

        if waiting <= 0:
            return

        try:
            data = self.ser.read(waiting)
        except Exception:
            return

        for b in data:
            if b in (10, 13):
                if not self.rx_buf:
                    continue
                line = self.rx_buf.decode('ascii', errors='ignore').strip()
                self.rx_buf.clear()
                self.handle_line(line)
            else:
                if len(self.rx_buf) < 256:
                    self.rx_buf.append(b)
                else:
                    self.rx_buf.clear()

    def handle_line(self, line: str):
        if not line.startswith('I,'):
            return

        parts = line.split(',')
        if len(parts) != 12:  # I,timestamp,ax,ay,az,gx,gy,gz,qw,qx,qy,qz
            return

        try:
            arduino_time_us = int(parts[1])
            ax = float(parts[2])
            ay = float(parts[3])
            az = float(parts[4])
            gx = float(parts[5])
            gy = float(parts[6])
            gz = float(parts[7])
            qw = float(parts[8])
            qx = float(parts[9])
            qy = float(parts[10])
            qz = float(parts[11])
        except ValueError:
            return

        # Convert Arduino micros to seconds
        arduino_time_sec = arduino_time_us / 1_000_000.0
        ros_now = self.get_clock().now()
        ros_now_sec = ros_now.nanoseconds / 1e9

        # Detect Arduino reset (micros rollover after ~71 minutes or actual reset)
        if self.last_arduino_time is not None and arduino_time_sec < self.last_arduino_time - 1.0:
            self.time_offset = None  # force re-sync
            self.get_logger().warn("Arduino time reset detected, re-syncing")

        self.last_arduino_time = arduino_time_sec

        # Establish or update time offset (ROS_time - Arduino_time)
        if self.time_offset is None:
            self.time_offset = ros_now_sec - arduino_time_sec
            self.get_logger().info(f"Time sync established: offset = {self.time_offset:.3f}s")

        # Compute synchronized timestamp
        synced_time_sec = arduino_time_sec + self.time_offset
        synced_sec = int(synced_time_sec)
        synced_nsec = int((synced_time_sec - synced_sec) * 1e9)

        msg = Imu()
        msg.header.stamp.sec = synced_sec
        msg.header.stamp.nanosec = synced_nsec
        msg.header.frame_id = self.imu_frame_id

        # IMU is mounted upside down - flip Y and Z axes (180° rotation around X)
        msg.orientation.w = qw
        msg.orientation.x = qx
        msg.orientation.y = -qy  # flip
        msg.orientation.z = -qz  # flip

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = -gy  # flip
        msg.angular_velocity.z = -gz  # flip

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = -ay  # flip
        msg.linear_acceleration.z = -az  # flip (gravity should now be +9.8 in Z)

        msg.orientation_covariance = [-1.0] * 9
        msg.angular_velocity_covariance = [-1.0] * 9
        msg.linear_acceleration_covariance = [-1.0] * 9

        self.imu_pub.publish(msg)


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

