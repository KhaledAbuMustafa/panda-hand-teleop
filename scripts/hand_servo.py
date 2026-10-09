#!/usr/bin/env python3
import socket
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import Empty
from moveit_msgs.srv import ServoCommandType
from control_msgs.action import GripperCommand
from tf2_ros import Buffer, TransformListener

BASE = "panda_link0"
EE = "panda_hand"
TUBES = ["tube_red", "tube_green", "tube_blue"]
SCALE_X = 0.3
SCALE_Y = 1.2
SCALE_Z = 1.5
LIMIT_X = 0.15
LIMIT_Y = 0.40
LIMIT_Z_DOWN = 0.35
LIMIT_Z_UP = 0.15
GAIN = 3.0
MAX_SPEED = 0.3
ALPHA = 0.3
CLOSE_BELOW = 0.35
OPEN_ABOVE = 0.60
GRASP_XY = 0.04
GRASP_Z = 0.08


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class HandServo(Node):
    def __init__(self):
        super().__init__("hand_servo")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 5005))
        self.sock.setblocking(False)
        self.pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.client = self.create_client(ServoCommandType, "/servo_node/switch_command_type")
        self.client.wait_for_service()
        self.gripper = ActionClient(self, GripperCommand, "/panda_hand_controller/gripper_cmd")
        self.gripper_closed = None
        self.tube_pos = {}
        self.attach_pub = {}
        self.detach_pub = {}
        for t in TUBES:
            self.create_subscription(PoseStamped, f"/model/{t}/pose",
                                     lambda msg, t=t: self.tube_cb(t, msg), 10)
            self.attach_pub[t] = self.create_publisher(Empty, f"/grasp/{t}/attach", 10)
            self.detach_pub[t] = self.create_publisher(Empty, f"/grasp/{t}/detach", 10)
        self.held = None
        self.detach_count = 20
        self.mode_set = False
        self.future = None
        self.hand = None
        self.last_msg_time = 0.0
        self.ref_hand = None
        self.ref_ee = None
        self.timer = self.create_timer(0.02, self.loop)
        self.get_logger().info("Bereit, Hand vor die Kamera halten")

    def tube_cb(self, tube, msg):
        p = msg.pose.position
        self.tube_pos[tube] = (p.x, p.y, p.z)

    def find_tube(self):
        try:
            left = self.tf_buffer.lookup_transform(BASE, "panda_leftfinger", rclpy.time.Time()).transform.translation
            right = self.tf_buffer.lookup_transform(BASE, "panda_rightfinger", rclpy.time.Time()).transform.translation
        except Exception:
            return None
        gx = (left.x + right.x) / 2
        gy = (left.y + right.y) / 2
        gz = (left.z + right.z) / 2 - 0.03
        best, best_d = None, GRASP_XY
        for t, (x, y, z) in self.tube_pos.items():
            d_xy = ((x - gx) ** 2 + (y - gy) ** 2) ** 0.5
            if d_xy < best_d and abs(z - gz) < GRASP_Z:
                best, best_d = t, d_xy
        return best

    def switch_mode(self):
        if self.future is None:
            req = ServoCommandType.Request()
            req.command_type = ServoCommandType.Request.TWIST
            self.future = self.client.call_async(req)
        elif self.future.done():
            result = self.future.result()
            if result is not None and result.success:
                self.mode_set = True
                self.get_logger().info("Servo-Modus TWIST aktiv")
            self.future = None

    def set_gripper(self, closed):
        if not self.gripper.server_is_ready():
            return
        goal = GripperCommand.Goal()
        goal.command.position = 0.0 if closed else 0.04
        goal.command.max_effort = 10.0
        self.gripper.send_goal_async(goal)
        self.gripper_closed = closed
        self.get_logger().info("Klaue zu" if closed else "Klaue auf")
        if closed:
            tube = self.find_tube()
            if tube:
                self.attach_pub[tube].publish(Empty())
                self.held = tube
                self.get_logger().info(f"{tube} gegriffen")
        elif self.held:
            self.detach_pub[self.held].publish(Empty())
            self.get_logger().info(f"{self.held} abgestellt")
            self.held = None

    def read_udp(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(1024)
            except BlockingIOError:
                break
            msg = json.loads(data.decode())
            if self.hand is None:
                self.hand = msg
            else:
                for k in msg:
                    self.hand[k] = ALPHA * msg[k] + (1 - ALPHA) * self.hand[k]
            self.last_msg_time = time.time()

    def loop(self):
        if not self.mode_set:
            self.switch_mode()
            return

        if self.detach_count > 0:
            for t in TUBES:
                self.detach_pub[t].publish(Empty())
            self.detach_count -= 1

        self.read_udp()
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = BASE

        if self.hand is None or time.time() - self.last_msg_time > 0.5:
            self.pub.publish(twist)
            return

        pinch = self.hand["pinch"]
        if self.gripper_closed is not True and pinch < CLOSE_BELOW:
            self.set_gripper(True)
        elif self.gripper_closed is not False and pinch > OPEN_ABOVE:
            self.set_gripper(False)

        try:
            tf = self.tf_buffer.lookup_transform(BASE, EE, rclpy.time.Time())
        except Exception:
            return
        ee = tf.transform.translation

        if self.ref_hand is None:
            self.ref_hand = dict(self.hand)
            self.ref_ee = (ee.x, ee.y, ee.z)
            self.get_logger().info("Kalibriert, Hand steuert jetzt den Arm")

        dx = (self.hand["size"] / self.ref_hand["size"] - 1.0) * SCALE_X
        dy = (self.hand["x"] - self.ref_hand["x"]) * SCALE_Y
        dz = -(self.hand["y"] - self.ref_hand["y"]) * SCALE_Z

        dx = clamp(dx, -LIMIT_X, LIMIT_X)
        dy = clamp(dy, -LIMIT_Y, LIMIT_Y)
        dz = clamp(dz, -LIMIT_Z_DOWN, LIMIT_Z_UP)

        target_x = self.ref_ee[0] + dx
        target_y = self.ref_ee[1] + dy
        target_z = self.ref_ee[2] + dz

        twist.twist.linear.x = clamp(GAIN * (target_x - ee.x), -MAX_SPEED, MAX_SPEED)
        twist.twist.linear.y = clamp(GAIN * (target_y - ee.y), -MAX_SPEED, MAX_SPEED)
        twist.twist.linear.z = clamp(GAIN * (target_z - ee.z), -MAX_SPEED, MAX_SPEED)
        self.pub.publish(twist)


def main():
    rclpy.init()
    node = HandServo()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
