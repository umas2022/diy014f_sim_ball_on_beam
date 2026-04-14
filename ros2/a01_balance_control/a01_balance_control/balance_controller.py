from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class BalanceController(Node):
    def __init__(self) -> None:
        super().__init__('balance_controller')

        self.declare_parameter('control_rate', 100.0)
        self.declare_parameter('ball_pos_ref', 0.0)
        self.declare_parameter('ball_pos_kp', 2.6)
        self.declare_parameter('ball_vel_kd', 1.1)
        self.declare_parameter('control_sign', 1.0)
        self.declare_parameter('track_angle_kp', 8.0)
        self.declare_parameter('track_rate_kd', 0.55)
        self.declare_parameter('max_track_angle', 0.10)
        self.declare_parameter('max_track_velocity', 2.2)
        self.declare_parameter('command_alpha', 0.18)
        self.declare_parameter('max_cmd_step', 0.20)
        self.declare_parameter('ball_pos_alpha', 0.20)
        self.declare_parameter('ball_vel_alpha', 0.18)
        self.declare_parameter('track_angle_alpha', 0.20)
        self.declare_parameter('track_rate_alpha', 0.16)
        self.declare_parameter('track_angle_limit_for_stop', 0.22)
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter(
            'track_cmd_topic',
            '/model/a01_balance/joint/joint_track/cmd_vel',
        )
        self.declare_parameter('track_joint_name', 'joint_track')
        self.declare_parameter('ball_joint_name', 'joint_ball')

        self.control_rate = float(self.get_parameter('control_rate').value)
        self.ball_pos_ref = float(self.get_parameter('ball_pos_ref').value)
        self.ball_pos_kp = float(self.get_parameter('ball_pos_kp').value)
        self.ball_vel_kd = float(self.get_parameter('ball_vel_kd').value)
        self.control_sign = float(self.get_parameter('control_sign').value)
        self.track_angle_kp = float(self.get_parameter('track_angle_kp').value)
        self.track_rate_kd = float(self.get_parameter('track_rate_kd').value)
        self.max_track_angle = float(self.get_parameter('max_track_angle').value)
        self.max_track_velocity = float(self.get_parameter('max_track_velocity').value)
        self.command_alpha = float(self.get_parameter('command_alpha').value)
        self.max_cmd_step = float(self.get_parameter('max_cmd_step').value)
        self.ball_pos_alpha = float(self.get_parameter('ball_pos_alpha').value)
        self.ball_vel_alpha = float(self.get_parameter('ball_vel_alpha').value)
        self.track_angle_alpha = float(self.get_parameter('track_angle_alpha').value)
        self.track_rate_alpha = float(self.get_parameter('track_rate_alpha').value)
        self.track_angle_limit_for_stop = float(
            self.get_parameter('track_angle_limit_for_stop').value
        )
        joint_states_topic = str(self.get_parameter('joint_states_topic').value)
        track_cmd_topic = str(self.get_parameter('track_cmd_topic').value)
        self.track_joint_name = str(self.get_parameter('track_joint_name').value)
        self.ball_joint_name = str(self.get_parameter('ball_joint_name').value)

        self.ball_pos = 0.0
        self.ball_vel = 0.0
        self.track_angle = 0.0
        self.track_rate = 0.0
        self.filtered_ball_pos = 0.0
        self.filtered_ball_vel = 0.0
        self.filtered_track_angle = 0.0
        self.filtered_track_rate = 0.0
        self.have_state = False
        self.last_command = 0.0
        self.last_debug_time = self.get_clock().now()

        self.create_subscription(JointState, joint_states_topic, self.joint_states_cb, 50)
        self.cmd_pub = self.create_publisher(Float64, track_cmd_topic, 20)
        self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.get_logger().info(
            'Balance controller started with '
            f'ball_joint={self.ball_joint_name}, track_joint={self.track_joint_name}, '
            f'cmd_topic={track_cmd_topic}'
        )

    def joint_states_cb(self, msg: JointState) -> None:
        indices = {name: i for i, name in enumerate(msg.name)}
        if self.track_joint_name not in indices or self.ball_joint_name not in indices:
            return

        track_index = indices[self.track_joint_name]
        ball_index = indices[self.ball_joint_name]

        self.track_angle = msg.position[track_index]
        self.track_rate = msg.velocity[track_index]
        self.ball_pos = msg.position[ball_index]
        self.ball_vel = msg.velocity[ball_index]

        if not self.have_state:
            self.filtered_ball_pos = self.ball_pos
            self.filtered_ball_vel = self.ball_vel
            self.filtered_track_angle = self.track_angle
            self.filtered_track_rate = self.track_rate
            self.have_state = True
            return

        self.filtered_ball_pos = self._filter(
            self.filtered_ball_pos,
            self.ball_pos,
            self.ball_pos_alpha,
        )
        self.filtered_ball_vel = self._filter(
            self.filtered_ball_vel,
            self.ball_vel,
            self.ball_vel_alpha,
        )
        self.filtered_track_angle = self._filter(
            self.filtered_track_angle,
            self.track_angle,
            self.track_angle_alpha,
        )
        self.filtered_track_rate = self._filter(
            self.filtered_track_rate,
            self.track_rate,
            self.track_rate_alpha,
        )

    def control_loop(self) -> None:
        if not self.have_state:
            self.publish_command(0.0)
            return

        if abs(self.filtered_track_angle) > self.track_angle_limit_for_stop:
            self.publish_command(0.0)
            self.get_logger().warn(
                'Track angle exceeded safe limit, zeroing command.',
                throttle_duration_sec=1.0,
            )
            return

        pos_error = self.filtered_ball_pos - self.ball_pos_ref
        desired_track_angle = self.control_sign * (
            self.ball_pos_kp * pos_error + self.ball_vel_kd * self.filtered_ball_vel
        )
        desired_track_angle = clamp(
            desired_track_angle,
            -self.max_track_angle,
            self.max_track_angle,
        )

        raw_command = (
            self.track_angle_kp * (desired_track_angle - self.filtered_track_angle)
            - self.track_rate_kd * self.filtered_track_rate
        )
        smoothed_command = self._filter(
            self.last_command,
            raw_command,
            self.command_alpha,
        )
        stepped_command = clamp(
            smoothed_command,
            self.last_command - self.max_cmd_step,
            self.last_command + self.max_cmd_step,
        )
        final_command = clamp(
            stepped_command,
            -self.max_track_velocity,
            self.max_track_velocity,
        )
        self.publish_command(final_command)

        now = self.get_clock().now()
        if (now - self.last_debug_time).nanoseconds > int(1.0e9):
            self.last_debug_time = now
            self.get_logger().info(
                'ball_pos=%.4f ball_vel=%.4f track=%.4f target=%.4f cmd=%.4f'
                % (
                    self.filtered_ball_pos,
                    self.filtered_ball_vel,
                    self.filtered_track_angle,
                    desired_track_angle,
                    final_command,
                )
            )

    def publish_command(self, value: float) -> None:
        msg = Float64()
        msg.data = float(value)
        self.cmd_pub.publish(msg)
        self.last_command = msg.data

    @staticmethod
    def _filter(current: float, new_value: float, alpha: float) -> float:
        alpha = clamp(alpha, 0.0, 1.0)
        return (1.0 - alpha) * current + alpha * new_value


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = BalanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_command(0.0)
        node.destroy_node()
        rclpy.shutdown()
