from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from typing import Sequence

import numpy as np


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def low_pass(current: float, new_value: float, alpha: float) -> float:
    alpha = clamp(alpha, 0.0, 1.0)
    return (1.0 - alpha) * current + alpha * new_value


def quat_from_x_rotation(angle_rad: float) -> np.ndarray:
    half = 0.5 * angle_rad
    return np.array([math.cos(half), math.sin(half), 0.0, 0.0], dtype=np.float64)


def quat_to_rot_matrix(quat_wxyz: Sequence[float]) -> np.ndarray:
    w, x, y, z = quat_wxyz
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


@dataclass
class ControllerConfig:
    ball_pos_ref: float = 0.0
    control_sign: float = 1.0
    max_track_angle: float = 0.022
    track_angle_limit_for_stop: float = 0.12
    position_deadband: float = 0.0012
    velocity_deadband: float = 0.006
    integral_deadband: float = 0.004
    integral_limit: float = 0.08
    settling_position_threshold: float = 0.004
    settling_velocity_threshold: float = 0.020
    settling_track_threshold: float = 0.010
    settling_rate_threshold: float = 0.04
    settling_blend: float = 0.90
    hold_position_entry: float = 0.005
    hold_velocity_entry: float = 0.018
    hold_position_exit: float = 0.010
    hold_velocity_exit: float = 0.035
    hold_max_track_angle: float = 0.010
    hold_pos_gain: float = 0.40
    hold_vel_gain: float = 0.18
    hold_angle_gain: float = 1.10
    hold_rate_gain: float = 0.38
    lqi_pos_gain: float = 1.45
    lqi_vel_gain: float = 1.20
    lqi_angle_gain: float = 0.78
    lqi_rate_gain: float = 0.30
    lqi_integral_gain: float = 0.20
    estimator_q_pos: float = 1.0e-5
    estimator_q_vel: float = 7.0e-4
    estimator_r_pos: float = 2.5e-5
    estimator_r_vel: float = 2.0e-3


@dataclass
class BeamActuatorConfig:
    target_angle_alpha: float = 0.06
    velocity_alpha: float = 0.12
    max_velocity: float = 0.18
    max_acceleration: float = 0.9
    centering_gain: float = 0.22
    centering_deadband: float = 0.003


class ConstantVelocityKalmanFilter:
    def __init__(self, q_pos: float, q_vel: float, r_pos: float, r_vel: float) -> None:
        self.q_pos = q_pos
        self.q_vel = q_vel
        self.r_pos = r_pos
        self.r_vel = r_vel
        self.reset()

    def reset(self, position: float = 0.0, velocity: float = 0.0) -> None:
        self.state = np.array([position, velocity], dtype=np.float64)
        self.covariance = np.eye(2, dtype=np.float64) * 1.0e-3

    def update(self, dt: float, measured_position: float, measured_velocity: float) -> tuple[float, float]:
        a = np.array([[1.0, dt], [0.0, 1.0]], dtype=np.float64)
        q = np.array([[self.q_pos, 0.0], [0.0, self.q_vel]], dtype=np.float64)
        h = np.eye(2, dtype=np.float64)
        r = np.array([[self.r_pos, 0.0], [0.0, self.r_vel]], dtype=np.float64)
        z = np.array([measured_position, measured_velocity], dtype=np.float64)

        predicted_state = a @ self.state
        predicted_cov = a @ self.covariance @ a.T + q

        innovation = z - (h @ predicted_state)
        innovation_cov = h @ predicted_cov @ h.T + r
        kalman_gain = predicted_cov @ np.linalg.inv(innovation_cov)

        self.state = predicted_state + kalman_gain @ innovation
        self.covariance = (np.eye(2, dtype=np.float64) - kalman_gain @ h) @ predicted_cov
        return float(self.state[0]), float(self.state[1])


class BalanceController:
    def __init__(self, config: ControllerConfig) -> None:
        self.config = config
        self.ball_filter = ConstantVelocityKalmanFilter(
            q_pos=config.estimator_q_pos,
            q_vel=config.estimator_q_vel,
            r_pos=config.estimator_r_pos,
            r_vel=config.estimator_r_vel,
        )
        self.reset()

    def reset(self) -> None:
        self.estimated_ball_pos = 0.0
        self.estimated_ball_vel = 0.0
        self.error_integral = 0.0
        self.last_desired_angle = 0.0
        self.hold_mode = False
        self.have_state = False

    def update(
        self,
        dt: float,
        ball_pos: float,
        ball_vel: float,
        track_angle: float,
        track_rate: float,
    ) -> tuple[float, float]:
        cfg = self.config
        if not self.have_state:
            self.estimated_ball_pos = ball_pos
            self.estimated_ball_vel = ball_vel
            self.ball_filter.reset(ball_pos, ball_vel)
            self.have_state = True
            return track_angle, 0.0

        self.estimated_ball_pos, self.estimated_ball_vel = self.ball_filter.update(dt, ball_pos, ball_vel)

        if abs(track_angle) > cfg.track_angle_limit_for_stop:
            self.last_desired_angle = 0.0
            return 0.0, 0.0

        pos_error = self.estimated_ball_pos - cfg.ball_pos_ref
        if abs(pos_error) < cfg.position_deadband:
            pos_error = 0.0
        estimated_ball_vel = self.estimated_ball_vel
        if abs(estimated_ball_vel) < cfg.velocity_deadband:
            estimated_ball_vel = 0.0
        if abs(pos_error) < cfg.integral_deadband:
            self.error_integral *= 0.96
        else:
            self.error_integral += pos_error * dt
        self.error_integral = clamp(self.error_integral, -cfg.integral_limit, cfg.integral_limit)

        if self.hold_mode:
            if abs(pos_error) > cfg.hold_position_exit or abs(estimated_ball_vel) > cfg.hold_velocity_exit:
                self.hold_mode = False
        elif abs(pos_error) < cfg.hold_position_entry and abs(estimated_ball_vel) < cfg.hold_velocity_entry:
            self.hold_mode = True

        if self.hold_mode:
            desired_track_angle = cfg.control_sign * (
                cfg.hold_pos_gain * pos_error + cfg.hold_vel_gain * estimated_ball_vel
            ) - cfg.hold_angle_gain * track_angle - cfg.hold_rate_gain * track_rate
            desired_track_angle = clamp(
                desired_track_angle,
                -cfg.hold_max_track_angle,
                cfg.hold_max_track_angle,
            )
            self.error_integral *= 0.92
        else:
            desired_track_angle = cfg.control_sign * (
                cfg.lqi_pos_gain * pos_error
                + cfg.lqi_vel_gain * estimated_ball_vel
                + cfg.lqi_integral_gain * self.error_integral
            ) - cfg.lqi_angle_gain * track_angle - cfg.lqi_rate_gain * track_rate

        desired_track_angle = clamp(
            desired_track_angle,
            -cfg.max_track_angle,
            cfg.max_track_angle,
        )
        if (
            abs(pos_error) < cfg.settling_position_threshold
            and abs(estimated_ball_vel) < cfg.settling_velocity_threshold
            and abs(track_angle) < cfg.settling_track_threshold
            and abs(track_rate) < cfg.settling_rate_threshold
        ):
            desired_track_angle *= 1.0 - cfg.settling_blend
            self.error_integral *= 0.90

        final_command = low_pass(self.last_desired_angle, desired_track_angle, 0.16)
        final_command = clamp(
            final_command,
            -cfg.max_track_angle,
            cfg.max_track_angle,
        )
        self.last_desired_angle = final_command
        return desired_track_angle, final_command


class BeamActuator:
    def __init__(self, config: BeamActuatorConfig, angle_limit: float) -> None:
        self.config = config
        self.angle_limit = angle_limit
        self.reset()

    def reset(self) -> None:
        self.angle = 0.0
        self.rate = 0.0

    def update(self, desired_angle: float, dt: float) -> tuple[float, float]:
        cfg = self.config
        limited_desired_angle = clamp(desired_angle, -self.angle_limit, self.angle_limit)
        filtered_target_angle = low_pass(self.angle, limited_desired_angle, cfg.target_angle_alpha)

        if abs(filtered_target_angle) < cfg.centering_deadband:
            filtered_target_angle = (1.0 - cfg.centering_gain) * filtered_target_angle

        target_rate = (filtered_target_angle - self.angle) / max(dt, 1.0e-6)
        target_rate = clamp(target_rate, -cfg.max_velocity, cfg.max_velocity)
        smoothed_rate = low_pass(self.rate, target_rate, cfg.velocity_alpha)
        max_rate_step = cfg.max_acceleration * dt
        next_rate = clamp(smoothed_rate, self.rate - max_rate_step, self.rate + max_rate_step)
        next_rate = clamp(next_rate, -cfg.max_velocity, cfg.max_velocity)

        next_angle = self.angle + next_rate * dt
        next_angle = clamp(next_angle, -self.angle_limit, self.angle_limit)
        if next_angle in (-self.angle_limit, self.angle_limit):
            next_rate = 0.0

        self.angle = next_angle
        self.rate = next_rate
        return self.angle, self.rate


class BeamAssembly:
    def __init__(
        self,
        world,
        FixedCuboid,
        beam_material,
        pivot_position: np.ndarray,
        beam_length: float,
        beam_width: float,
        beam_thickness: float,
        rail_height: float,
        rail_thickness: float,
    ) -> None:
        self._pivot_position = pivot_position.astype(np.float64)
        self._angle = 0.0

        self._parts = [
            (
                world.scene.add(
                    FixedCuboid(
                        prim_path="/World/Beam/track",
                        name="track",
                        position=self._pivot_position,
                        scale=np.array([beam_width, beam_length, beam_thickness], dtype=np.float64),
                        color=np.array([0.85, 0.85, 0.9], dtype=np.float64),
                        physics_material=beam_material,
                    )
                ),
                np.array([0.0, 0.0, 0.0], dtype=np.float64),
            ),
            (
                world.scene.add(
                    FixedCuboid(
                        prim_path="/World/Beam/rail_left",
                        name="rail_left",
                        position=self._pivot_position,
                        scale=np.array([rail_thickness, beam_length, rail_height], dtype=np.float64),
                        color=np.array([0.3, 0.45, 0.8], dtype=np.float64),
                        physics_material=beam_material,
                    )
                ),
                np.array(
                    [0.5 * (beam_width - rail_thickness), 0.0, 0.5 * (beam_thickness + rail_height)],
                    dtype=np.float64,
                ),
            ),
            (
                world.scene.add(
                    FixedCuboid(
                        prim_path="/World/Beam/rail_right",
                        name="rail_right",
                        position=self._pivot_position,
                        scale=np.array([rail_thickness, beam_length, rail_height], dtype=np.float64),
                        color=np.array([0.3, 0.45, 0.8], dtype=np.float64),
                        physics_material=beam_material,
                    )
                ),
                np.array(
                    [-0.5 * (beam_width - rail_thickness), 0.0, 0.5 * (beam_thickness + rail_height)],
                    dtype=np.float64,
                ),
            ),
            (
                world.scene.add(
                    FixedCuboid(
                        prim_path="/World/Beam/end_stop_pos",
                        name="end_stop_pos",
                        position=self._pivot_position,
                        scale=np.array([beam_width, rail_thickness, rail_height], dtype=np.float64),
                        color=np.array([0.75, 0.3, 0.3], dtype=np.float64),
                        physics_material=beam_material,
                    )
                ),
                np.array(
                    [0.0, 0.5 * (beam_length - rail_thickness), 0.5 * (beam_thickness + rail_height)],
                    dtype=np.float64,
                ),
            ),
            (
                world.scene.add(
                    FixedCuboid(
                        prim_path="/World/Beam/end_stop_neg",
                        name="end_stop_neg",
                        position=self._pivot_position,
                        scale=np.array([beam_width, rail_thickness, rail_height], dtype=np.float64),
                        color=np.array([0.75, 0.3, 0.3], dtype=np.float64),
                        physics_material=beam_material,
                    )
                ),
                np.array(
                    [0.0, -0.5 * (beam_length - rail_thickness), 0.5 * (beam_thickness + rail_height)],
                    dtype=np.float64,
                ),
            ),
        ]
        self.set_angle(0.0)

    @property
    def pivot_position(self) -> np.ndarray:
        return self._pivot_position.copy()

    @property
    def angle(self) -> float:
        return self._angle

    def set_angle(self, angle_rad: float) -> None:
        self._angle = float(angle_rad)
        orientation = quat_from_x_rotation(self._angle)
        rotation = quat_to_rot_matrix(orientation)
        for part, local_offset in self._parts:
            world_position = self._pivot_position + rotation @ local_offset
            part.set_world_pose(position=world_position, orientation=orientation)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Isaac Sim ball-on-beam demo")
    parser.add_argument("--headless", action="store_true", help="Run without GUI")
    parser.add_argument("--max-steps", type=int, default=0, help="Stop automatically after N steps, 0 means unlimited")
    parser.add_argument("--physics-dt", type=float, default=1.0 / 120.0, help="Physics timestep")
    parser.add_argument("--rendering-dt", type=float, default=1.0 / 60.0, help="Rendering timestep")
    parser.add_argument("--ball-pos-ref", type=float, default=0.0, help="Target ball position along the beam in meters")
    parser.add_argument("--beam-angle-limit", type=float, default=0.08, help="Hard beam angle clamp in radians")
    parser.add_argument("--spawn-offset", type=float, default=0.018, help="Initial ball offset along the beam in meters")
    parser.add_argument("--debug-interval", type=float, default=0.5, help="Seconds between debug prints")
    parser.add_argument("--ball-linear-damping", type=float, default=0.18, help="Ball rigid-body linear damping")
    parser.add_argument("--ball-angular-damping", type=float, default=1.20, help="Ball rigid-body angular damping")
    parser.add_argument("--lqi-pos-gain", type=float, default=1.45, help="LQI position-state gain")
    parser.add_argument("--lqi-vel-gain", type=float, default=1.20, help="LQI velocity-state gain")
    parser.add_argument("--lqi-angle-gain", type=float, default=0.78, help="LQI beam-angle damping gain")
    parser.add_argument("--lqi-rate-gain", type=float, default=0.30, help="LQI beam-rate damping gain")
    parser.add_argument("--lqi-integral-gain", type=float, default=0.20, help="LQI integral gain")
    parser.add_argument("--max-track-angle", type=float, default=0.022, help="Controller target angle clamp in radians")
    parser.add_argument("--actuator-max-velocity", type=float, default=0.18, help="Beam actuator velocity limit in rad/s")
    parser.add_argument("--actuator-max-acceleration", type=float, default=0.9, help="Beam actuator acceleration limit in rad/s^2")
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(line_buffering=True)

    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": args.headless})

    try:
        from isaacsim.core.api import World
        from isaacsim.core.api.materials import PhysicsMaterial
        from isaacsim.core.api.objects import DynamicSphere, FixedCuboid, GroundPlane
        from isaacsim.core.utils.viewports import set_camera_view
        from pxr import PhysxSchema, Sdf, UsdLux

        world = World(
            stage_units_in_meters=1.0,
            physics_dt=args.physics_dt,
            rendering_dt=args.rendering_dt,
            backend="numpy",
            device="cpu",
        )

        stage = world.stage
        light = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/DistantLight"))
        light.CreateIntensityAttr(2500)

        if not args.headless:
            set_camera_view(
                eye=np.array([1.2, -1.5, 0.7], dtype=np.float64),
                target=np.array([0.0, 0.0, 0.1], dtype=np.float64),
                camera_prim_path="/OmniverseKit_Persp",
            )

        world.scene.add(GroundPlane(prim_path="/World/GroundPlane", z_position=0.0))

        beam_material = PhysicsMaterial(
            prim_path="/World/PhysicsMaterials/BeamMaterial",
            dynamic_friction=0.85,
            static_friction=0.95,
            restitution=0.0,
        )
        ball_material = PhysicsMaterial(
            prim_path="/World/PhysicsMaterials/BallMaterial",
            dynamic_friction=0.95,
            static_friction=1.10,
            restitution=0.0,
        )

        support_height = 0.11
        support = world.scene.add(
            FixedCuboid(
                prim_path="/World/Support",
                name="support",
                position=np.array([0.0, 0.0, 0.5 * support_height], dtype=np.float64),
                scale=np.array([0.08, 0.08, support_height], dtype=np.float64),
                color=np.array([0.18, 0.18, 0.2], dtype=np.float64),
                physics_material=beam_material,
            )
        )
        _ = support

        beam_length = 0.26
        beam_width = 0.05
        beam_thickness = 0.008
        rail_height = 0.015
        rail_thickness = 0.006
        ball_radius = 0.012
        pivot_position = np.array([0.0, 0.0, support_height + 0.04], dtype=np.float64)

        beam = BeamAssembly(
            world=world,
            FixedCuboid=FixedCuboid,
            beam_material=beam_material,
            pivot_position=pivot_position,
            beam_length=beam_length,
            beam_width=beam_width,
            beam_thickness=beam_thickness,
            rail_height=rail_height,
            rail_thickness=rail_thickness,
        )

        ball_start_local = np.array(
            [0.0, clamp(args.spawn_offset, -0.08, 0.08), 0.5 * beam_thickness + ball_radius + 0.002],
            dtype=np.float64,
        )
        ball_start_world = beam.pivot_position + ball_start_local
        ball = world.scene.add(
            DynamicSphere(
                prim_path="/World/Ball",
                name="ball",
                position=ball_start_world,
                radius=ball_radius,
                mass=0.11,
                color=np.array([0.95, 0.55, 0.15], dtype=np.float64),
                physics_material=ball_material,
            )
        )
        ball_rigid_body = PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath("/World/Ball"))
        ball_rigid_body.CreateLinearDampingAttr().Set(args.ball_linear_damping)
        ball_rigid_body.CreateAngularDampingAttr().Set(args.ball_angular_damping)

        controller = BalanceController(
            ControllerConfig(
                ball_pos_ref=args.ball_pos_ref,
                max_track_angle=min(args.max_track_angle, args.beam_angle_limit),
                lqi_pos_gain=args.lqi_pos_gain,
                lqi_vel_gain=args.lqi_vel_gain,
                lqi_angle_gain=args.lqi_angle_gain,
                lqi_rate_gain=args.lqi_rate_gain,
                lqi_integral_gain=args.lqi_integral_gain,
            )
        )
        beam_actuator = BeamActuator(
            BeamActuatorConfig(
                max_velocity=min(args.actuator_max_velocity, args.beam_angle_limit / max(args.physics_dt, 1.0e-6)),
                max_acceleration=args.actuator_max_acceleration,
            ),
            angle_limit=args.beam_angle_limit,
        )

        def reset_scene() -> None:
            controller.reset()
            beam_actuator.reset()
            beam.set_angle(0.0)
            ball.set_world_pose(position=ball_start_world, orientation=np.array([1.0, 0.0, 0.0, 0.0]))
            ball.set_linear_velocity(np.zeros(3, dtype=np.float64))
            ball.set_angular_velocity(np.zeros(3, dtype=np.float64))

        world.reset()
        reset_scene()
        world.play()

        steps = 0
        elapsed = 0.0
        debug_timer = 0.0
        while True:
            if not args.headless and not simulation_app.is_running():
                break
            ball_position, _ = ball.get_world_pose()
            ball_velocity = ball.get_linear_velocity()
            beam_rotation = quat_to_rot_matrix(quat_from_x_rotation(beam_actuator.angle))
            local_position = beam_rotation.T @ (ball_position - beam.pivot_position)
            local_velocity = beam_rotation.T @ ball_velocity

            desired_angle, commanded_angle = controller.update(
                dt=args.physics_dt,
                ball_pos=float(local_position[1]),
                ball_vel=float(local_velocity[1]),
                track_angle=beam_actuator.angle,
                track_rate=beam_actuator.rate,
            )

            beam_angle, beam_rate = beam_actuator.update(desired_angle=commanded_angle, dt=args.physics_dt)
            beam.set_angle(beam_angle)

            world.step(render=not args.headless)

            steps += 1
            elapsed += args.physics_dt
            debug_timer += args.physics_dt

            if debug_timer >= args.debug_interval:
                debug_timer = 0.0
                print(
                    "t=%.2f ball_pos=%.4f ball_vel=%.4f desired=%.4f cmd=%.4f beam=%.4f beam_rate=%.4f"
                    % (
                        elapsed,
                        local_position[1],
                        local_velocity[1],
                        desired_angle,
                        commanded_angle,
                        beam_angle,
                        beam_rate,
                    ),
                    flush=True,
                )

            if abs(local_position[1]) > 0.12 or ball_position[2] < 0.02:
                print("reset: ball left the controllable area", flush=True)
                reset_scene()
                world.play()

            if args.max_steps > 0 and steps >= args.max_steps:
                break

    finally:
        simulation_app.close()


if __name__ == "__main__":
    main()
