from __future__ import annotations

import argparse
import math
import re
import shlex
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]
ROS2_ROOT = REPO_ROOT / "ros2"
DEFAULT_XACRO_PATH = ROS2_ROOT / "a01_balance_1dof" / "urdf" / "a01_balance_1dof.xacro"


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def low_pass(current: float, new_value: float, alpha: float) -> float:
    alpha = clamp(alpha, 0.0, 1.0)
    return (1.0 - alpha) * current + alpha * new_value


@dataclass
class ControllerConfig:
    ball_pos_ref: float = 0.0
    control_sign: float = 1.0
    max_track_angle: float = 0.022
    track_angle_limit_for_stop: float = 0.12
    position_deadband: float = 0.0012
    velocity_deadband: float = 0.006
    integral_deadband: float = 0.006
    integral_limit: float = 0.04
    settling_position_threshold: float = 0.008
    settling_velocity_threshold: float = 0.030
    settling_track_threshold: float = 0.008
    settling_rate_threshold: float = 0.03
    settling_blend: float = 0.95
    hold_position_entry: float = 0.012
    hold_velocity_entry: float = 0.030
    hold_position_exit: float = 0.018
    hold_velocity_exit: float = 0.050
    hold_max_track_angle: float = 0.006
    hold_pos_gain: float = 0.24
    hold_vel_gain: float = 0.10
    hold_angle_gain: float = 1.45
    hold_rate_gain: float = 0.62
    lqi_pos_gain: float = 1.45
    lqi_vel_gain: float = 1.20
    lqi_angle_gain: float = 0.78
    lqi_rate_gain: float = 0.55
    lqi_integral_gain: float = 0.06
    estimator_q_pos: float = 1.0e-5
    estimator_q_vel: float = 7.0e-4
    estimator_r_pos: float = 2.5e-5
    estimator_r_vel: float = 2.0e-3


@dataclass
class BeamActuatorConfig:
    target_angle_alpha: float = 0.045
    velocity_alpha: float = 0.10
    max_velocity: float = 0.12
    max_acceleration: float = 0.45
    centering_gain: float = 0.40
    centering_deadband: float = 0.006


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
            desired_track_angle = clamp(desired_track_angle, -cfg.hold_max_track_angle, cfg.hold_max_track_angle)
            self.error_integral *= 0.92
        else:
            desired_track_angle = cfg.control_sign * (
                cfg.lqi_pos_gain * pos_error
                + cfg.lqi_vel_gain * estimated_ball_vel
                + cfg.lqi_integral_gain * self.error_integral
            ) - cfg.lqi_angle_gain * track_angle - cfg.lqi_rate_gain * track_rate

        desired_track_angle = clamp(desired_track_angle, -cfg.max_track_angle, cfg.max_track_angle)
        if (
            abs(pos_error) < cfg.settling_position_threshold
            and abs(estimated_ball_vel) < cfg.settling_velocity_threshold
            and abs(track_angle) < cfg.settling_track_threshold
            and abs(track_rate) < cfg.settling_rate_threshold
        ):
            desired_track_angle *= 1.0 - cfg.settling_blend
            self.error_integral *= 0.90

        final_command = low_pass(self.last_desired_angle, desired_track_angle, 0.10)
        final_command = clamp(final_command, -cfg.max_track_angle, cfg.max_track_angle)
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


@dataclass
class VelocityCommandConfig:
    track_vel_kp: float = 8.0
    track_vel_kd: float = 0.85
    command_alpha: float = 0.14
    max_cmd_step: float = 0.08
    max_track_velocity: float = 0.8


class VelocityCommandController:
    def __init__(self, config: VelocityCommandConfig) -> None:
        self.config = config
        self.reset()

    def reset(self) -> None:
        self.last_command = 0.0

    def update(self, target_angle: float, track_angle: float, track_rate: float) -> float:
        cfg = self.config
        raw_command = cfg.track_vel_kp * (target_angle - track_angle) - cfg.track_vel_kd * track_rate
        smoothed_command = low_pass(self.last_command, raw_command, cfg.command_alpha)
        stepped_command = clamp(
            smoothed_command,
            self.last_command - cfg.max_cmd_step,
            self.last_command + cfg.max_cmd_step,
        )
        final_command = clamp(stepped_command, -cfg.max_track_velocity, cfg.max_track_velocity)
        self.last_command = final_command
        return final_command


def render_xacro_to_urdf(xacro_path: Path) -> str:
    command = (
        "source /opt/ros/jazzy/setup.bash >/dev/null 2>&1 && "
        f"xacro {shlex.quote(str(xacro_path))}"
    )
    result = subprocess.run(
        ["/bin/bash", "-lc", command],
        check=True,
        text=True,
        capture_output=True,
    )
    return result.stdout


def replace_package_urls(urdf_text: str) -> str:
    package_paths = {
        package_dir.name: package_dir
        for package_dir in ROS2_ROOT.iterdir()
        if package_dir.is_dir() and (package_dir / "package.xml").exists()
    }

    def replace_match(match: re.Match[str]) -> str:
        package_name = match.group(1)
        if package_name not in package_paths:
            raise FileNotFoundError(f"package://{package_name} not found under {ROS2_ROOT}")
        return str(package_paths[package_name])

    return re.sub(r"package://([^/]+)", replace_match, urdf_text)


def write_resolved_urdf(xacro_path: Path) -> tuple[Path, str]:
    urdf_text = replace_package_urls(render_xacro_to_urdf(xacro_path))
    robot_name_match = re.search(r"<robot[^>]*name=\"([^\"]+)\"", urdf_text)
    if robot_name_match is None:
        raise ValueError("Failed to detect robot name in rendered URDF")
    temp_file = tempfile.NamedTemporaryFile(prefix="isaac_a01_", suffix=".urdf", delete=False)
    temp_path = Path(temp_file.name)
    temp_file.write(urdf_text.encode("utf-8"))
    temp_file.flush()
    temp_file.close()
    return temp_path, robot_name_match.group(1)


def debug_print_imported_structure(stage, robot_prim_path: str) -> None:
    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    print(f"Robot children under {robot_prim_path}:", flush=True)
    for child in robot_prim.GetChildren():
        print(f"  child: {child.GetPath()}", flush=True)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Isaac Sim ball-on-beam demo using the ROS2 model")
    parser.add_argument("--headless", action="store_true", help="Run without GUI")
    parser.add_argument("--max-steps", type=int, default=0, help="Stop automatically after N steps, 0 means unlimited")
    parser.add_argument("--physics-dt", type=float, default=1.0 / 120.0, help="Physics timestep")
    parser.add_argument("--rendering-dt", type=float, default=1.0 / 60.0, help="Rendering timestep")
    parser.add_argument("--ball-pos-ref", type=float, default=0.0, help="Target ball position along the beam in meters")
    parser.add_argument("--control-sign", type=float, default=1.0, help="Control direction sign, use -1 if the beam reacts in reverse")
    parser.add_argument("--beam-angle-limit", type=float, default=0.08, help="Hard beam angle clamp in radians")
    parser.add_argument("--spawn-offset", type=float, default=0.006, help="Initial ball offset along the beam in meters")
    parser.add_argument("--spawn-z", type=float, default=0.08, help="Initial robot base height in meters")
    parser.add_argument("--debug-interval", type=float, default=0.5, help="Seconds between debug prints")
    parser.add_argument("--xacro-path", type=Path, default=DEFAULT_XACRO_PATH, help="Path to the source xacro model")
    parser.add_argument("--lqi-pos-gain", type=float, default=1.60, help="LQI position-state gain")
    parser.add_argument("--lqi-vel-gain", type=float, default=1.10, help="LQI velocity-state gain")
    parser.add_argument("--lqi-angle-gain", type=float, default=1.00, help="LQI beam-angle damping gain")
    parser.add_argument("--lqi-rate-gain", type=float, default=0.50, help="LQI beam-rate damping gain")
    parser.add_argument("--lqi-integral-gain", type=float, default=0.10, help="LQI integral gain")
    parser.add_argument("--max-track-angle", type=float, default=0.05, help="Controller target angle clamp in radians")
    parser.add_argument("--actuator-max-velocity", type=float, default=0.35, help="Beam actuator velocity limit in rad/s")
    parser.add_argument("--actuator-max-acceleration", type=float, default=1.2, help="Beam actuator acceleration limit in rad/s^2")
    parser.add_argument("--track-vel-kp", type=float, default=8.0, help="Velocity-loop proportional gain from target angle to joint velocity")
    parser.add_argument("--track-vel-kd", type=float, default=0.55, help="Velocity-loop damping on measured joint rate")
    parser.add_argument("--command-alpha", type=float, default=0.18, help="Low-pass factor applied to velocity commands")
    parser.add_argument("--max-cmd-step", type=float, default=0.12, help="Maximum command change per control step")
    parser.add_argument("--max-track-velocity", type=float, default=1.2, help="Clamp for joint velocity command in rad/s")
    parser.add_argument("--track-drive-stiffness", type=float, default=0.0, help="Imported track joint drive stiffness")
    parser.add_argument("--track-drive-damping", type=float, default=8000.0, help="Imported track joint drive damping")
    parser.add_argument("--reset-threshold", type=float, default=0.049, help="Reset when the ball prismatic position magnitude exceeds this value")
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(line_buffering=True)

    resolved_urdf_path, robot_name = write_resolved_urdf(args.xacro_path)

    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": args.headless})

    try:
        import omni.kit.commands
        from isaacsim.asset.importer.urdf._urdf import UrdfJointTargetType
        from isaacsim.core.api import World
        from isaacsim.core.api.objects import GroundPlane
        from isaacsim.core.utils.viewports import set_camera_view
        from pxr import Gf, PhysxSchema, Sdf, UsdLux, UsdPhysics

        world = World(
            stage_units_in_meters=1.0,
            physics_dt=args.physics_dt,
            rendering_dt=args.rendering_dt,
            backend="numpy",
            device="cpu",
        )

        stage = world.stage
        world.scene.add(GroundPlane(prim_path="/World/GroundPlane", z_position=0.0))

        light = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/DistantLight"))
        light.CreateIntensityAttr(2500)

        if not args.headless:
            set_camera_view(
                eye=np.array([1.1, -1.4, 0.55], dtype=np.float64),
                target=np.array([0.0, 0.0, 0.08], dtype=np.float64),
                camera_prim_path="/OmniverseKit_Persp",
            )

        status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
        if not status:
            raise RuntimeError("Failed to create Isaac Sim URDF import config")
        import_config.merge_fixed_joints = False
        import_config.import_inertia_tensor = True
        import_config.fix_base = True
        import_config.make_default_prim = True
        import_config.create_physics_scene = False
        import_config.default_drive_type = UrdfJointTargetType.JOINT_DRIVE_NONE

        status, articulation_prim_path = omni.kit.commands.execute(
            "URDFParseAndImportFile",
            urdf_path=str(resolved_urdf_path),
            import_config=import_config,
            get_articulation_root=True,
        )
        if not status:
            raise RuntimeError(f"Failed to import URDF from {resolved_urdf_path}")

        robot_prim_path = f"/{robot_name}"
        if not stage.GetPrimAtPath(robot_prim_path).IsValid():
            raise RuntimeError(f"Imported robot root prim not found at {robot_prim_path}")

        print(f"Imported URDF root: {robot_prim_path}", flush=True)
        debug_print_imported_structure(stage, robot_prim_path)
        print(f"Importer articulation prim: {articulation_prim_path}", flush=True)

        controller = BalanceController(
            ControllerConfig(
                ball_pos_ref=args.ball_pos_ref,
                control_sign=args.control_sign,
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
        velocity_controller = VelocityCommandController(
            VelocityCommandConfig(
                track_vel_kp=args.track_vel_kp,
                track_vel_kd=args.track_vel_kd,
                command_alpha=args.command_alpha,
                max_cmd_step=args.max_cmd_step,
                max_track_velocity=args.max_track_velocity,
            )
        )

        print("Starting timeline before articulation initialization", flush=True)
        world.play()
        for _ in range(5):
            world.step(render=not args.headless)
        from omni.isaac.dynamic_control import _dynamic_control

        dc = _dynamic_control.acquire_dynamic_control_interface()
        articulation = _dynamic_control.INVALID_HANDLE
        articulation_prim_path = f"{robot_prim_path}/base_link"
        for _ in range(120):
            articulation = dc.get_articulation(articulation_prim_path)
            if articulation != _dynamic_control.INVALID_HANDLE:
                break
            world.step(render=not args.headless)
        if articulation == _dynamic_control.INVALID_HANDLE:
            raise RuntimeError(f"Failed to acquire articulation handle for {articulation_prim_path}")
        print(f"Bound dynamic_control articulation: {articulation_prim_path}", flush=True)

        dof_names: list[str] = []
        for dof_index in range(dc.get_articulation_dof_count(articulation)):
            dof_handle = dc.get_articulation_dof(articulation, dof_index)
            dof_names.append(dc.get_dof_name(dof_handle))
        print(f"Detected articulation joints: {dof_names}", flush=True)

        joint_track_dof = dc.find_articulation_dof(articulation, "joint_track")
        joint_ball_dof = dc.find_articulation_dof(articulation, "joint_ball")
        if joint_track_dof == _dynamic_control.INVALID_HANDLE or joint_ball_dof == _dynamic_control.INVALID_HANDLE:
            raise RuntimeError("Failed to resolve joint_track/joint_ball DOFs from imported articulation")

        track_props = dc.get_dof_properties(joint_track_dof)
        track_props.stiffness = args.track_drive_stiffness
        track_props.damping = args.track_drive_damping
        track_props.drive_mode = _dynamic_control.DRIVE_FORCE
        dc.set_dof_properties(joint_track_dof, track_props)

        articulation_api = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(articulation_prim_path))
        articulation_api.CreateSolverPositionIterationCountAttr(64)
        articulation_api.CreateSolverVelocityIterationCountAttr(64)

        ball_body = PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(f"{robot_prim_path}/ball"))
        ball_body.CreateLinearDampingAttr().Set(0.18)
        ball_body.CreateAngularDampingAttr().Set(1.20)

        track_drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(f"{robot_prim_path}/joints/joint_track"), "angular")
        track_drive.CreateStiffnessAttr(0.0)
        track_drive.CreateDampingAttr(0.0)

        spawn_offset = clamp(args.spawn_offset, -0.045, 0.045)
        reset_cooldown_steps = int(max(1, round(0.35 / args.physics_dt)))
        reset_cooldown = 0

        def reset_scene() -> None:
            nonlocal reset_cooldown
            controller.reset()
            beam_actuator.reset()
            velocity_controller.reset()
            dc.wake_up_articulation(articulation)
            dc.set_dof_state(joint_track_dof, _dynamic_control.DofState(0.0, 0.0, 0.0), _dynamic_control.STATE_ALL)
            dc.set_dof_velocity_target(joint_track_dof, 0.0)
            dc.set_dof_state(
                joint_ball_dof,
                _dynamic_control.DofState(spawn_offset, 0.0, 0.0),
                _dynamic_control.STATE_ALL,
            )
            dc.set_dof_position(joint_ball_dof, spawn_offset)
            dc.set_dof_velocity(joint_ball_dof, 0.0)
            reset_cooldown = reset_cooldown_steps

        reset_scene()

        steps = 0
        elapsed = 0.0
        debug_timer = 0.0

        while True:
            if not args.headless and not simulation_app.is_running():
                break

            track_angle = float(dc.get_dof_position(joint_track_dof))
            track_rate = float(dc.get_dof_velocity(joint_track_dof))
            ball_pos = float(dc.get_dof_position(joint_ball_dof))
            ball_vel = float(dc.get_dof_velocity(joint_ball_dof))

            desired_angle, commanded_angle = controller.update(
                dt=args.physics_dt,
                ball_pos=ball_pos,
                ball_vel=ball_vel,
                track_angle=track_angle,
                track_rate=track_rate,
            )

            beam_angle, _ = beam_actuator.update(desired_angle=commanded_angle, dt=args.physics_dt)
            beam_rate = velocity_controller.update(
                target_angle=beam_angle,
                track_angle=track_angle,
                track_rate=track_rate,
            )
            dc.wake_up_articulation(articulation)
            dc.set_dof_velocity_target(joint_track_dof, beam_rate)

            world.step(render=not args.headless)

            steps += 1
            elapsed += args.physics_dt
            debug_timer += args.physics_dt
            if reset_cooldown > 0:
                reset_cooldown -= 1

            if debug_timer >= args.debug_interval:
                debug_timer = 0.0
                print(
                    "t=%.2f ball_pos=%.4f ball_vel=%.4f desired=%.4f cmd=%.4f beam=%.4f beam_rate=%.4f"
                    % (
                        elapsed,
                        ball_pos,
                        ball_vel,
                        desired_angle,
                        commanded_angle,
                        beam_angle,
                        beam_rate,
                    ),
                    flush=True,
                )

            if reset_cooldown == 0 and abs(ball_pos) > args.reset_threshold:
                print("reset: ball reached prismatic travel limit", flush=True)
                reset_scene()
                world.play()

            if args.max_steps > 0 and steps >= args.max_steps:
                break

    finally:
        simulation_app.close()
        try:
            resolved_urdf_path.unlink(missing_ok=True)
        except OSError:
            pass


if __name__ == "__main__":
    main()
