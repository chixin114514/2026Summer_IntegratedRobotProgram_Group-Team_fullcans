"""One-object fixed-point pick-and-place Action server for Task3."""

from __future__ import annotations

import json
import math
import threading
import time
from collections import defaultdict, deque
from pathlib import Path
from typing import Deque, Dict, Iterable, Optional, Tuple

import yaml

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage

from task3_interfaces.action import SortObject

from .motion_plan import MotionStep, build_sequence, validate_goal
from .pose_evidence import (
    MAX_POSE_AGE_S,
    estimate_speed,
    extract_model_name,
    frame_priority,
    is_fresh,
    placement_is_valid,
)


ARM_COMMAND_TOPICS = tuple(
    f"/task3/arm/joint{index}/cmd_pos" for index in range(1, 7)
)
GRIPPER_COMMAND_NAMES = (
    "left3",
    "left2",
    "left1",
    "right3",
    "right2",
    "right1",
)
GRIPPER_COMMAND_TOPICS = tuple(
    f"/task3/gripper/{name}_cmd_pos" for name in GRIPPER_COMMAND_NAMES
)
POSE_TOPIC = "/task3/gazebo/pose/info"
ACTION_NAME = "/task3/sort_object"
POSE_SAMPLE = Tuple[float, float, float, float]


def _load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"configuration must be a mapping: {path}")
    return data


def _smoothstep(start: float, target: float, fraction: float) -> float:
    fraction = max(0.0, min(1.0, fraction))
    eased = fraction * fraction * (3.0 - 2.0 * fraction)
    return start + (target - start) * eased


class PickSortServer(Node):
    """ROS 2 Action server that only commands measured fixed joint poses."""

    def __init__(self) -> None:
        super().__init__("pick_sort_server")
        self.declare_parameter("motion_config", "")
        self.declare_parameter("scene_config", "")
        self.declare_parameter("log_path", "")

        motion_path = Path(str(self.get_parameter("motion_config").value))
        scene_path = Path(str(self.get_parameter("scene_config").value))
        if not motion_path.is_file():
            raise FileNotFoundError(f"motion config not found: {motion_path}")
        if not scene_path.is_file():
            raise FileNotFoundError(f"scene config not found: {scene_path}")
        self.motion_config = _load_yaml(motion_path)
        self.scene_config = _load_yaml(scene_path)

        self._grid_centres = {
            str(name): (float(values[0]), float(values[1]))
            for name, values in self.scene_config.get("grids", {}).items()
        }
        self._bin_centres = {
            str(name): (float(values[0]), float(values[1]))
            for name, values in self.scene_config.get("bins", {}).items()
        }
        self._home = tuple(float(value) for value in self.motion_config["home"])
        self._open = float(self.motion_config["gripper"]["open_rad"])
        self._closed = float(self.motion_config["gripper"]["closed_rad"])
        self._rate_hz = float(self.motion_config["motion"]["command_rate_hz"])
        if len(self._home) != 6 or self._rate_hz <= 0.0:
            raise ValueError("motion configuration has an invalid home or command rate")

        self._arm_publishers = tuple(
            self.create_publisher(Float64, topic, 10)
            for topic in ARM_COMMAND_TOPICS
        )
        self._gripper_publishers = tuple(
            self.create_publisher(Float64, topic, 10)
            for topic in GRIPPER_COMMAND_TOPICS
        )

        self._callback_group = ReentrantCallbackGroup()
        self._pose_lock = threading.Lock()
        self._goal_lock = threading.Lock()
        self._poses: Dict[str, POSE_SAMPLE] = {}
        self._pose_history: Dict[str, Deque[POSE_SAMPLE]] = defaultdict(
            lambda: deque(maxlen=240)
        )
        self._last_arm = self._home
        self._last_gripper = self._open
        self._active_goal = None
        self._goal_reserved = False
        self._ready = False

        self._log_path = self._resolve_log_path()
        self._pose_subscription = self.create_subscription(
            TFMessage,
            POSE_TOPIC,
            self._pose_callback,
            10,
            callback_group=self._callback_group,
        )
        self._action_server = ActionServer(
            self,
            SortObject,
            ACTION_NAME,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            handle_accepted_callback=self._handle_accepted_callback,
            callback_group=self._callback_group,
        )

        init_ticks = max(5, int(round(self._rate_hz * 0.75)))
        self._initialisation_ticks_left = init_ticks
        self._initialisation_timer = self.create_timer(
            1.0 / self._rate_hz,
            self._initialisation_tick,
            callback_group=self._callback_group,
        )
        self._log("startup", ready=False, init_ticks=init_ticks)

    def _resolve_log_path(self) -> Path:
        configured = str(self.get_parameter("log_path").value or "").strip()
        path = Path(configured) if configured else Path.home() / ".ros" / "task3_sort_log.jsonl"
        path.parent.mkdir(parents=True, exist_ok=True)
        return path

    def _log(self, event: str, **fields) -> None:
        record = {"time": time.time(), "event": event, **fields}
        try:
            with self._log_path.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(record, ensure_ascii=False) + "\n")
        except OSError as exc:
            self.get_logger().warning(f"could not append Task3 log: {exc}")

    def _initialisation_tick(self) -> None:
        self._publish_gripper(self._open)
        self._publish_arm(self._home)
        self._initialisation_ticks_left -= 1
        if self._initialisation_ticks_left > 0:
            return
        self._last_arm = self._home
        self._last_gripper = self._open
        self._ready = True
        self.destroy_timer(self._initialisation_timer)
        self._log("startup_complete", ready=True)
        self.get_logger().info("Task3 pick_sort_server ready")

    def _pose_callback(self, message: TFMessage) -> None:
        now = time.monotonic()
        selected = {}
        for transform in message.transforms:
            frame_id = getattr(transform, "child_frame_id", "")
            name = extract_model_name(frame_id)
            if not name:
                continue
            priority = frame_priority(frame_id, name)
            translation = transform.transform.translation
            sample = (
                now,
                float(translation.x),
                float(translation.y),
                float(translation.z),
            )
            current = selected.get(name)
            if current is None or priority < current[0]:
                selected[name] = (priority, sample)
        with self._pose_lock:
            for name, (_, sample) in selected.items():
                self._poses[name] = sample
                self._pose_history[name].append(sample)

    def _goal_callback(self, request) -> GoalResponse:
        with self._goal_lock:
            if not self._ready:
                self._log("goal_rejected", reason="initialisation_incomplete")
                return GoalResponse.REJECT
            if self._active_goal is not None or self._goal_reserved:
                self._log("goal_rejected", reason="another_goal_is_active")
                return GoalResponse.REJECT
            try:
                validate_goal(request.grid_id, request.bin_id, self.motion_config)
            except (KeyError, TypeError, ValueError) as exc:
                self._log(
                    "goal_rejected",
                    reason="invalid_goal",
                    grid_id=str(request.grid_id),
                    bin_id=str(request.bin_id),
                    message=str(exc),
                )
                self._safe_reset("invalid_goal")
                return GoalResponse.REJECT
            self._goal_reserved = True
            return GoalResponse.ACCEPT

    def _handle_accepted_callback(self, goal_handle) -> None:
        with self._goal_lock:
            self._active_goal = goal_handle
            self._goal_reserved = True
        goal_handle.execute()

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _publish_arm(self, arm_deg: Iterable[float]) -> None:
        values = tuple(float(value) for value in arm_deg)
        if len(values) != 6:
            raise ValueError("an arm command must contain six joints")
        for publisher, value in zip(self._arm_publishers, values):
            message = Float64()
            message.data = math.radians(value)
            publisher.publish(message)

    def _publish_gripper(self, gripper_rad: float) -> None:
        value = float(gripper_rad)
        # The six linkage joints use mirrored signs around the common command.
        values = (value, value, -value, -value, -value, value)
        for publisher, joint_value in zip(self._gripper_publishers, values):
            message = Float64()
            message.data = joint_value
            publisher.publish(message)

    def _publish_feedback(self, goal_handle, stage: str, progress: float) -> None:
        feedback = SortObject.Feedback()
        feedback.stage = str(stage)
        feedback.progress = float(max(0.0, min(1.0, progress)))
        goal_handle.publish_feedback(feedback)

    def _run_step(
        self,
        goal_handle,
        step: MotionStep,
        index: int,
        total: int,
    ) -> bool:
        start_arm = self._last_arm
        start_gripper = self._last_gripper
        ticks = max(1, int(round(step.duration_s * self._rate_hz)))
        for tick in range(1, ticks + 1):
            if goal_handle.is_cancel_requested:
                return False
            fraction = tick / ticks
            arm = tuple(
                _smoothstep(start, target, fraction)
                for start, target in zip(start_arm, step.arm_deg)
            )
            gripper = _smoothstep(start_gripper, step.gripper_rad, fraction)
            self._publish_arm(arm)
            self._publish_gripper(gripper)
            self._publish_feedback(
                goal_handle,
                step.stage,
                (index + fraction) / total,
            )
            self._last_arm = tuple(arm)
            self._last_gripper = float(gripper)
            time.sleep(1.0 / self._rate_hz)
        self._last_arm = tuple(float(value) for value in step.arm_deg)
        self._last_gripper = float(step.gripper_rad)
        self._publish_feedback(goal_handle, step.stage, (index + 1.0) / total)
        self._log("stage", stage=step.stage, progress=(index + 1.0) / total)
        return True

    def _run_settle(self, goal_handle, index: int, total: int) -> bool:
        duration = float(self.motion_config["motion"]["durations_s"]["settle"])
        ticks = max(1, int(round(duration * self._rate_hz)))
        for tick in range(1, ticks + 1):
            if goal_handle.is_cancel_requested:
                return False
            self._publish_arm(self._last_arm)
            self._publish_gripper(self._open)
            progress = (index + 1.0) / total
            self._publish_feedback(goal_handle, "SETTLE", progress)
            self._last_gripper = self._open
            time.sleep(1.0 / self._rate_hz)
        self._last_gripper = self._open
        self._log("stage", stage="SETTLE", progress=(index + 1.0) / total)
        return True

    def _nearest_object(
        self,
        grid_id: str,
        now: Optional[float] = None,
    ) -> Optional[Tuple[str, POSE_SAMPLE]]:
        centre = self._grid_centres.get(grid_id)
        if centre is None:
            return None
        current_time = time.monotonic() if now is None else float(now)
        candidates = []
        with self._pose_lock:
            for name, sample in self._poses.items():
                if not is_fresh(sample, current_time, MAX_POSE_AGE_S):
                    continue
                distance = math.hypot(sample[1] - centre[0], sample[2] - centre[1])
                if distance <= 0.06:
                    candidates.append((distance, name, sample))
        if not candidates:
            return None
        _, name, sample = min(candidates, key=lambda item: item[0])
        return name, sample

    def _current_pose(
        self,
        object_name: str,
        now: Optional[float] = None,
    ) -> Optional[POSE_SAMPLE]:
        current_time = time.monotonic() if now is None else float(now)
        with self._pose_lock:
            sample = self._poses.get(object_name)
        if not is_fresh(sample, current_time, MAX_POSE_AGE_S):
            return None
        return sample

    def _lift_verified(
        self,
        object_name: str,
        initial_z: float,
        started_at: float,
    ) -> bool:
        now = time.monotonic()
        with self._pose_lock:
            history = tuple(
                sample
                for sample in self._pose_history.get(object_name, ())
                if sample[0] >= started_at
                and is_fresh(sample, now, MAX_POSE_AGE_S)
            )
        if not history or not is_fresh(history[-1], now, MAX_POSE_AGE_S):
            return False
        maximum_z = max(sample[3] for sample in history)
        rise = maximum_z - initial_z
        self._log("lift_check", object_name=object_name, rise_m=rise, threshold_m=0.025)
        return rise >= 0.025

    def _place_verified(self, object_name: str, bin_id: str) -> bool:
        now = time.monotonic()
        pose = self._current_pose(object_name, now)
        centre = self._bin_centres.get(bin_id)
        if pose is None or centre is None:
            return False
        distance = math.hypot(pose[1] - centre[0], pose[2] - centre[1])
        with self._pose_lock:
            samples = tuple(self._pose_history.get(object_name, ()))
        speed = estimate_speed(samples, now, MAX_POSE_AGE_S)
        valid = placement_is_valid(
            pose,
            centre,
            samples,
            now,
            max_age_s=MAX_POSE_AGE_S,
        )
        self._log(
            "place_check",
            object_name=object_name,
            bin_id=bin_id,
            distance_m=distance,
            z_m=pose[3],
            speed_m_s=speed,
        )
        return valid

    def _safe_reset(self, reason: str) -> None:
        duration = float(self.motion_config["motion"]["durations_s"]["home"])
        ticks = max(1, int(round(duration * self._rate_hz)))
        start_arm = tuple(float(value) for value in self._last_arm)
        self._publish_gripper(self._open)
        for tick in range(1, ticks + 1):
            fraction = tick / ticks
            arm = tuple(
                _smoothstep(start, target, fraction)
                for start, target in zip(start_arm, self._home)
            )
            self._publish_gripper(self._open)
            self._publish_arm(arm)
            self._last_arm = arm
            self._last_gripper = self._open
            time.sleep(1.0 / self._rate_hz)
        self._last_arm = tuple(self._home)
        self._last_gripper = self._open
        self._log("safe_reset", reason=reason, home_commanded=True)

    def _failure_result(
        self,
        goal_handle,
        result,
        object_name: str,
        message: str,
        pick_verified: bool = False,
        cancelled: bool = False,
    ):
        self._safe_reset(message)
        result.success = False
        result.pick_verified = bool(pick_verified)
        result.place_verified = False
        result.object_name = object_name
        result.message = message
        self._log("result", success=False, message=message, object_name=object_name)
        if cancelled:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result

    def _execute_callback(self, goal_handle):
        request = goal_handle.request
        result = SortObject.Result()
        object_name = ""
        pick_verified = False
        try:
            validate_goal(request.grid_id, request.bin_id, self.motion_config)
            nearest = self._nearest_object(request.grid_id)
            if nearest is None:
                return self._failure_result(
                    goal_handle,
                    result,
                    object_name,
                    f"no executable object near {request.grid_id}",
                )
            object_name, initial_sample = nearest
            initial_z = initial_sample[3]
            execution_started = time.monotonic()
            self._log(
                "goal_started",
                grid_id=request.grid_id,
                bin_id=request.bin_id,
                object_name=object_name,
            )
            sequence = build_sequence(request.grid_id, request.bin_id, self.motion_config)
            total = len(sequence)
            for index, step in enumerate(sequence):
                if not self._run_step(goal_handle, step, index, total):
                    return self._failure_result(
                        goal_handle,
                        result,
                        object_name,
                        "goal_cancelled" if goal_handle.is_cancel_requested else "motion_failed",
                        pick_verified=pick_verified,
                        cancelled=goal_handle.is_cancel_requested,
                    )
                if step.stage == "LIFT":
                    pick_verified = self._lift_verified(
                        object_name,
                        initial_z,
                        execution_started,
                    )
                    if not pick_verified:
                        return self._failure_result(
                            goal_handle,
                            result,
                            object_name,
                            "measured lift below 0.025 m",
                        )
                if step.stage == "RELEASE":
                    if not self._run_settle(goal_handle, index, total):
                        return self._failure_result(
                            goal_handle,
                            result,
                            object_name,
                            "goal_cancelled" if goal_handle.is_cancel_requested else "settle_failed",
                            pick_verified=pick_verified,
                            cancelled=goal_handle.is_cancel_requested,
                        )

            if goal_handle.is_cancel_requested:
                return self._failure_result(
                    goal_handle,
                    result,
                    object_name,
                    "goal_cancelled_after_motion",
                    pick_verified=pick_verified,
                    cancelled=True,
                )

            place_verified = self._place_verified(object_name, request.bin_id)
            if not place_verified:
                return self._failure_result(
                    goal_handle,
                    result,
                    object_name,
                    "measured placement did not settle in the requested bin",
                    pick_verified=pick_verified,
                )

            if goal_handle.is_cancel_requested:
                return self._failure_result(
                    goal_handle,
                    result,
                    object_name,
                    "goal_cancelled_after_place_check",
                    pick_verified=pick_verified,
                    cancelled=True,
                )

            result.success = True
            result.pick_verified = pick_verified
            result.place_verified = place_verified
            result.object_name = object_name
            result.message = "physical lift and settled placement verified"
            goal_handle.succeed()
            self._log(
                "result",
                success=True,
                object_name=object_name,
                pick_verified=pick_verified,
                place_verified=place_verified,
            )
            return result
        except (KeyError, TypeError, ValueError) as exc:
            return self._failure_result(
                goal_handle,
                result,
                object_name,
                f"invalid motion configuration or goal: {exc}",
                pick_verified=pick_verified,
            )
        except Exception as exc:  # Keep every runtime failure fail-closed.
            self.get_logger().error(f"Task3 motion failed: {exc}")
            return self._failure_result(
                goal_handle,
                result,
                object_name,
                f"motion exception: {exc}",
                pick_verified=pick_verified,
            )
        finally:
            with self._goal_lock:
                self._active_goal = None
                self._goal_reserved = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PickSortServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
