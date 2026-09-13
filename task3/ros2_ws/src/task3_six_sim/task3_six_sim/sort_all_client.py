"""Send the six fixed pickup goals to their colour-specific landing slots."""

import os
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from task3_interfaces.action import SortObject


ACTION_NAME = "/task3_six/sort_object"
# Optional grid-id filter for tuning, e.g. ``TASK3_SIX_JOBS=P1,P2`` runs only
# those jobs; with the variable unset all six run in order.
JOBS_ENV = "TASK3_SIX_JOBS"
SORT_JOBS = (
    ("P1", "YELLOW_1", "yellow 1/3"),
    ("P2", "GREEN_1", "green 1/3"),
    ("P3", "YELLOW_2", "yellow 2/3"),
    ("P4", "GREEN_2", "green 2/3"),
    ("P5", "YELLOW_3", "yellow 3/3"),
    ("P6", "GREEN_3", "green 3/3"),
)


class SortAllClient(Node):
    def __init__(self) -> None:
        super().__init__("task3_six_sort_all")
        self._client = ActionClient(self, SortObject, ACTION_NAME)
        self._last_stage = ""
        self._wanted = {
            value.strip().upper()
            for value in os.environ.get(JOBS_ENV, "").split(",")
            if value.strip()
        }

    def _select_jobs(self):
        if not self._wanted:
            return SORT_JOBS
        jobs = tuple(job for job in SORT_JOBS if job[0].upper() in self._wanted)
        self.get_logger().info("restricted to " + ", ".join(job[0] for job in jobs))
        return jobs

    def _feedback(self, message) -> None:
        stage = str(message.feedback.stage)
        if stage != self._last_stage:
            self._last_stage = stage
            self.get_logger().info(f"stage: {stage}")

    def run(self) -> bool:
        jobs = self._select_jobs()
        self.get_logger().info(f"waiting for {ACTION_NAME}")
        if not self._client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error("six-object sort action server is unavailable")
            return False

        completed = 0
        failed = []
        for grid_id, bin_id, label in jobs:
            self._last_stage = ""
            goal = SortObject.Goal()
            goal.grid_id = grid_id
            goal.bin_id = bin_id
            self.get_logger().info(f"sorting {label}: {grid_id} -> {bin_id}")

            sent = self._client.send_goal_async(
                goal,
                feedback_callback=self._feedback,
            )
            rclpy.spin_until_future_complete(self, sent)
            handle = sent.result()
            if handle is None or not handle.accepted:
                # A rejected goal says the server is busy or not ready, which no
                # later goal can fix, so this is the one case that does stop.
                self.get_logger().error(f"goal rejected: {grid_id} -> {bin_id}")
                return False

            result_future = handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            response = result_future.result()
            result = None if response is None else response.result
            if result is None or not result.success:
                # One missed grasp must not abandon the remaining blocks: note it
                # and carry on, then report the whole run at the end.
                message = "no result" if result is None else result.message
                failed.append((grid_id, bin_id, message))
                self.get_logger().error(
                    f"sort failed at {grid_id} -> {bin_id}: {message} "
                    f"-- continuing with the remaining blocks"
                )
                continue
            completed += 1
            self.get_logger().info(
                f"completed {completed}/{len(jobs)}: {result.object_name} -> {bin_id}"
            )

        if failed:
            summary = "; ".join(
                f"{grid_id}->{bin_id}: {message}" for grid_id, bin_id, message in failed
            )
            self.get_logger().error(
                f"{len(failed)} of {len(jobs)} jobs failed: {summary}"
            )
            return False
        self.get_logger().info(
            "SORT COMPLETE: yellow region contains 3 yellow blocks; "
            "green region contains 3 green blocks"
        )
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SortAllClient()
    try:
        success = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not success:
        sys.exit(1)


if __name__ == "__main__":
    main()
