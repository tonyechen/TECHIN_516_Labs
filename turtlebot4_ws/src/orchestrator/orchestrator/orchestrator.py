import numpy as np
import rclpy
import rclpy.time
from enum import Enum, auto
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import String

from nav2_msgs.action import NavigateToPose, Spin

from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import tf2_geometry_msgs  # noqa: F401


class State(Enum):
    INITIALIZING = auto()
    EXPLORING    = auto()
    SPINNING     = auto()   # spin after seeing marker, before returning
    RETURNING    = auto()
    SPINNING_2   = auto()   # spin after returning, before navigating to cube
    NAVIGATING   = auto()
    DONE         = auto()


class Orchestrator(Node):

    ARUCO_APPROACH_OFFSET = 0.10  # metres — stop this far in front of the cube
    MARKER_CONFIRM_COUNT  = 3     # consecutive frames before accepting marker

    # How long to wait for TF tree to be fully connected before attempting spin
    TF_SPIN_WAIT_SEC      = 1.0
    SPIN_RETRY_MAX        = 3

    def __init__(self):
        super().__init__('orchestrator')

        # TF2
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State
        self.state             = State.INITIALIZING
        self.start_pose        = None   # robot pose in map frame at startup
        self.cube_pose         = None   # approach point in map frame
        self.cube_pose_frame   = 'map'

        self._marker_candidate       = None
        self._marker_confirm_counter = 0
        self._nav2_in_progress       = False

        # Spin retry tracking
        self._spin_retry_count   = 0
        self._spin_pending        = False  # True while waiting for spin result

        # Nav2 action client
        self._nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._spin_client = ActionClient(self, Spin, 'spin')

        # Publishers
        self._annotator_pub = self.create_publisher(PoseStamped, '/map_annotator/save_pose', 10)
        self.state_pub      = self.create_publisher(String,      '/orchestrator/state',      10)

        # Subscribers
        self.create_subscription(PoseArray, '/aruco_detections/poses', self._aruco_cb, 10)

        self.create_timer(0.1, self._control_loop)

        self.get_logger().info('Orchestrator ready — INITIALIZING (waiting for TF)')

    # ── TF2 robot pose lookup ──────────────────────────────────────────────────

    def _get_robot_pose(self, frame='map'):
        """Look up current robot pose in the given frame via TF2. Non-blocking."""
        try:
            t = self.tf_buffer.lookup_transform(
                frame, 'base_link', rclpy.time.Time(),
                timeout=Duration(seconds=0.0)
            )
            ps = PoseStamped()
            ps.header.frame_id    = frame
            ps.header.stamp       = self.get_clock().now().to_msg()
            ps.pose.position.x    = t.transform.translation.x
            ps.pose.position.y    = t.transform.translation.y
            ps.pose.position.z    = t.transform.translation.z
            ps.pose.orientation   = t.transform.rotation
            return ps.pose
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'Could not get robot pose in {frame}: {e}',
                                   throttle_duration_sec=2.0)
            return None

    def _best_frame(self, source_frame):
        """Return map if the full chain exists, else odom."""
        if self.tf_buffer.can_transform('map', source_frame, rclpy.time.Time()):
            return 'map'
        return 'odom'

    def _tf_tree_healthy(self):
        """
        Check that odom→base_link and map→odom are both available.
        This is the same check the behavior_server does before running spin.
        """
        return (
            self.tf_buffer.can_transform('odom', 'base_link', rclpy.time.Time()) and
            self.tf_buffer.can_transform('map',  'odom',      rclpy.time.Time())
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _aruco_cb(self, msg: PoseArray):
        """Process detections during EXPLORING at any distance."""
        if self.state != State.EXPLORING:
            return
        if not msg.poses:
            self._marker_confirm_counter = 0
            return

        z_dist = msg.poses[0].position.z
        self._marker_candidate = msg
        self._marker_confirm_counter += 1
        self.get_logger().info(
            f'Marker detected at z={z_dist:.3f}m '
            f'({self._marker_confirm_counter}/{self.MARKER_CONFIRM_COUNT})',
            throttle_duration_sec=0.5
        )

        if self._marker_confirm_counter >= self.MARKER_CONFIRM_COUNT:
            self._on_marker_confirmed()

    # ── Control loop ──────────────────────────────────────────────────────────

    def _control_loop(self):
        self._publish_state()

        if self.state == State.INITIALIZING:
            if not self.tf_buffer.can_transform('map', 'odom', rclpy.time.Time()):
                self.get_logger().info('Waiting for map frame...', throttle_duration_sec=2.0)
                return
            pose = self._get_robot_pose('map')
            if pose is not None:
                self.start_pose = pose
                self._save_pose_to_annotator('start', self.start_pose)
                self.get_logger().info(
                    f'Start pose saved in map: '
                    f'x={pose.position.x:.2f} y={pose.position.y:.2f}'
                )
                self._transition(State.EXPLORING)

        elif self.state == State.EXPLORING:
            pass  # wall follower and aruco detector handle this via topic gates

        elif self.state == State.SPINNING:
            # Spin after marker confirmed, before returning to start
            if not self._spin_pending:
                self._try_spin(next_state=State.RETURNING)

        elif self.state == State.RETURNING:
            if not self._nav2_in_progress and self.start_pose is not None:
                self._send_nav2_goal(self.start_pose, 'map',
                                     on_complete=self._on_returned_to_start)

        elif self.state == State.SPINNING_2:
            # Spin after returning to start, before navigating to cube
            if not self._spin_pending:
                self._try_spin(next_state=State.NAVIGATING)

        elif self.state == State.NAVIGATING:
            if not self._nav2_in_progress and self.cube_pose is not None:
                self._send_nav2_goal(self.cube_pose, self.cube_pose_frame,
                                     on_complete=self._on_reached_cube)

    # ── Marker confirmed ──────────────────────────────────────────────────────

    def _on_marker_confirmed(self):
        """Transform cube pose to map/odom frame, compute approach point."""
        raw_pose = self._marker_candidate.poses[0]
        frame_id = self._marker_candidate.header.frame_id or 'oakd_right_camera_optical_frame'
        target_frame = self._best_frame(frame_id)

        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.header.stamp    = rclpy.time.Time().to_msg()
        ps.pose            = raw_pose

        try:
            transformed      = self.tf_buffer.transform(ps, target_frame,
                                                        timeout=Duration(seconds=0.0))
            cube_pose_world  = transformed.pose

            robot_pose = self._get_robot_pose(target_frame)
            if robot_pose is not None:
                dx   = robot_pose.position.x - cube_pose_world.position.x
                dy   = robot_pose.position.y - cube_pose_world.position.y
                dist = np.hypot(dx, dy)
                if dist > 0:
                    offset_x = (dx / dist) * self.ARUCO_APPROACH_OFFSET
                    offset_y = (dy / dist) * self.ARUCO_APPROACH_OFFSET
                else:
                    offset_x, offset_y = self.ARUCO_APPROACH_OFFSET, 0.0
            else:
                offset_x, offset_y = self.ARUCO_APPROACH_OFFSET, 0.0

            approach_pose            = cube_pose_world
            approach_pose.position.x += offset_x
            approach_pose.position.y += offset_y

            self.cube_pose       = approach_pose
            self.cube_pose_frame = target_frame

            self.get_logger().info(
                f'Cube in {target_frame}: '
                f'x={cube_pose_world.position.x:.2f} y={cube_pose_world.position.y:.2f} — '
                f'approach: x={approach_pose.position.x:.2f} y={approach_pose.position.y:.2f}'
            )

            self._save_pose_to_annotator('aruco_cube',     cube_pose_world)
            self._save_pose_to_annotator('aruco_approach', self.cube_pose)

            # Reset spin state and enter SPINNING state
            self._spin_retry_count = 0
            self._spin_pending     = False
            self._transition(State.SPINNING)

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'TF transform failed: {e} — retrying next detection')

    # ── Spin then return ──────────────────────────────────────────────────────

    def _try_spin(self, next_state: State):
        """
        Attempt to send a 180° spin goal. Guards against a broken TF tree
        by checking connectivity first and retrying up to SPIN_RETRY_MAX times.
        """
        if self._spin_retry_count >= self.SPIN_RETRY_MAX:
            self.get_logger().warn(
                f'Spin failed {self.SPIN_RETRY_MAX} times — skipping spin, going to {next_state.name}'
            )
            self._transition(next_state)
            return

        # Guard: make sure the behavior_server can actually find base_link
        if not self._tf_tree_healthy():
            self.get_logger().warn(
                'TF tree not healthy (odom↔base_link missing) — '
                f'waiting before spin attempt {self._spin_retry_count + 1}/{self.SPIN_RETRY_MAX}',
                throttle_duration_sec=0.5
            )
            return  # will retry next control loop tick

        if not self._spin_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn('Spin action server not available yet — retrying...',
                                   throttle_duration_sec=1.0)
            return

        self._spin_retry_count += 1
        self._spin_pending      = True
        self._spin_next_state   = next_state  # store where to go after spin

        goal = Spin.Goal()
        goal.target_yaw = 3.14159  # 180 degrees
        self.get_logger().info(
            f'Spinning 180° before {next_state.name} '
            f'(attempt {self._spin_retry_count}/{self.SPIN_RETRY_MAX})...'
        )
        future = self._spin_client.send_goal_async(goal)
        future.add_done_callback(self._on_spin_goal_accepted)

    def _on_spin_goal_accepted(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Spin goal rejected — will retry')
            self._spin_pending = False  # allow _control_loop to retry
            return

        self.get_logger().info('Spin goal accepted — executing...')
        goal_handle.get_result_async().add_done_callback(self._on_spin_result)

    def _on_spin_result(self, future):
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Spin completed successfully — transitioning to {self._spin_next_state.name}')
            self._transition(self._spin_next_state)
        else:
            self.get_logger().warn(
                f'Spin ended with non-success status {status} — '
                f'retry {self._spin_retry_count}/{self.SPIN_RETRY_MAX}'
            )
            self._spin_pending = False  # allow _control_loop to retry via _try_spin

    # ── Nav2 ──────────────────────────────────────────────────────────────────

    def _send_nav2_goal(self, target_pose, frame: str, on_complete):
        if not self._nav2_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Nav2 not available yet, retrying...')
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = frame
        goal.pose.header.stamp    = self.get_clock().now().to_msg()
        goal.pose.pose            = target_pose

        self.get_logger().info(
            f'Nav2 goal ({frame}) → '
            f'x={target_pose.position.x:.2f} y={target_pose.position.y:.2f}'
        )

        self._nav2_in_progress = True
        future = self._nav2_client.send_goal_async(goal)
        future.add_done_callback(
            lambda f: self._on_nav2_goal_accepted(f, on_complete)
        )

    def _on_nav2_goal_accepted(self, future, on_complete):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 goal rejected')
            self._nav2_in_progress = False
            return
        self.get_logger().info('Nav2 goal accepted — navigating...')
        goal_handle.get_result_async().add_done_callback(
            lambda f: self._on_nav2_result(f, on_complete)
        )

    def _on_nav2_result(self, future, on_complete):
        self._nav2_in_progress = False
        self.get_logger().info(f'Nav2 finished with status: {future.result().status}')
        on_complete()

    def _on_returned_to_start(self):
        self.get_logger().info('Returned to start — spinning before navigating to cube')
        # Reset spin state for the second spin
        self._spin_retry_count = 0
        self._spin_pending     = False
        self._transition(State.SPINNING_2)

    def _on_reached_cube(self):
        self.get_logger().info('Reached cube — mission complete!')
        self._transition(State.DONE)

    # ── Map annotator ─────────────────────────────────────────────────────────

    def _save_pose_to_annotator(self, name: str, pose):
        ps = PoseStamped()
        ps.header.frame_id = name
        ps.header.stamp    = self.get_clock().now().to_msg()
        ps.pose            = pose
        self._annotator_pub.publish(ps)
        self.get_logger().info(f"Saved pose '{name}' to map annotator")

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _transition(self, new_state: State):
        self.get_logger().info(f'{self.state.name} → {new_state.name}')
        self.state             = new_state
        self._nav2_in_progress = False

    def _publish_state(self):
        msg      = String()
        msg.data = self.state.name
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Orchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()