#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray
from tf2_ros import Buffer, TransformListener
import pandas as pd
import os
from threading import Thread

class MapAnnotator(Node):
    def __init__(self):
        super().__init__('map_annotator')

        # TF
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.goal_pub       = self.create_publisher(PoseStamped, '/goal_pose',   10)
        self.pose_array_pub = self.create_publisher(PoseArray,   '/saved_poses', 10)

        # ── Topic subscriber (used by orchestrator) ────────────────────────
        # Orchestrator publishes PoseStamped where header.frame_id = pose name
        self.create_subscription(
            PoseStamped,
            '/map_annotator/save_pose',
            self._handle_save_pose_topic,
            10
        )

        # Timer
        self.create_timer(1.0, self.publish_pose_array)

        # Storage
        self.poses    = {}           # {name: PoseStamped}
        self.csv_file = 'saved_poses.csv'
        self.load_poses()

        self.print_commands()

    # ── Topic handler ──────────────────────────────────────────────────────

    def _handle_save_pose_topic(self, msg: PoseStamped):
        """Called by orchestrator. Name is packed into header.frame_id."""
        name = msg.header.frame_id.strip()
        if not name or name == 'map':
            self.get_logger().warn('Received save_pose topic with no name in frame_id — ignoring')
            return
        # Restore frame_id to map before storing
        msg.header.frame_id = 'map'
        self.poses[name] = msg
        self.save_poses()
        self.get_logger().info(f"Topic saved pose '{name}': "
                               f"x={msg.pose.position.x:.2f} "
                               f"y={msg.pose.position.y:.2f}")

    # ── CSV persistence ────────────────────────────────────────────────────

    def load_poses(self):
        if os.path.exists(self.csv_file):
            try:
                df = pd.read_csv(self.csv_file)
                for _, row in df.iterrows():
                    ps = PoseStamped()
                    ps.header.frame_id      = 'map'
                    ps.pose.position.x      = row['px']
                    ps.pose.position.y      = row['py']
                    ps.pose.position.z      = row['pz']
                    ps.pose.orientation.x   = row['ox']
                    ps.pose.orientation.y   = row['oy']
                    ps.pose.orientation.z   = row['oz']
                    ps.pose.orientation.w   = row['ow']
                    self.poses[row['name']] = ps
                self.get_logger().info(f'Loaded {len(self.poses)} poses from {self.csv_file}')
            except Exception as e:
                self.get_logger().error(f'Error loading poses: {e}')

    def save_poses(self):
        if not self.poses:
            if os.path.exists(self.csv_file):
                os.remove(self.csv_file)
            return

        data = [
            {
                'name': name,
                'px': ps.pose.position.x,
                'py': ps.pose.position.y,
                'pz': ps.pose.position.z,
                'ox': ps.pose.orientation.x,
                'oy': ps.pose.orientation.y,
                'oz': ps.pose.orientation.z,
                'ow': ps.pose.orientation.w,
            }
            for name, ps in self.poses.items()
        ]
        pd.DataFrame(data).to_csv(self.csv_file, index=False)
        self.get_logger().info(f'Persisted {len(self.poses)} poses to {self.csv_file}')

    # ── TF pose lookup ─────────────────────────────────────────────────────

    def get_current_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            ps = PoseStamped()
            ps.header.frame_id     = 'map'
            ps.header.stamp        = self.get_clock().now().to_msg()
            ps.pose.position.x     = transform.transform.translation.x
            ps.pose.position.y     = transform.transform.translation.y
            ps.pose.position.z     = transform.transform.translation.z
            ps.pose.orientation    = transform.transform.rotation
            return ps
        except Exception as e:
            self.get_logger().error(f'Could not get current pose: {e}')
            return None

    # ── Pose array publisher ───────────────────────────────────────────────

    def publish_pose_array(self):
        if not self.poses:
            return
        pa = PoseArray()
        pa.header.frame_id = 'map'
        pa.header.stamp    = self.get_clock().now().to_msg()
        pa.poses           = [ps.pose for ps in self.poses.values()]
        self.pose_array_pub.publish(pa)

    # ── CLI commands ───────────────────────────────────────────────────────

    def print_commands(self):
        print('\ncommands:')
        print('  list')
        print('  save <name>    — save current robot pose')
        print('  delete <name>')
        print('  goto <name>')
        print('  exit')
        print('  help\n')

    def cmd_list(self):
        if not self.poses:
            print('No poses saved')
        else:
            print('Saved poses:')
            for name, ps in self.poses.items():
                print(f'  {name}: '
                      f'x={ps.pose.position.x:.2f} '
                      f'y={ps.pose.position.y:.2f}')

    def cmd_save(self, name):
        if not name:
            print('Error: provide a name')
            return
        ps = self.get_current_pose()
        if ps is None:
            print('Error: could not get current pose')
            return
        self.poses[name] = ps
        self.save_poses()
        print(f"Saved pose '{name}'")

    def cmd_delete(self, name):
        if not name:
            print('Error: provide a name')
            return
        if name in self.poses:
            del self.poses[name]
            self.save_poses()
            print(f"Deleted '{name}'")
        else:
            print(f"'{name}' not found")

    def cmd_goto(self, name):
        if not name:
            print('Error: provide a name')
            return
        if name not in self.poses:
            print(f"'{name}' not found")
            return
        ps = self.poses[name]
        ps.header.stamp = self.get_clock().now().to_msg()
        self.goal_pub.publish(ps)
        print(f"Navigating to '{name}'")

    def process_command(self, command: str) -> bool:
        command = command.strip()
        if not command:
            return True
        parts = command.split(maxsplit=1)
        cmd   = parts[0].lower()
        arg   = parts[1] if len(parts) > 1 else ''

        if   cmd == 'list':   self.cmd_list()
        elif cmd == 'save':   self.cmd_save(arg)
        elif cmd == 'delete': self.cmd_delete(arg)
        elif cmd == 'goto':   self.cmd_goto(arg)
        elif cmd == 'help':   self.print_commands()
        elif cmd == 'exit':
            print('Saving and exiting...')
            self.save_poses()
            return False
        else:
            print(f"Unknown command '{cmd}'. Type 'help' for commands.")
        return True

    def run(self):
        running = True
        while running and rclpy.ok():
            try:
                running = self.process_command(input('> '))
            except (EOFError, KeyboardInterrupt):
                print('\nExiting...')
                self.save_poses()
                running = False


def main(args=None):
    rclpy.init(args=args)
    node = MapAnnotator()

    spin_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    node.run()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()