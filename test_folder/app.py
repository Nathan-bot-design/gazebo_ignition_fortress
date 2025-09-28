# #!/usr/bin/env python3
# import rclpy
# import py_trees
# import py_trees_ros
# from geometry_msgs.msg import PoseStamped, Twist
# from nav_msgs.msg import Odometry
# import sys
# import math
# import time

# def print_same_line(msg1, msg2):
#     sys.stdout.write("\033[F\033[F")  # move cursor 2 lines up
#     sys.stdout.write(f"{msg1}\n")
#     sys.stdout.write(f"{msg2}\n")
#     sys.stdout.flush()

# # ---- Simple print tasks ----
# class PrintHello(py_trees.behaviour.Behaviour):
#     def __init__(self, name="PrintHello"):
#         super().__init__(name)
#         self.executed = False

#     def update(self):
#         if not self.executed:
#             print("hello")
#             self.executed = True
#         return py_trees.common.Status.SUCCESS


# class PrintHi(py_trees.behaviour.Behaviour):
#     def __init__(self, name="PrintHi"):
#         super().__init__(name)
#         self.executed = False

#     def update(self):
#         if not self.executed:
#             print("hi")
#             self.executed = True
#         return py_trees.common.Status.SUCCESS


# # ---- MoveToPosition behaviour using /diff_cont/odom ----
# class MoveToPosition(py_trees.behaviour.Behaviour):
#     # Shared class-level state for all instances
#     origin_x = None
#     origin_y = None
#     origin_yaw = None
#     origin_samples = []
#     origin_sample_count = 10
#     origin_locked = False

#     global_x = 0.0
#     global_y = 0.0
#     global_yaw = 0.0
#     odom_initialized = False
#     odom_sub = None

#     def __init__(self, name, target_x, target_y, tolerance=1.0):
#         super().__init__(name)
#         self.target_x = float(target_x)
#         self.target_y = float(target_y)
#         self.tolerance = float(tolerance)
#         self.cmd_vel_pub = None
#         self.completed = False

#     def setup(self, **kwargs):
#         node = kwargs.get('node')
#         if node:
#             # publisher for velocity
#             self.cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)

#             # create a single shared subscription to the odom topic used in your system
#             if MoveToPosition.odom_sub is None:
#                 MoveToPosition.odom_sub = node.create_subscription(
#                     Odometry,
#                     '/diff_cont/odom',                 # <--- your odom topic
#                     MoveToPosition.shared_odom_callback,
#                     10
#                 )
#                 print("Shared odom subscription created on /diff_cont/odom")
#             print(f"{self.name}: setup complete")

#     @classmethod
#     def shared_odom_callback(cls, msg: Odometry):
#         # read raw odom pose
#         raw_x = msg.pose.pose.position.x
#         raw_y = msg.pose.pose.position.y

#         # quaternion -> yaw
#         o = msg.pose.pose.orientation
#         siny_cosp = 2.0 * (o.w * o.z + o.x * o.y)
#         cosy_cosp = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
#         raw_yaw = math.atan2(siny_cosp, cosy_cosp)

#         # If origin not locked, collect samples while robot is stationary
#         if not cls.origin_locked and cls.origin_x is None:
#             cls.origin_samples.append((raw_x, raw_y, raw_yaw))

#             if len(cls.origin_samples) >= cls.origin_sample_count:
#                 xs = [s[0] for s in cls.origin_samples]
#                 ys = [s[1] for s in cls.origin_samples]
#                 ysaw = [s[2] for s in cls.origin_samples]

#                 x_range = max(xs) - min(xs)
#                 y_range = max(ys) - min(ys)
#                 yaw_range = max(ysaw) - min(ysaw)

#                 # Robot considered stationary if samples are very close
#                 if x_range < 0.005 and y_range < 0.005 and yaw_range < 0.02:
#                     cls.origin_x = sum(xs) / len(xs)
#                     cls.origin_y = sum(ys) / len(ys)
#                     cls.origin_yaw = sum(ysaw) / len(ysaw)
#                     cls.origin_locked = True
#                     print(f"ORIGIN LOCKED at ({cls.origin_x:.3f}, {cls.origin_y:.3f}, {cls.origin_yaw:.3f})")
#                 else:
#                     # Not stationary; clear and retry
#                     cls.origin_samples = []
#         # If origin is locked, compute global coordinates relative to that origin
#         if cls.origin_locked and cls.origin_x is not None:
#             dx = raw_x - cls.origin_x
#             dy = raw_y - cls.origin_y
#             cos_y = math.cos(-cls.origin_yaw)
#             sin_y = math.sin(-cls.origin_yaw)
#             cls.global_x = dx * cos_y - dy * sin_y
#             cls.global_y = dx * sin_y + dy * cos_y
#             cls.global_yaw = raw_yaw - cls.origin_yaw
#             cls.odom_initialized = True

#     def update(self):
#         # If already completed, report success
#         if self.completed:
#             return py_trees.common.Status.SUCCESS

#         # wait until origin is locked and odom data arrived
#         if not MoveToPosition.origin_locked or not MoveToPosition.odom_initialized:
#             # log less frequently to avoid spam
#             print(f"{self.name}: Waiting for origin to be LOCKED and odom data...")
#             return py_trees.common.Status.RUNNING

#         # current position in robot-centered frame
#         current_x = MoveToPosition.global_x
#         current_y = MoveToPosition.global_y
#         current_yaw = MoveToPosition.global_yaw

#         # compute distance to target
#         dx = self.target_x - current_x
#         dy = self.target_y - current_y
#         distance = math.hypot(dx, dy)

#         # reached?
#         if distance < self.tolerance:
#             # stop robot
#             if self.cmd_vel_pub:
#                 stop = Twist()
#                 self.cmd_vel_pub.publish(stop)
#                 # publish a second time to ensure stop
#                 time.sleep(0.05)
#                 self.cmd_vel_pub.publish(stop)
#             print(f"{self.name}: Reached target (dist {distance:.3f} < tol {self.tolerance})")
#             self.completed = True
#             return py_trees.common.Status.SUCCESS

#         # angle to target
#         angle_to_target = math.atan2(dy, dx)
#         angular_error = angle_to_target - current_yaw

#         # normalize to [-pi, pi]
#         while angular_error > math.pi:
#             angular_error -= 2.0 * math.pi
#         while angular_error < -math.pi:
#             angular_error += 2.0 * math.pi

#         # controller
#         twist = Twist()

#         # only drive forward if roughly facing the target
#         if abs(angular_error) < (math.pi / 4.0):
#             if distance > 1.0:
#                 twist.linear.x = 0.3
#             else:
#                 twist.linear.x = max(0.08, distance * 0.25)
#         else:
#             twist.linear.x = 0.0

#         # angular velocity with limit
#         twist.angular.z = max(-0.8, min(0.8, angular_error * 2.0))

#         # print concise status
#         dist_msg = f"{self.name}: pos=({current_x:.2f},{current_y:.2f}) dist={distance:.2f}"
#         ang_msg = f"angle_to_target={angle_to_target:.2f} current_yaw={current_yaw:.2f} err={angular_error:.2f}"
#         print(dist_msg + " | " + ang_msg)

#         # publish velocity
#         if self.cmd_vel_pub:
#             self.cmd_vel_pub.publish(twist)

#         return py_trees.common.Status.RUNNING


# # ---- Helper for goals ----
# def make_pose(x, y, yaw=0.0, frame="map"):
#     pose = PoseStamped()
#     pose.header.frame_id = frame
#     # leave stamp zero (use current time)
#     pose.header.stamp.sec = 0
#     pose.header.stamp.nanosec = 0
#     pose.pose.position.x = float(x)
#     pose.pose.position.y = float(y)
#     pose.pose.position.z = 0.0
#     pose.pose.orientation.x = 0.0
#     pose.pose.orientation.y = 0.0
#     pose.pose.orientation.z = math.sin(yaw / 2.0)
#     pose.pose.orientation.w = math.cos(yaw / 2.0)
#     return pose


# # ---- Build the tree ----
# def create_root():
#     root = py_trees.composites.Sequence("RootSequence", memory=True)

#     move1 = MoveToPosition("MoveToPoint1", 1.0, 1.0, tolerance=0.45)
#     task1 = PrintHello("Task1")
#     move2 = MoveToPosition("MoveToPoint2", -1.0, -1.0, tolerance=0.45)
#     task2 = PrintHi("Task2")

#     root.add_children([move1, task1, move2, task2])
#     return root


# # ---- Main ----
# def main():
#     rclpy.init()
#     root = create_root()

#     # BehaviourTree wrapper (provides a ROS node)
#     bt = py_trees_ros.trees.BehaviourTree(root)

#     try:
#         bt.setup(timeout=10.0, node=bt.node)
#         print("Behavior tree starting...")

#         tree_completed = False

#         def tick_tree():
#             nonlocal tree_completed
#             if tree_completed:
#                 return
#             bt.tick_tock(period_ms=100)  # tick the tree
#             # print concise child statuses
#             for i, child in enumerate(bt.root.children):
#                 print(f"  Child {i} ({child.name}): {child.status}")
#             if bt.root.status == py_trees.common.Status.SUCCESS:
#                 print("Behavior tree completed successfully!")
#                 tree_completed = True
#             elif bt.root.status == py_trees.common.Status.FAILURE:
#                 print("Behavior tree failed!")
#                 tree_completed = True

#         timer = bt.node.create_timer(0.5, tick_tree)
#         print("Behavior tree is running...")
#         rclpy.spin(bt.node)

#     except RuntimeError as e:
#         print(f"Setup failed: {e}")
#         print("Make sure your robot is publishing /diff_cont/odom and that TF is setup correctly.")
#     except KeyboardInterrupt:
#         pass
#     finally:
#         try:
#             bt.shutdown()
#         except Exception:
#             pass
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()

#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
import py_trees
import py_trees_ros
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.qos import QoSProfile
from rclpy.action.client import GoalStatus
import math

# ---- Simple print tasks ----
class PrintHello(py_trees.behaviour.Behaviour):
    def __init__(self, name="PrintHello"):
        super().__init__(name)
        self.executed = False

    def update(self):
        if not self.executed:
            print("hello")
            self.executed = True
        return py_trees.common.Status.SUCCESS


class PrintHi(py_trees.behaviour.Behaviour):
    def __init__(self, name="PrintHi"):
        super().__init__(name)
        self.executed = False

    def update(self):
        if not self.executed:
            print("hi")
            self.executed = True
        return py_trees.common.Status.SUCCESS


# ---- Nav2 Move Node ----
class MoveToMapPosition(py_trees.behaviour.Behaviour):
    def __init__(self, name, target_pose: PoseStamped):
        super().__init__(name)
        self.target_pose = target_pose
        self._action_client = None
        self._goal_future = None
        self._goal_handle = None
        self._result_future = None
        self._completed = False
        self._goal_pub = None
        self._node = None

    def setup(self, **kwargs):
        node = kwargs.get("node")
        if node:
            self._node = node
            self._action_client = ActionClient(node, NavigateToPose, "navigate_to_pose")
            self._goal_pub = node.create_publisher(PoseStamped, "/goal_pose", QoSProfile(depth=10))
            print(f"{self.name}: Action client created for Nav2")

    def update(self):
        if self._completed:
            return py_trees.common.Status.SUCCESS

        if self._goal_future is None:
            if not self._action_client.wait_for_server(timeout_sec=5.0):
                print(f"{self.name}: Nav2 action server not available")
                return py_trees.common.Status.FAILURE

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = self.target_pose

            # send goal to Nav2
            self._goal_future = self._action_client.send_goal_async(goal_msg)
            self._goal_future.add_done_callback(self._goal_response_callback)

            # publish to /goal_pose so RViz shows marker
            if self._goal_pub:
                self._goal_pub.publish(self.target_pose)

            print(f"{self.name}: Goal sent to Nav2")
            return py_trees.common.Status.RUNNING

        # Check if result has been received
        if self._result_future is not None and self._result_future.done():
            result = self._result_future.result().result
            status = self._result_future.result().status

            if status == GoalStatus.STATUS_SUCCEEDED:
                print(f"{self.name}: Reached target successfully")
                self._completed = True
                return py_trees.common.Status.SUCCESS
            else:
                print(f"{self.name}: Failed to reach target, status={status}")
                self._completed = True
                return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def _goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            print(f"{self.name}: Goal rejected")
            self._completed = True
        else:
            print(f"{self.name}: Goal accepted")
            self._result_future = self._goal_handle.get_result_async()


# ---- Helper for map goals ----
def make_pose(x, y, yaw=0.0, frame="map"):
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.header.stamp = rclpy.time.Time().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


# ---- Build the tree ----
def create_root():
    root = py_trees.composites.Sequence("RootSequence", memory=True)

    move1 = MoveToMapPosition("MoveToPoint1", make_pose(1.0, 1.0))
    task1 = PrintHello("Task1")
    move2 = MoveToMapPosition("MoveToPoint2", make_pose(-1.0, -1.0))
    task2 = PrintHi("Task2")

    root.add_children([move1, task1, move2, task2])
    return root


# ---- Main ----
def main():
    rclpy.init()
    root = create_root()
    bt = py_trees_ros.trees.BehaviourTree(root)

    try:
        bt.setup(timeout=10.0, node=bt.node)
        print("Behavior tree starting...")

        tree_completed = False

        def tick_tree():
            nonlocal tree_completed
            if tree_completed:
                return
            bt.tick_tock(period_ms=100)

            # print concise child statuses
            for i, child in enumerate(bt.root.children):
                print(f"  Child {i} ({child.name}): {child.status}")

            if bt.root.status == py_trees.common.Status.SUCCESS:
                print("Behavior tree completed successfully!")
                tree_completed = True
            elif bt.root.status == py_trees.common.Status.FAILURE:
                print("Behavior tree failed!")
                tree_completed = True

        bt.node.create_timer(0.5, tick_tree)
        print("Behavior tree is running...")
        rclpy.spin(bt.node)

    except KeyboardInterrupt:
        pass
    finally:
        bt.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
