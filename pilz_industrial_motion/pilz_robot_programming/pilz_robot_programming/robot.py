# Copyright (c) 2018 Pilz GmbH & Co. KG
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

"""API for easy usage of Pilz robot commands - ROS2 Jazzy version."""

from __future__ import absolute_import
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import Quaternion, PoseStamped, Pose
from std_srvs.srv import Trigger
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.action import MoveGroupSequence
from moveit_msgs.srv import GetPlanningScene
from sensor_msgs.msg import JointState

import tf2_ros
import tf2_geometry_msgs  # noqa: F401 - registers transform types

from pilz_msgs.srv import GetSpeedOverride

from .move_control_request import _MoveControlState, MoveControlAction, _MoveControlStateMachine
from .commands import _AbstractCmd, _DEFAULT_PLANNING_GROUP, _DEFAULT_TARGET_LINK, _DEFAULT_BASE_LINK, Sequence
from .exceptions import *

__version__ = '1.1.0'


class Robot(object):
    """Main component of the API which allows the user to execute robot motion commands.

    Supported commands: Ptp, Lin, Circ, Sequence, Gripper.

    ROS2 Jazzy version - uses rclpy action client to communicate with MoveGroupSequence action.
    """

    _SUCCESS = 1
    _STOPPED = -7
    _FAILURE = 99999

    _SEQUENCE_TOPIC = "sequence_move_group"
    _GET_SPEED_OVERRIDE_SRV = "/prbt/get_speed_override"

    _SERVICE_WAIT_TIMEOUT_S = 5.0

    def __init__(self, version=None, node_name='robot_program_node', *args, **kwargs):
        super(Robot, self).__init__(*args, **kwargs)

        self._check_version(version)

        # Create or get a ROS2 node
        if not rclpy.ok():
            rclpy.init()

        self._node = rclpy.create_node(
            '_pilz_robot_api_internal',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self._logger = self._node.get_logger()
        self._callback_group = ReentrantCallbackGroup()

        self._logger.info("Initialize Robot Api (ROS2).")

        # TF2 buffer and listener
        self.tf_buffer_ = tf2_ros.Buffer()
        self.tf_listener_ = tf2_ros.TransformListener(self.tf_buffer_, self._node)

        self._move_lock = threading.Lock()
        self._move_ctrl_sm = _MoveControlStateMachine()

        # Action client for MoveGroupSequence
        self._sequence_client = ActionClient(
            self._node,
            MoveGroupSequence,
            self._SEQUENCE_TOPIC,
            callback_group=self._callback_group,
        )

        self._logger.info(f"Waiting for connection to action server '{self._SEQUENCE_TOPIC}'...")
        if not self._sequence_client.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("Timed out waiting for action server '%s'" % self._SEQUENCE_TOPIC)
        self._logger.info(f"Connection to action server '{self._SEQUENCE_TOPIC}' established.")

        # Current goal handle (for cancellation)
        self._goal_handle = None
        self._goal_handle_lock = threading.Lock()
        self._result_future = None
        self._result_event = threading.Event()
        self._action_result = None

        # Joint state subscriber for getting current joint values
        self._current_joint_state = None
        self._joint_state_lock = threading.Lock()
        self._joint_state_sub = self._node.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10,
            callback_group=self._callback_group,
        )

        # Speed override service client
        self._speed_override_value = 1.0
        try:
            self._speed_override_client = self._node.create_client(
                GetSpeedOverride,
                self._GET_SPEED_OVERRIDE_SRV,
                callback_group=self._callback_group,
            )
            if self._speed_override_client.wait_for_service(timeout_sec=self._SERVICE_WAIT_TIMEOUT_S):
                self._logger.info("Connected to speed override service.")
            else:
                self._logger.warning("Speed override service not available, using default 1.0")
                self._speed_override_client = None
        except Exception:
            self._logger.warning("Speed override service not available, using default 1.0")
            self._speed_override_client = None

        # Pause/resume/stop services
        self._pause_service = self._node.create_service(
            Trigger, "pause_movement", self._pause_service_callback,
            callback_group=self._callback_group)
        self._resume_service = self._node.create_service(
            Trigger, "resume_movement", self._resume_service_callback,
            callback_group=self._callback_group)
        self._stop_service = self._node.create_service(
            Trigger, "stop_movement", self._stop_service_callback,
            callback_group=self._callback_group)

        # Planning frame and group info - obtained from move_group via planning scene
        self._planning_frame = "world"
        self._group_joints = {}

        # Try to get planning scene info
        self._get_planning_scene_client = self._node.create_client(
            GetPlanningScene,
            '/get_planning_scene',
            callback_group=self._callback_group,
        )

        # Start a spinner thread
        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

        # Wait for initial joint state
        self._logger.info("Waiting for joint states...")
        timeout = 10.0
        start = time.time()
        while self._current_joint_state is None and (time.time() - start) < timeout:
            time.sleep(0.1)
        if self._current_joint_state is None:
            self._logger.warning("No joint states received yet, continuing anyway.")
        else:
            self._logger.info("Joint states received.")

        # Discover planning frame and joint groups from the planning scene
        self._discover_robot_info()

        # Read simulation joint limits from move_group parameters (for PTP correction)
        self._sim_joint_vel_limit = None
        self._sim_joint_acc_limit = None
        self._read_sim_joint_limits()

        # Create persistent gripper action client (avoids destroy-while-spinning errors)
        self._gripper_client = None
        self._init_gripper_client()

        self._logger.info("Robot API ready.")

    def _joint_state_callback(self, msg):
        with self._joint_state_lock:
            self._current_joint_state = msg

    def _discover_robot_info(self):
        """Try to discover planning frame and group joints from move_group."""
        if not self._get_planning_scene_client.wait_for_service(timeout_sec=5.0):
            self._logger.warning("Planning scene service not available. Using defaults.")
            self._planning_frame = "world"
            self._group_joints = {
                'manipulator': [
                    'prbt_joint_1', 'prbt_joint_2', 'prbt_joint_3',
                    'prbt_joint_4', 'prbt_joint_5', 'prbt_joint_6'
                ],
                'gripper': ['prbt_gripper_finger_left_joint'],
            }
            return

        req = GetPlanningScene.Request()
        req.components.components = 0  # minimal
        future = self._get_planning_scene_client.call_async(req)

        # Wait for response
        timeout = 5.0
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            time.sleep(0.05)

        if future.done():
            result = future.result()
            if result is not None:
                self._planning_frame = result.scene.robot_model_root if hasattr(result.scene, 'robot_model_root') else "world"
        else:
            self._logger.warning("Timed out getting planning scene.")

        # Use hardcoded joint groups (matching SRDF)
        self._group_joints = {
            'manipulator': [
                'prbt_joint_1', 'prbt_joint_2', 'prbt_joint_3',
                'prbt_joint_4', 'prbt_joint_5', 'prbt_joint_6'
            ],
            'gripper': ['prbt_gripper_finger_left_joint'],
        }

    def _read_sim_joint_limits(self):
        """Read current joint velocity/acceleration limits from move_group parameters.

        These are used by Ptp to compute correction factors so PTP motions
        run at realistic (hardware-like) speeds despite elevated simulation limits.
        """
        from rcl_interfaces.srv import GetParameters
        try:
            param_client = self._node.create_client(
                GetParameters, '/move_group/get_parameters',
                callback_group=self._callback_group,
            )
            if not param_client.wait_for_service(timeout_sec=2.0):
                self._logger.warning("Cannot read move_group params for PTP correction.")
                return

            req = GetParameters.Request()
            req.names = [
                'robot_description_planning.joint_limits.prbt_joint_1.max_velocity',
                'robot_description_planning.joint_limits.prbt_joint_1.max_acceleration',
            ]
            future = param_client.call_async(req)

            timeout = 2.0
            start = time.time()
            while not future.done() and (time.time() - start) < timeout:
                time.sleep(0.05)

            if future.done() and future.result() is not None:
                values = future.result().values
                if len(values) >= 2:
                    self._sim_joint_vel_limit = values[0].double_value
                    self._sim_joint_acc_limit = values[1].double_value
                    self._logger.info(
                        f"PTP correction: sim limits vel={self._sim_joint_vel_limit}, "
                        f"acc={self._sim_joint_acc_limit}"
                    )
        except Exception as e:
            self._logger.warning(f"Failed to read sim joint limits: {e}")

    def _init_gripper_client(self):
        """Create a persistent FollowJointTrajectory client for the gripper."""
        try:
            from rclpy.action import ActionClient as RclpyActionClient
            from control_msgs.action import FollowJointTrajectory
            self._gripper_client = RclpyActionClient(
                self._node,
                FollowJointTrajectory,
                '/gripper_trajectory_controller/follow_joint_trajectory',
                callback_group=self._callback_group,
            )
        except Exception as e:
            self._logger.warning(f"Failed to create gripper action client: {e}")
            self._gripper_client = None

    @property
    def _speed_override(self):
        """Returns the currently active speed override."""
        if self._speed_override_client is not None:
            try:
                req = GetSpeedOverride.Request()
                future = self._speed_override_client.call_async(req)
                timeout = 2.0
                start = time.time()
                while not future.done() and (time.time() - start) < timeout:
                    time.sleep(0.01)
                if future.done() and future.result() is not None:
                    return future.result().speed_override
            except Exception as e:
                self._logger.warning("Failed to get speed override: %s" % str(e))
        return self._speed_override_value

    def get_planning_frame(self):
        """Get the name of the frame in which the robot is planning."""
        return self._planning_frame

    def get_active_joints(self, planning_group):
        """Returns the joints contained in the specified planning group."""
        if planning_group in self._group_joints:
            return self._group_joints[planning_group]
        raise RobotCurrentStateError("Unknown planning group: %s" % planning_group)

    def get_current_joint_states(self, planning_group=_DEFAULT_PLANNING_GROUP):
        """Returns the current joint state values of the robot."""
        with self._joint_state_lock:
            if self._current_joint_state is None:
                raise RobotCurrentStateError("No joint states available.")

            joint_names = self.get_active_joints(planning_group)
            values = []
            for name in joint_names:
                if name in self._current_joint_state.name:
                    idx = list(self._current_joint_state.name).index(name)
                    values.append(self._current_joint_state.position[idx])
                else:
                    values.append(0.0)
            return values

    def get_current_pose_stamped(self, target_link=_DEFAULT_TARGET_LINK, base=_DEFAULT_BASE_LINK):
        """Returns the current stamped pose of target link in the reference frame."""
        try:
            zero_pose = PoseStamped()
            zero_pose.header.frame_id = target_link
            zero_pose.pose.orientation = Quaternion(w=1.0)

            transform = self.tf_buffer_.lookup_transform(
                base, target_link, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=5.0))

            from tf2_geometry_msgs import do_transform_pose_stamped
            current_pose = do_transform_pose_stamped(zero_pose, transform)
            return current_pose
        except Exception as e:
            self._logger.error(str(e))
            raise RobotCurrentStateError(e)

    def get_current_pose(self, target_link=_DEFAULT_TARGET_LINK, base=_DEFAULT_BASE_LINK):
        """Returns the current pose of target link in the reference frame."""
        return self.get_current_pose_stamped(target_link, base).pose

    def move(self, cmd):
        """Execute a robot motion command. Blocks until the command is completed.

        Supported commands: Ptp, Lin, Circ, Sequence, Gripper.
        """
        if not isinstance(cmd, _AbstractCmd):
            self._logger.error("Unknown command type.")
            raise RobotUnknownCommandType("Unknown command type.")

        if not self._move_lock.acquire(False):
            raise RobotMoveAlreadyRunningError("Parallel calls to move are not allowed.")

        self._logger.info(f"Move: {cmd.__class__.__name__}")
        self._logger.debug(f"Move: {cmd}")

        # automatic transition from STOP_REQUESTED to NO_REQUEST
        if self._move_ctrl_sm.state == _MoveControlState.STOP_REQUESTED:
            self._move_ctrl_sm.switch(MoveControlAction.MOTION_STOPPED)

        if self._move_ctrl_sm.state == _MoveControlState.RESUME_REQUESTED:
            self._move_ctrl_sm.switch(MoveControlAction.MOTION_RESUMED)

        try:
            self._move_execution_loop(cmd)
        finally:
            self._move_lock.release()

    def stop(self):
        """Cancel the currently running robot motion command."""
        self._logger.info("Stop called.")
        self._move_ctrl_sm.switch(MoveControlAction.STOP)

        with self._move_ctrl_sm:
            with self._goal_handle_lock:
                if self._goal_handle is not None:
                    self._goal_handle.cancel_goal_async()

    def pause(self):
        """Pause the currently running robot motion command."""
        self._logger.info("Pause called.")
        self._move_ctrl_sm.switch(MoveControlAction.PAUSE)

        with self._move_ctrl_sm:
            with self._goal_handle_lock:
                if self._goal_handle is not None:
                    self._goal_handle.cancel_goal_async()

    def resume(self):
        """Resume a paused robot motion."""
        self._logger.info("Resume called.")
        self._move_ctrl_sm.switch(MoveControlAction.RESUME)

    def _send_goal(self, goal):
        """Send a goal to the sequence action server."""
        self._result_event.clear()
        self._action_result = None

        send_goal_future = self._sequence_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._logger.error("Goal rejected by server")
            self._action_result = None
            self._result_event.set()
            return

        with self._goal_handle_lock:
            self._goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        result = future.result()
        with self._goal_handle_lock:
            self._goal_handle = None
        self._action_result = result.result if result else None
        self._result_event.set()

    def _wait_for_result(self, timeout=120.0):
        """Wait for the action result."""
        self._result_event.wait(timeout=timeout)
        return self._action_result

    def _move_execution_loop(self, cmd):
        continue_execution_of_cmd = True
        first_iteration_flag = True

        while continue_execution_of_cmd:
            self._logger.debug("Move execution loop.")

            if ((self._move_ctrl_sm.state == _MoveControlState.NO_REQUEST and first_iteration_flag) or
                self._move_ctrl_sm.state == _MoveControlState.RESUME_REQUESTED) and \
                    continue_execution_of_cmd:
                self._logger.debug("start execute")

                if self._move_ctrl_sm.state == _MoveControlState.RESUME_REQUESTED:
                    self._move_ctrl_sm.switch(MoveControlAction.MOTION_RESUMED)

                execution_result = cmd._execute(self)

                if execution_result == Robot._STOPPED:
                    if self._move_ctrl_sm.state == _MoveControlState.PAUSE_REQUESTED \
                            or self._move_ctrl_sm.state == _MoveControlState.RESUME_REQUESTED:
                        if isinstance(cmd, Sequence):
                            self._logger.error("Pause not implemented for sequence yet")
                            raise RobotMoveFailed("Pause not implemented for sequence yet")
                    elif self._move_ctrl_sm.state == _MoveControlState.NO_REQUEST:
                        self._logger.error("External stop of move command")
                        raise RobotMoveFailed("External stop of move command")
                    else:
                        self._logger.error("Execution of move command is stopped")
                        raise RobotMoveFailed("Execution of move command is stopped")
                elif execution_result == Robot._SUCCESS:
                    continue_execution_of_cmd = False
                else:
                    self._logger.error(f"Failure during execution of: {cmd}")
                    raise RobotMoveFailed("Failure during execution of: " + str(cmd))

            if self._move_ctrl_sm.state == _MoveControlState.PAUSE_REQUESTED:
                self._logger.info("start wait for resume")
                self._move_ctrl_sm.wait_for_resume()

            if self._move_ctrl_sm.state == _MoveControlState.STOP_REQUESTED:
                self._logger.error("Execution of move command is stopped")
                raise RobotMoveFailed("Execution of move command is stopped")

            first_iteration_flag = False

    def _map_error_code(self, moveit_error_code):
        """Maps the given MoveIt error code to API specific return values."""
        if moveit_error_code.val == MoveItErrorCodes.SUCCESS:
            return self._SUCCESS
        elif moveit_error_code.val == MoveItErrorCodes.PREEMPTED:
            return self._STOPPED
        else:
            return self._FAILURE

    @staticmethod
    def _check_version(version):
        if version is None:
            raise RobotVersionError("Version of Robot API is not set! "
                                    "Current installed version is " + __version__ + "!")
        if version != __version__.split(".")[0]:
            raise RobotVersionError("Version of Robot API does not match! "
                                    "Current installed version is " + __version__ + "!")

    def _pause_service_callback(self, request, response):
        self.pause()
        response.success = True
        response.message = "success"
        return response

    def _resume_service_callback(self, request, response):
        self.resume()
        response.success = True
        response.message = "success"
        return response

    def _stop_service_callback(self, request, response):
        self.stop()
        response.success = True
        response.message = "success"
        return response

    def shutdown(self):
        """Shutdown the robot API and clean up resources."""
        self._logger.info("Shutting down Robot API.")
        with self._goal_handle_lock:
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()
        self._executor.shutdown()
        self._node.destroy_node()

    def __del__(self):
        try:
            self.shutdown()
        except Exception:
            pass
