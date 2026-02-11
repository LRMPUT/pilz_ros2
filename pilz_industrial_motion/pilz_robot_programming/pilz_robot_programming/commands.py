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

import logging
from copy import deepcopy
from math import pi
from operator import add

import rclpy.time
import rclpy.duration
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest,
                             MotionSequenceItem)
from moveit_msgs.action import MoveGroupSequence
import shape_msgs.msg as shape_msgs
from moveit_msgs.msg import (OrientationConstraint, PlanningOptions,
                             PositionConstraint)

from .move_control_request import _MoveControlState

logger = logging.getLogger(__name__)

__version__ = '0.0.dev1'

# Default velocities
_DEFAULT_CARTESIAN_VEL_SCALE = 0.1
_DEFAULT_JOINT_VEL_SCALE = 1.0

# Default acceleration
_DEFAULT_ACC_SCALE = 0.1

# Tolerance for cartesian pose
_DEFAULT_POSITION_TOLERANCE = 2e-3
_DEFAULT_ORIENTATION_TOLERANCE = 1e-5

# axis sequence of euler angles
_AXIS_SEQUENCE = "rzyz"

_DEFAULT_PLANNING_GROUP = "manipulator"
_DEFAULT_TARGET_LINK = "prbt_tcp"
_DEFAULT_GRIPPER_PLANNING_GROUP = "gripper"
_DEFAULT_BASE_LINK = "prbt_base"


class _AbstractCmd(object):
    """Base class for all commands."""

    def __init__(self, *args, **kwargs):
        super(_AbstractCmd, self).__init__(*args, **kwargs)
        self._planning_options = PlanningOptions()
        self._planning_options.planning_scene_diff.robot_state.is_diff = True
        return

    def _get_sequence_request(self, robot):
        """Called by robot class to generate a sequence request."""
        raise NotImplementedError("Cannot execute abstract command")

    def _execute(self, robot):
        logger.debug("Executing command.")

        try:
            sequence_goal = self._get_sequence_request(robot)
        except Exception as e:
            logger.error(str(e))
            return robot._FAILURE

        sequence_goal.planning_options = self._planning_options

        logger.debug("Sending goal.")
        if not _AbstractCmd._locked_send_goal(robot, sequence_goal):
            logger.debug("Command was paused before goal could be sent.")
            return robot._STOPPED
        logger.debug("Wait till motion finished...")

        # Block until the action completes
        result = robot._wait_for_result()

        if result is None:
            logger.error("No result received from action server.")
            return robot._FAILURE

        error_code = result.response.error_code
        if error_code.val != 1:  # not SUCCESS
            logger.error(f"MoveIt error code: {error_code.val}")
            if hasattr(result.response, 'plan_response'):
                for i, pr in enumerate(result.response.plan_response):
                    if pr.error_code.val != 1:
                        logger.error(f"  Item {i} failed with error code: {pr.error_code.val}")

        return robot._map_error_code(error_code)

    @staticmethod
    def _locked_send_goal(robot, goal):
        """Synchronize to the robot._move_ctrl_sm to ensure safe goal sending."""
        with robot._move_ctrl_sm:
            if robot._move_ctrl_sm.state != _MoveControlState.NO_REQUEST:
                return False
            robot._send_goal(goal)
            return True

    def __str__(self):
        out_str = self.__class__.__name__
        return out_str

    __repr__ = __str__


class BaseCmd(_AbstractCmd):
    """Base class for all single commands (Ptp, Lin, Circ)."""

    def __init__(self, goal=None, planning_group=_DEFAULT_PLANNING_GROUP, target_link=_DEFAULT_TARGET_LINK,
                 vel_scale=_DEFAULT_CARTESIAN_VEL_SCALE, acc_scale=_DEFAULT_ACC_SCALE, relative=False,
                 reference_frame=_DEFAULT_BASE_LINK, *args, **kwargs):
        super(BaseCmd, self).__init__(*args, **kwargs)

        self._planner_id = None

        try:
            if isinstance(goal, str):
                raise TypeError()
            self._goal = tuple(goal)
        except TypeError:
            self._goal = goal

        self._planning_group = planning_group
        self._target_link = target_link
        self._vel_scale = vel_scale
        self._acc_scale = acc_scale
        self._relative = relative
        self._reference_frame = reference_frame

    def __str__(self):
        out_str = _AbstractCmd.__str__(self)
        out_str += " vel_scale: " + str(self._vel_scale)
        out_str += " acc_scale: " + str(self._acc_scale)
        out_str += " reference: " + str(self._reference_frame)
        return out_str

    def __eq__(self, other):
        if isinstance(other, BaseCmd):
            return hash(self) == hash(other)
        return NotImplemented

    def __ne__(self, other):
        x = self.__eq__(other)
        if x is not NotImplemented:
            return not x
        return NotImplemented

    def __hash__(self):
        return hash(tuple(sorted([str(t) for t in self.__dict__.items()])))

    __repr__ = __str__

    def _cmd_to_request(self, robot):
        """Transforms the given command to a MotionPlanRequest."""
        req = MotionPlanRequest()

        self._robot_reference_frame = robot.get_planning_frame()
        self._active_joints = robot.get_active_joints(self._planning_group)
        self._start_joint_states = robot.get_current_joint_states(planning_group=self._planning_group)
        self._start_pose = robot.get_current_pose(target_link=self._target_link, base=self._reference_frame)
        self._tf_buffer = robot.tf_buffer_

        # Set general info
        req.planner_id = self._planner_id
        req.group_name = self._planning_group
        req.max_velocity_scaling_factor = self._vel_scale * robot._speed_override
        req.max_acceleration_scaling_factor = self._acc_scale * self._calc_acc_scale(robot._speed_override)
        req.allowed_planning_time = 1.0

        # Set an empty diff as start_state => current state used by planner
        req.start_state.is_diff = True

        # Set goal constraint
        if self._goal is None:
            raise NameError("Goal is not given.")

        convertion_methods = [self._pose_to_constraint,
                              self._pose_stamped_to_constraint,
                              self._joint_values_to_constraint]
        error_list = []

        for method in convertion_methods:
            try:
                req.goal_constraints = method()
                break
            except (TypeError, AttributeError) as e:
                error_list.append(e)
                pass
        else:
            raise NotImplementedError("Unknown type of goal: %s \nerrors: %s" % (str(self._goal), str(error_list)))

        return req

    def _check_header_time(self):
        from builtin_interfaces.msg import Time
        zero_time = Time()
        if self._goal.header.stamp != zero_time:
            raise ValueError("Given goal has unsupported time for future execution.")

    def _joint_values_to_constraint(self, joint_names=()):
        if isinstance(self._goal, str):
            raise TypeError("String is not convertible into joint values.")
        joint_names = joint_names if len(joint_names) != 0 else self._active_joints
        joint_values = list(self._get_joint_pose())
        if len(joint_names) != len(joint_values):
            raise IndexError("Given joint goal does not match the active joints " + str(joint_names) + ".")

        goal_constraints = Constraints()
        goal_constraints.joint_constraints = [JointConstraint(joint_name=name, position=value, weight=1.0)
                                              for name, value in zip(joint_names, joint_values)]
        return [goal_constraints]

    def _pose_to_constraint(self):
        goal_pose = self._get_goal_pose()
        goal_constraints = Constraints()
        robot_reference_frame = self._robot_reference_frame
        goal_constraints.orientation_constraints.append(
            _to_ori_constraint(goal_pose, robot_reference_frame, self._target_link))
        goal_constraints.position_constraints.append(
            _to_pose_constraint(goal_pose, robot_reference_frame, self._target_link))
        return [goal_constraints]

    def _pose_stamped_to_constraint(self):
        self._reference_frame = self._goal.header.frame_id if self._goal.header.frame_id != "" else _DEFAULT_BASE_LINK
        self._check_header_time()
        self._goal = self._goal.pose
        return self._pose_to_constraint()

    def _get_sequence_request(self, robot):
        """Constructs a sequence request from the command."""
        sequence_goal = MoveGroupSequence.Goal()

        sequence_item = MotionSequenceItem()
        sequence_item.blend_radius = 0.0
        sequence_item.req = self._cmd_to_request(robot)

        sequence_goal.request.items.append(sequence_item)

        return sequence_goal

    def _get_goal_pose(self):
        """Determines the goal pose for the given command."""
        if self._relative:
            self._goal = _pose_relative_to_absolute(self._start_pose, self._goal)

        if not self._reference_frame == _DEFAULT_BASE_LINK:
            return self._to_robot_reference(self._reference_frame, self._goal)

        if _is_quaternion_initialized(self._goal.orientation):
            return self._goal
        else:
            return Pose(position=self._goal.position, orientation=self._start_pose.orientation)

    def _get_joint_pose(self):
        """Determines the joint goal for the given command."""
        goal_joint_state = self._goal if not self._relative else \
            list(map(add, self._goal, self._start_joint_states))
        return goal_joint_state

    @staticmethod
    def _calc_acc_scale(vel_scale):
        raise NotImplementedError("Needs to be defined by child class")

    def _to_robot_reference(self, pose_frame, goal_pose_custom_ref):
        """Transforms a pose from a custom reference frame to robot reference frame."""
        if not _is_quaternion_initialized(goal_pose_custom_ref.orientation):
            goal_pose_custom_ref.orientation = Quaternion(w=1.0)
        if pose_frame == self._robot_reference_frame:
            return goal_pose_custom_ref

        stamped = PoseStamped()
        stamped.header.frame_id = pose_frame
        stamped.pose = goal_pose_custom_ref
        from tf2_geometry_msgs import do_transform_pose_stamped
        try:
            transform = self._tf_buffer.lookup_transform(
                self._robot_reference_frame, pose_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0))
            transformed = do_transform_pose_stamped(stamped, transform)
            return transformed.pose
        except Exception as e:
            logger.error(f"TF transform failed from '{pose_frame}' to '{self._robot_reference_frame}': {e}")
            raise


class Ptp(BaseCmd):
    """Represents a single point-to-point (Ptp) command."""

    # PRBT hardware joint limits (constant for real robot)
    _HARDWARE_VEL = 1.57   # rad/s
    _HARDWARE_ACC = 3.49   # rad/s²

    def __init__(self, vel_scale=_DEFAULT_JOINT_VEL_SCALE, acc_scale=None, *args, **kwargs):
        acc_scale_final = acc_scale if acc_scale is not None else Ptp._calc_acc_scale(vel_scale)
        super(Ptp, self).__init__(vel_scale=vel_scale, acc_scale=acc_scale_final, *args, **kwargs)
        self._planner_id = "PTP"

    def _cmd_to_request(self, robot):
        req = super()._cmd_to_request(robot)
        # Correct PTP velocity/acceleration for simulation.
        # URDF limits are elevated so LIN near singularities doesn't fail.
        # This scales PTP back to realistic hardware speeds.
        sim_vel = robot._sim_joint_vel_limit or Ptp._HARDWARE_VEL
        sim_acc = robot._sim_joint_acc_limit or Ptp._HARDWARE_ACC
        if sim_vel > Ptp._HARDWARE_VEL:
            req.max_velocity_scaling_factor *= (Ptp._HARDWARE_VEL / sim_vel)
        if sim_acc > Ptp._HARDWARE_ACC:
            req.max_acceleration_scaling_factor *= (Ptp._HARDWARE_ACC / sim_acc)
        return req

    def __str__(self):
        out_str = BaseCmd.__str__(self)
        if self._relative:
            out_str += " relative: True"
        if isinstance(self._goal, Pose) or isinstance(self._goal, PoseStamped):
            out_str += " Cartesian goal:\n" + str(self._goal)
        elif isinstance(self._goal, tuple):
            out_str += " joint goal: " + str(self._goal)
        return out_str

    __repr__ = __str__

    @staticmethod
    def _calc_acc_scale(vel_scale):
        return vel_scale * vel_scale


class Lin(BaseCmd):
    """Represents a linear command."""

    def __init__(self, vel_scale=_DEFAULT_CARTESIAN_VEL_SCALE, acc_scale=None, *args, **kwargs):
        acc_scale_final = acc_scale if acc_scale is not None else Lin._calc_acc_scale(vel_scale)
        super(Lin, self).__init__(vel_scale=vel_scale, acc_scale=acc_scale_final, *args, **kwargs)
        self._planner_id = "LIN"

    def __str__(self):
        out_str = BaseCmd.__str__(self)
        if self._relative:
            out_str += " relative: True"
        if isinstance(self._goal, Pose) or isinstance(self._goal, PoseStamped):
            out_str += " Cartesian goal:\n" + str(self._goal)
        elif isinstance(self._goal, tuple):
            out_str += " joint goal: " + str(self._goal)
        return out_str

    __repr__ = __str__

    @staticmethod
    def _calc_acc_scale(vel_scale):
        return vel_scale


class Circ(BaseCmd):
    """Represents a circular command."""

    def __init__(self, interim=None, center=None, vel_scale=_DEFAULT_CARTESIAN_VEL_SCALE, acc_scale=None,
                 *args, **kwargs):
        acc_scale_final = acc_scale if acc_scale is not None else Circ._calc_acc_scale(vel_scale)
        super(Circ, self).__init__(vel_scale=vel_scale, acc_scale=acc_scale_final, *args, **kwargs)
        self._planner_id = "CIRC"
        self._interim = interim
        self._center = center

    def __str__(self):
        out_str = BaseCmd.__str__(self)
        if isinstance(self._goal, Pose) and self._goal is not None:
            out_str += " goal:\n" + str(self._goal)
        if self._interim is not None:
            out_str += "\ninterim:\n" + str(self._interim)
        if self._center is not None:
            out_str += "\ncenter:\n" + str(self._center)
        return out_str

    __repr__ = __str__

    def _cmd_to_request(self, robot):
        req = BaseCmd._cmd_to_request(self, robot)

        if self._center is not None and self._interim is not None:
            raise NameError("Both center and interim are set for circ command!")

        if self._center is None and self._interim is None:
            raise NameError("Both center and interim are not set for circ command!")

        path_point = Pose()
        if self._center is not None:
            req.path_constraints.name = 'center'
            path_point.position = self._center
        else:
            req.path_constraints.name = 'interim'
            path_point.position = self._interim

        if not self._reference_frame == _DEFAULT_BASE_LINK:
            path_point = self._to_robot_reference(self._reference_frame, path_point)

        position_constraint = _to_pose_constraint(path_point, self._robot_reference_frame, self._target_link,
                                                  float('+inf'))

        req.path_constraints.position_constraints = [position_constraint]

        return req

    @staticmethod
    def _calc_acc_scale(vel_scale):
        return vel_scale


class _SequenceSubCmd(object):
    def __init__(self, cmd, blend_radius=0.):
        self.cmd = cmd
        self.blend_radius = blend_radius

    def __str__(self):
        out_str = self.__class__.__name__
        out_str += " - " + str(self.cmd)
        out_str += "\nblend radius: " + str(self.blend_radius)
        return out_str

    __repr__ = __str__


class Sequence(_AbstractCmd):
    """Represents an overall Sequence command."""

    def __init__(self, *args, **kwargs):
        super(Sequence, self).__init__(*args, **kwargs)
        self.items = []

    def append(self, cmd, blend_radius=0.0):
        """Adds the given robot motion command to the sequence."""
        self.items.append(_SequenceSubCmd(cmd, blend_radius))

    def _get_sequence_request(self, robot):
        sequence_goal = MoveGroupSequence.Goal()

        for item in self.items:
            curr_sequence_req = MotionSequenceItem()
            curr_sequence_req.blend_radius = item.blend_radius
            curr_sequence_req.req = item.cmd._cmd_to_request(robot)
            sequence_goal.request.items.append(curr_sequence_req)

        return sequence_goal

    def __str__(self):
        out_str = _AbstractCmd.__str__(self)
        out_str += ":\n"
        for item in self.items:
            out_str += str(item)
            out_str += "\n"
        return out_str

    __repr__ = __str__


class Gripper(_AbstractCmd):
    """Represents a gripper command to open and close the gripper.

    Sends goals directly to the gripper_trajectory_controller via
    FollowJointTrajectory action, bypassing MoveGroup (which requires
    an IK solver that the single-joint gripper group doesn't have).
    """

    _GRIPPER_CONTROLLER_ACTION = "/gripper_trajectory_controller/follow_joint_trajectory"

    def __init__(self, goal, vel_scale=_DEFAULT_CARTESIAN_VEL_SCALE, *args, **kwargs):
        super(Gripper, self).__init__(*args, **kwargs)
        self._goal = float(goal)
        self._vel_scale = vel_scale
        self._planning_group = _DEFAULT_GRIPPER_PLANNING_GROUP

    def __str__(self):
        out_str = _AbstractCmd.__str__(self)
        if self._goal is not None:
            out_str += " gripper goal: " + str(self._goal)
        out_str += " velocity scaling: " + str(self._vel_scale)
        return out_str

    __repr__ = __str__

    def _execute(self, robot):
        """Execute gripper command via FollowJointTrajectory action."""
        from control_msgs.action import FollowJointTrajectory
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        from builtin_interfaces.msg import Duration
        import time as _time

        logger.debug("Executing gripper command.")

        gripper_client = robot._gripper_client
        if gripper_client is None:
            logger.error("Gripper action client not initialized.")
            return robot._FAILURE

        joint_names = robot.get_active_joints(self._planning_group)
        if len(joint_names) != 1:
            logger.error("PG70 gripper should have exactly one joint.")
            return robot._FAILURE

        # Get current gripper position
        current_values = robot.get_current_joint_states(planning_group=self._planning_group)
        current_pos = current_values[0] if current_values else 0.0

        # Calculate duration based on distance and velocity
        distance = abs(self._goal - current_pos)
        max_vel = 0.082  # m/s from joint limits
        vel = max_vel * self._vel_scale
        duration_s = distance / vel if vel > 0 else 2.0
        duration_s = max(duration_s, 0.5)  # minimum 0.5s

        # Create the trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = [self._goal]
        point.velocities = [0.0]
        point.time_from_start = Duration(sec=int(duration_s), nanosec=int((duration_s % 1) * 1e9))
        trajectory.points.append(point)

        if not gripper_client.wait_for_server(timeout_sec=5.0):
            logger.error("Gripper trajectory controller action server not available.")
            return robot._FAILURE

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory

        send_future = gripper_client.send_goal_async(goal_msg)

        # Wait for goal acceptance
        timeout = 5.0
        start = _time.time()
        while not send_future.done() and (_time.time() - start) < timeout:
            _time.sleep(0.01)

        if not send_future.done():
            logger.error("Timeout sending gripper goal.")
            return robot._FAILURE

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            logger.error("Gripper goal rejected.")
            return robot._FAILURE

        # Wait for result
        result_future = goal_handle.get_result_async()
        timeout = duration_s + 10.0
        start = _time.time()
        while not result_future.done() and (_time.time() - start) < timeout:
            _time.sleep(0.01)

        if not result_future.done():
            logger.error("Timeout waiting for gripper result.")
            return robot._FAILURE

        result = result_future.result()
        if result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            return robot._SUCCESS
        else:
            logger.error(f"Gripper action failed with error code: {result.result.error_code}")
            return robot._FAILURE


def _to_ori_constraint(pose, reference_frame, link_name, orientation_tolerance=_DEFAULT_ORIENTATION_TOLERANCE):
    """Returns an orientation constraint suitable for ActionGoal's."""
    ori_con = OrientationConstraint()
    ori_con.header.frame_id = reference_frame
    ori_con.link_name = link_name
    ori_con.orientation = pose.orientation
    ori_con.absolute_x_axis_tolerance = orientation_tolerance
    ori_con.absolute_y_axis_tolerance = orientation_tolerance
    ori_con.absolute_z_axis_tolerance = orientation_tolerance
    ori_con.weight = 1.0
    return ori_con


def _to_pose_constraint(pose, reference_frame, link_name, position_tolerance=_DEFAULT_POSITION_TOLERANCE):
    """Returns a position constraint suitable for ActionGoal's."""
    pos_con = PositionConstraint()
    pos_con.header.frame_id = reference_frame
    pos_con.link_name = link_name
    pos_con.constraint_region.primitive_poses.append(pose)
    pos_con.weight = 1.0

    region = shape_msgs.SolidPrimitive()
    region.type = shape_msgs.SolidPrimitive.SPHERE
    region.dimensions.append(position_tolerance)

    pos_con.constraint_region.primitives.append(region)

    return pos_con


def _is_quaternion_initialized(quaternion):
    """Check if the quaternion is initialized."""
    return quaternion != Quaternion()


def _pose_relative_to_absolute(current_pose, relative_pose):
    """Add the offset relative_pose to current_pose and return an absolute goal pose."""
    goal_pose = deepcopy(current_pose)

    goal_pose.position.x += relative_pose.position.x
    goal_pose.position.y += relative_pose.position.y
    goal_pose.position.z += relative_pose.position.z

    a_cur, b_cur, c_cur = euler_from_quaternion([current_pose.orientation.x,
                                                  current_pose.orientation.y,
                                                  current_pose.orientation.z,
                                                  current_pose.orientation.w],
                                                 axes=_AXIS_SEQUENCE)

    a, b, c = euler_from_quaternion([relative_pose.orientation.x,
                                      relative_pose.orientation.y,
                                      relative_pose.orientation.z,
                                      relative_pose.orientation.w],
                                     axes=_AXIS_SEQUENCE)

    a2, b2, c2 = min([pi+a, -pi+a], key=abs), -b, min([pi+c, -pi+c], key=abs)
    if abs(a)+abs(b)+abs(c) > abs(a2)+abs(b2)+abs(c2):
        a, b, c = a2, b2, c2

    goal_pose.orientation = from_euler(a + a_cur, b + b_cur, c + c_cur)

    return goal_pose


def from_euler(a, b, c):
    """Convert euler angles into a geometry_msgs/Quaternion.

    Pass euler angles a, b, c in intrinsic ZYZ convention (in radians).

    Usage::

        r.move(Ptp(goal=Pose(position=Point(0.6, -0.3, 0.2), orientation=from_euler(0, pi, 0))))

    :param a: rotates around the z-axis.
    :param b: rotates around the new y-axis.
    :param c: rotates around the new z-axis.
    """
    q = quaternion_from_euler(a, b, c, axes=_AXIS_SEQUENCE)
    return Quaternion(x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))
