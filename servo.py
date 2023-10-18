import threading
from typing import List, Optional, Tuple, Union

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from moveit_msgs.srv import (
    GetCartesianPath,
    GetMotionPlan,
    GetPositionFK,
    GetPositionIK,
)
from rclpy.action import ActionClient
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MoveIt2:
    """
    Python interface for MoveIt 2 that enables planning and execution of trajectories.
    For execution, this interface requires that robot utilises JointTrajectoryController.
    """

    def __init__(
        self,
        node: Node,
        joint_names: List[str],
        base_link_name: str,
        end_effector_name: str,
        group_name: str = "arm",
        execute_via_moveit: bool = False,
        ignore_new_calls_while_executing: bool = False,
        callback_group: Optional[CallbackGroup] = None,
        follow_joint_trajectory_action_name: str = "joint_trajectory_controller/follow_joint_trajectory",
    ):
        """
        Construct an instance of `MoveIt2` interface.
          - `node` - ROS 2 node that this interface is attached to
          - `joint_names` - List of joint names of the robot (can be extracted from URDF)
          - `base_link_name` - Name of the robot base link
          - `end_effector_name` - Name of the robot end effector
          - `group_name` - Name of the planning group for robot arm
          - `execute_via_moveit` - Flag that enables execution via MoveGroup action (MoveIt 2)
                                   FollowJointTrajectory action (controller) is employed otherwise
                                   together with a separate planning service client
          - `ignore_new_calls_while_executing` - Flag to ignore requests to execute new trajectories
                                                 while previous is still being executed
          - `callback_group` - Optional callback group to use for ROS 2 communication (topics/services/actions)
          - `follow_joint_trajectory_action_name` - Name of the action server for the controller
        """

        self._node = node
        self._callback_group = callback_group

        # Create subscriber for current joint states
        self._node.create_subscription(
            msg_type=JointState,
            topic="joint_states",
            callback=self.__joint_state_callback,
            qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
            callback_group=self._callback_group,
        )

        # Create action client for move action
        self.__move_action_client = ActionClient(
            node=self._node,
            action_type=MoveGroup,
            action_name="move_action",
            goal_service_qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
            result_service_qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=5,
            ),
            cancel_service_qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=5,
            ),
            feedback_sub_qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
            status_sub_qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
            callback_group=self._callback_group,
        )

        # Otherwise create a separate service client for planning
        self._plan_kinematic_path_service = self._node.create_client(
            srv_type=GetMotionPlan,
            srv_name="plan_kinematic_path",
            qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
            callback_group=callback_group,
        )
        self.__kinematic_path_request = GetMotionPlan.Request()

        # Create a separate service client for Cartesian planning
        self._plan_cartesian_path_service = self._node.create_client(
            srv_type=GetCartesianPath,
            srv_name="compute_cartesian_path",
            qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
            callback_group=callback_group,
        )
        self.__cartesian_path_request = GetCartesianPath.Request()

        # Create action client for trajectory execution
        self.__follow_joint_trajectory_action_client = ActionClient(
            node=self._node,
            action_type=FollowJointTrajectory,
            action_name=follow_joint_trajectory_action_name,
            goal_service_qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
            result_service_qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=5,
            ),
            cancel_service_qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=5,
            ),
            feedback_sub_qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
            status_sub_qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
            callback_group=self._callback_group,
        )

        self.__collision_object_publisher = self._node.create_publisher(
            CollisionObject, "/collision_object", 10
        )
        self.__attached_collision_object_publisher = self._node.create_publisher(
            AttachedCollisionObject, "/attached_collision_object", 10
        )

        self.__joint_state_mutex = threading.Lock()
        self.__joint_state = None
        self.__new_joint_state_available = False
        self.__move_action_goal = self.__init_move_action_goal(
            frame_id=base_link_name,
            group_name=group_name,
            end_effector=end_effector_name,
        )

        # Flag to determine whether to execute trajectories via MoveIt2, or rather by calling a separate action with the controller itself
        # Applies to `move_to_pose()` and `move_to_configuraion()`
        self.__execute_via_moveit = execute_via_moveit

        # Flag that determines whether a new goal can be send while the previous one is being executed
        self.__ignore_new_calls_while_executing = ignore_new_calls_while_executing

        # Store additional variables for later use
        self.__joint_names = joint_names
        self.__base_link_name = base_link_name
        self.__end_effector_name = end_effector_name
        self.__group_name = group_name

        # Internal states that monitor the current motion requests and execution
        self.__is_motion_requested = False
        self.__is_executing = False
        self.motion_suceeded = False
        self.__wait_until_executed_rate = self._node.create_rate(1000.0)

        # Event that enables waiting until async future is done
        self.__future_done_event = threading.Event()

    def move_to_pose(
        self,
        pose: Optional[Union[PoseStamped, Pose]] = None,
        position: Optional[Union[Point, Tuple[float, float, float]]] = None,
        quat_xyzw: Optional[
            Union[Quaternion, Tuple[float, float, float, float]]
        ] = None,
        target_link: Optional[str] = None,
        frame_id: Optional[str] = None,
        tolerance_position: float = 0.001,
        tolerance_orientation: float = 0.001,
        weight_position: float = 1.0,
        cartesian: bool = False,
        weight_orientation: float = 1.0,
    ):
