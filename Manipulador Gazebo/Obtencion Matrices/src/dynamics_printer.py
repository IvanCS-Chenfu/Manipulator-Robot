#!/usr/bin/env python3
import os
import tempfile

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory

import xacro
import pinocchio as pin
import numpy as np

from rclpy.qos import QoSProfile, ReliabilityPolicy


# Globals to hold the latest joint state
_q = None
_v = None

def expand_xacro():
    pkg_share = get_package_share_directory('six_dof_manipulator_description')
    xacro_path = os.path.join(pkg_share, 'urdf', '6dof_manipulator.xacro')
    doc = xacro.process_file(xacro_path)
    urdf_xml = doc.toxml()
    fd, tmp = tempfile.mkstemp(suffix='.urdf')
    with os.fdopen(fd, 'w') as f:
        f.write(urdf_xml)
    return tmp

def joint_callback(msg: JointState):
    global _q, _v
    # assume that msg.position and msg.velocity match the model ordering
    _q = np.array(msg.position, dtype=float)
    _v = np.array(msg.velocity, dtype=float)

def timer_callback(node, model, data, frame_id):
    if _q is None or _v is None:
        node.get_logger().info('Waiting for /joint_states...')
        return

    # Compute dynamics
    H = pin.crba(model, data, _q)
    b = pin.rnea(model, data, _q, _v, np.zeros_like(_v))

    pin.computeJointJacobians(model, data, _q)
    J = pin.getFrameJacobian(
        model, data, frame_id,
        pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

    pin.computeJointJacobiansTimeVariation(model, data, _q, _v)
    Jd = pin.getFrameJacobianTimeVariation(
        model, data, frame_id,
        pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

    node.get_logger().info(f'\nH:\n{H}\nb:\n{b}\nJ:\n{J}\nJdot:\n{Jd}\n')

def main():
    # 1) Expand Xacro → temporary URDF
    urdf_tmp = expand_xacro()

    # 2) Build Pinocchio model + data
    model = pin.buildModelFromUrdf(urdf_tmp)
    data  = model.createData()
    # end‐effector frame: use last link “link6”
    frame_id = model.getFrameId('link6')

    # 3) Init ROS2 and a node
    rclpy.init()
    node = Node('dynamics_printer')

    # 4) Subscribe to joint_states
    qos = QoSProfile(depth=10)
    qos.reliability = ReliabilityPolicy.BEST_EFFORT
    node.create_subscription(JointState, '/joint_states', joint_callback, qos)

    # 5) Timer: print once per second
    node.create_timer(
        1.0,
        lambda: timer_callback(node, model, data, frame_id)
    )

    node.get_logger().info('Dynamics printer running...')
    try:
        rclpy.spin(node)
    finally:
        # clean up
        node.destroy_node()
        rclpy.shutdown()
        os.remove(urdf_tmp)

if __name__ == '__main__':
    main()
