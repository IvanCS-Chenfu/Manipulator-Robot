#!/usr/bin/env python3
import os, tempfile

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import (
    Float64MultiArray,
    MultiArrayLayout,
    MultiArrayDimension,
)
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ament_index_python.packages import get_package_share_directory

import xacro
import pinocchio as pin
import numpy as np

#_q = None
#_v = None

_q = None
_v = None

def to_msg(A: np.ndarray) -> Float64MultiArray:
    m = Float64MultiArray()
    # copy the raw data
    m.data = A.flatten().tolist()

    # build a proper layout
    layout = MultiArrayLayout()
    layout.data_offset = 0

    # rows dimension
    dim0 = MultiArrayDimension()
    dim0.label  = 'rows'
    dim0.size   = A.shape[0]
    dim0.stride = A.size
    layout.dim.append(dim0)

    # cols dimension (or 1 for a 1‑D array)
    dim1 = MultiArrayDimension()
    dim1.label  = 'cols'
    dim1.size   = A.shape[1] if A.ndim > 1 else 1
    dim1.stride = A.shape[-1] if A.ndim > 1 else 1
    layout.dim.append(dim1)

    m.layout = layout
    return m


def joint_cb(msg: JointState):
    global _q, _v
    _q = np.array(msg.position)
    _v = np.array(msg.velocity)
    

def main(args=None):
    rclpy.init(args=args)
    node = Node('dynamics_publisher')

    qos = QoSProfile(depth=10)
    qos.reliability = ReliabilityPolicy.BEST_EFFORT
    node.create_subscription(JointState, '/joint_states', joint_cb, qos)
    
    if _q is not None and _v is not None:
        node.get_logger().info(f"Got joint_states q={_q.round(2)} v={_v.round(2)}")

    # Expand Xacro → URDF
    pkg = get_package_share_directory('six_dof_manipulator_description')
    xacro_path = os.path.join(pkg, 'urdf', '6dof_manipulator.xacro')
    doc = xacro.process_file(xacro_path)
    urdf_xml = doc.toxml()
    fd, urdf_tmp = tempfile.mkstemp(suffix='.urdf')
    with os.fdopen(fd,'w') as f:
        f.write(urdf_xml)

    # Build Pinocchio model
    model = pin.buildModelFromUrdf(urdf_tmp)
    data  = model.createData()
    frame = model.getFrameId('link6')

    pub_H  = node.create_publisher(Float64MultiArray, '/dynamics/H', qos)
    pub_b  = node.create_publisher(Float64MultiArray, '/dynamics/b', qos)
    pub_J  = node.create_publisher(Float64MultiArray, '/dynamics/J', qos)
    pub_Jd = node.create_publisher(Float64MultiArray, '/dynamics/Jdot', qos)

    def timer_cb():
        if _q is None or _v is None:
            node.get_logger().info('Waiting for joint_states…')
            return
        H  = pin.crba(model, data, _q)
        b  = pin.rnea(model, data, _q, _v, np.zeros_like(_v))
        pin.computeJointJacobians(model, data, _q)
        J  = pin.getFrameJacobian(model, data, frame, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        pin.computeJointJacobiansTimeVariation(model, data, _q, _v)
        Jd = pin.getFrameJacobianTimeVariation(model, data, frame, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        
        node.get_logger().info(f'\nH:\n{H}\nb:\n{b}\nJ:\n{J}\nJdot:\n{Jd}\n')
                
        pub_H.publish(to_msg(H))
        pub_b.publish(to_msg(b.reshape(1,-1)))
        pub_J.publish(to_msg(J))
        pub_Jd.publish(to_msg(Jd))

    node.create_timer(1.0, timer_cb)
    node.get_logger().info('Publishing dynamics on /dynamics/{H,b,J,Jdot}')
    try:
        rclpy.spin(node)
    finally:
        os.remove(urdf_tmp)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__=='__main__':
    main()
