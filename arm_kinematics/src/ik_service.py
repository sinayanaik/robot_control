#!/usr/bin/env python3
import subprocess
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

try:
    import PyKDL as kdl  # type: ignore
    from kdl_parser_py.urdf import treeFromString  # type: ignore
except Exception:
    kdl = None
    treeFromString = None

from arm_kinematics.srv import ComputeIK
from ament_index_python.packages import get_package_share_directory


def _load_urdf_text() -> Optional[str]:
    try:
        urdf_xacro = get_package_share_directory('arm_description') + '/urdf/arm.urdf.xacro'
        proc = subprocess.run(['xacro', urdf_xacro], capture_output=True, check=True, text=True)
        return proc.stdout
    except Exception:
        return None


class IKService(Node):
    def __init__(self) -> None:
        super().__init__('ik_service')
        self.srv = self.create_service(ComputeIK, 'compute_ik', self.handle_request)
        self.tree = None
        self.cache = {}

    def _get_chain(self, base: str, tip: str) -> Optional[object]:
        key = (base, tip)
        if key in self.cache:
            return self.cache[key]
        urdf_text = _load_urdf_text()
        if urdf_text is None or treeFromString is None:
            return None
        ok, tree = treeFromString(urdf_text)
        if not ok:
            return None
        try:
            chain = tree.getChain(base, tip)
        except Exception:
            return None
        self.cache[key] = chain
        return chain

    def handle_request(self, req: ComputeIK.Request, res: ComputeIK.Response):
        if kdl is None:
            res.success = False
            res.message = 'PyKDL not available'
            return res
        chain = self._get_chain(req.base_link or 'base_link', req.tip_link or 'End-Coupler-v1')
        if chain is None:
            res.success = False
            res.message = 'failed to build KDL chain'
            return res

        nj = chain.getNrOfJoints()
        if len(req.joint_names) != nj or len(req.seed_positions) != nj:
            res.success = False
            res.message = 'joint arrays mismatch'
            return res

        frame = kdl.Frame(
            kdl.Rotation.Quaternion(
                req.target.pose.orientation.x,
                req.target.pose.orientation.y,
                req.target.pose.orientation.z,
                req.target.pose.orientation.w,
            ),
            kdl.Vector(
                req.target.pose.position.x,
                req.target.pose.position.y,
                req.target.pose.position.z,
            ),
        )
        solver = kdl.ChainIkSolverPos_LMA(chain)
        q_init = kdl.JntArray(nj)
        for i, v in enumerate(req.seed_positions):
            q_init[i] = v
        q_out = kdl.JntArray(nj)
        status = solver.CartToJnt(q_init, frame, q_out)
        if status < 0:
            res.success = False
            res.message = 'IK failed'
            return res
        res.success = True
        res.message = ''
        res.solution_positions = [q_out[i] for i in range(nj)]
        return res


def main() -> None:
    rclpy.init()
    node = IKService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


