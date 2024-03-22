#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from PyKDL import Chain, ChainIkSolverVel_pinv, JntArray, Frame
from tf.transformations import pose_from_msg

class KDLIKPlugin:
    def __init__(self):
        rospy.init_node('kdl_ik_plugin')

        self.base_link = rospy.get_param('~base_link', 'base_link')
        self.end_effector = rospy.get_param('~end_effector', 'end_effector')

        self.chain = Chain()
        self.load_chain()

        self.ik_solver_vel = ChainIkSolverVel_pinv(self.chain)
        self.q = JntArray(self.chain.getNrOfJoints())

        rospy.Subscriber('desired_pose', PoseStamped, self.pose_callback)

    def load_chain(self):
        if rospy.has_param('robot_description'):
            robot_description = rospy.get_param('robot_description')
            ok, _ = self.kdl_parser_tree.initString(robot_description)
            if ok:
                self.kdl_parser_tree.getChain(self.base_link, self.end_effector, self.chain)
            else:
                rospy.logerr("Failed to parse URDF")
                return
        else:
            rospy.logerr("No robot_description parameter found")
            return

    def pose_callback(self, pose_msg):
        desired_pose = Frame()
        position, orientation = pose_from_msg(pose_msg.pose)
        desired_pose.p = position
        desired_pose.M = orientation

        jnt_array_vel = JntArray(self.chain.getNrOfJoints())
        twist = Frame()
        
        if self.ik_solver_vel.CartToJnt(self.q, desired_pose, jnt_array_vel) < 0:
            rospy.logwarn("Failed to solve IK")
            return

        self.q = jnt_array_vel.q

        rospy.loginfo("Joint angles: {}".format(self.q))

if __name__ == '__main__':
    plugin = KDLIKPlugin()
    rospy.spin()