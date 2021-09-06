#!/usr/bin/env python

import rospy
from quadruped_ctrl.srv import QuadRLController, QuadRLControllerResponse

def handle_quad_rl_controller(req):
    rl_torque = []
    joint_pos = req.leg_data[0:12]
    joint_vel = req.leg_data[12:24]
    rl_torque.append(0.0)
    rl_torque.append(0.0)
    rl_torque.append(0.0)
    rospy.loginfo("Call quad_rl_controller Service done!")
    return QuadRLControllerResponse(rl_torque)

if __name__ == "__main__":
    rospy.init_node('test_quadcontroller_server')
    s = rospy.Service('quad_rl_controller', QuadRLController, handle_quad_rl_controller)
    rospy.loginfo("Start quad_rl_controller Service.")
    rospy.spin()