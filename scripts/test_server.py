#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from quadruped_ctrl.srv import QuadRLController, QuadRLControllerResponse

def handle_quad_rl_controller(req):
    rl_torque = []
    joint_pos = req.leg_data[0:12]
    joint_vel = req.leg_data[12:24]
    joint_tau = req.tau_data
    rl_torque.append(0.0)
    rl_torque.append(0.0)
    rl_torque.append(0.0)
    rospy.loginfo("Call quad_rl_controller Service done!")
    return QuadRLControllerResponse(rl_torque)

def callback_vel(msgs):
    cmd = []
    cmd.append(msgs.linear.x)
    cmd.append(msgs.linear.y)
    cmd.append(msgs.angular.z)


if __name__ == "__main__":
    rospy.init_node('test_quadcontroller_server')
    rospy.Subscriber("cmd_vel", Twist, callback_vel, buff_size=100)
    s = rospy.Service('quad_rl_controller', QuadRLController, handle_quad_rl_controller)
    rospy.loginfo("Start quad_rl_controller Service.")
    rospy.spin()