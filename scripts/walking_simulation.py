#!/usr/bin/env python

import os
import numpy
import pyquaternion
import pcl
import tf
import rospy
import rospkg
import time
import threading
import random
import ctypes
from PIL import Image as pil
import pybullet as p
import pybullet_data
from pybullet_utils import gazebo_world_parser
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, SetBoolResponse
from quadruped_ctrl.srv import QuadrupedCmd, QuadrupedCmdResponse
from quadruped_ctrl.srv import QuadRLController, QuadRLControllerResponse


class StructPointer(ctypes.Structure):
    _fields_ = [("eff", ctypes.c_double * 12)]


def convert_type(input):
    ctypes_map = {int: ctypes.c_int,
                  float: ctypes.c_double,
                  str: ctypes.c_char_p
                  }
    input_type = type(input)
    if input_type is list:
        length = len(input)
        if length == 0:
            rospy.logerr("convert type failed...input is "+input)
            return 0
        else:
            arr = (ctypes_map[type(input[0])] * length)()
            for i in range(length):
                arr[i] = bytes(
                    input[i], encoding="utf-8") if (type(input[0]) is str) else input[i]
            return arr
    else:
        if input_type in ctypes_map:
            return ctypes_map[input_type](bytes(input, encoding="utf-8") if type(input) is str else input)
        else:
            rospy.logerr("convert type failed...input is "+input)
            return 0


def acc_filter(value, last_accValue):
    a = 1
    filter_value = a * value + (1 - a) * last_accValue
    return filter_value


class QuadSimulator:
    #
    def __init__(self):

        rospy.init_node("quadruped_simulator", anonymous=True)
        # global param
        self.get_last_vel = [0] * 3
        self.robot_height = 0.30
        self.motor_id_list = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14]
        self.init_new_pos = [0.0, -0.8, 1.6, 0.0, -0.8, 1.6, 0.0, -0.8, 1.6, 0.0, -0.8, 1.6,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.reset_flag1 = False
        self.robot_start_pos = [0, 0, 0.42]
        # load ros param
        self.use_rl_controller = rospy.get_param('~use_rl_controller', False)
        self.terrain = rospy.get_param('/simulation/terrain')
        self.camera = rospy.get_param('/simulation/camera')
        self.lateralFriction = rospy.get_param('/simulation/lateralFriction')
        self.spinningFriction = rospy.get_param('/simulation/spinningFriction')
        self.freq = rospy.get_param('/simulation/freq')
        self.stand_kp = rospy.get_param('/simulation/stand_kp')
        self.stand_kd = rospy.get_param('/simulation/stand_kd')
        self.joint_kp = rospy.get_param('/simulation/joint_kp')
        self.joint_kd = rospy.get_param('/simulation/joint_kd')
        rospy.loginfo("lateralFriction = " + str(self.lateralFriction) + " spinningFriction = " + str(self.spinningFriction))
        rospy.loginfo(" freq = " + str(self.freq) + " PID = " + str([self.stand_kp, self.stand_kd, self.joint_kp, self.joint_kd]))

        # load cpp library
        rospack = rospkg.RosPack()
        path = rospack.get_path('quadruped_ctrl')
        so_file = path.replace('src/quadruped_ctrl',
                               'devel/lib/libquadruped_ctrl.so')
        if(not os.path.exists(so_file)):
            so_file = path.replace('src/quadruped_ctrl',
                                   'build/lib/libquadruped_ctrl.so')
        if(not os.path.exists(so_file)):
            rospy.logerr("cannot find cpp.so file")
        self.cpp_gait_ctrller = ctypes.cdll.LoadLibrary(so_file)
        self.cpp_gait_ctrller.toque_calculator.restype = ctypes.POINTER(StructPointer)
        rospy.logwarn("find so file = " + so_file)

        self.robot_tf = tf.TransformBroadcaster()
        # add ros server and subscriber
        s = rospy.Service('gait_type', QuadrupedCmd, self.callback_gait)
        s1 = rospy.Service('robot_mode', QuadrupedCmd, self.callback_mode)
        s2 = rospy.Service('robot_reset', SetBool, self.callback_reset)
        rospy.Subscriber("cmd_vel", Twist, self.callback_body_vel, buff_size=10000)

        self.pub_odom = rospy.Publisher("robot_odom", Odometry, queue_size=100)
        self.pub_imu = rospy.Publisher("imu0", Imu, queue_size=100)
        self.pub_joint = rospy.Publisher("joint_states", JointState, queue_size=100)
        self.pointcloud_publisher = rospy.Publisher("generated_pc", PointCloud2, queue_size=10)
        self.image_publisher = rospy.Publisher("cam0/image_raw", Image, queue_size=10)

        self.init_simulator()

        rospy.loginfo("waitting quad_rl_controller server...")
        rospy.wait_for_service('quad_rl_controller')
        rospy.loginfo("server done. start threading...")


    def callback_body_vel(self, msg):
        vel = [msg.linear.x, msg.linear.y, msg.angular.x, msg.angular.y]
        self.cpp_gait_ctrller.set_robot_vel(convert_type(vel))

    def callback_gait(self, req):
        self.cpp_gait_ctrller.set_gait_type(convert_type(req.cmd))
        return QuadrupedCmdResponse(0, "get the gait")

    def callback_mode(self, req):
        self.cpp_gait_ctrller.set_robot_mode(convert_type(req.cmd))
        return QuadrupedCmdResponse(0, "get the mode")

    def callback_reset(self, req):
        if req.data is True:
            self.reset_flag1 = True
            return SetBoolResponse(True, "reset the robot")
        else:
            return SetBoolResponse(False, "reset the robot")

    def pub_nav_msg(self, body_data, imu_data):
        odom = Odometry()
        odom.pose.pose.position.x = body_data[0]
        odom.pose.pose.position.y = body_data[1]
        odom.pose.pose.position.z = body_data[2]
        odom.pose.pose.orientation.x = imu_data[3]
        odom.pose.pose.orientation.y = imu_data[4]
        odom.pose.pose.orientation.z = imu_data[5]
        odom.pose.pose.orientation.w = imu_data[6]
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id ="world"
        odom.child_frame_id = "world"
        self.pub_odom.publish(odom)

    def pub_imu_msg(self, imu_data):
        imu_msg = Imu()
        imu_msg.linear_acceleration.x = imu_data[0]
        imu_msg.linear_acceleration.y = imu_data[1]
        imu_msg.linear_acceleration.z = imu_data[2]
        imu_msg.angular_velocity.x = imu_data[7]
        imu_msg.angular_velocity.y = imu_data[8]
        imu_msg.angular_velocity.z = imu_data[9]
        imu_msg.orientation.x = imu_data[3]
        imu_msg.orientation.y = imu_data[4]
        imu_msg.orientation.z = imu_data[5]
        imu_msg.orientation.w = imu_data[6]
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "robot"
        self.pub_imu.publish(imu_msg)

    def pub_joint_msg(self, joint_data, eff):
        joint_msg = JointState()
        joint_msg.name.append("torso_to_abduct_fr_j")
        joint_msg.name.append("abduct_fr_to_thigh_fr_j")
        joint_msg.name.append("thigh_fr_to_knee_fr_j")
        joint_msg.name.append("torso_to_abduct_fl_j")
        joint_msg.name.append("abduct_fl_to_thigh_fl_j")
        joint_msg.name.append("thigh_fl_to_knee_fl_j")
        joint_msg.name.append("torso_to_abduct_hr_j")
        joint_msg.name.append("abduct_hr_to_thigh_hr_j")
        joint_msg.name.append("thigh_hr_to_knee_hr_j")
        joint_msg.name.append("torso_to_abduct_hl_j")
        joint_msg.name.append("abduct_hl_to_thigh_hl_j")
        joint_msg.name.append("thigh_hl_to_knee_hl_j")
        for i in range(0, 12):
            joint_msg.position.append(joint_data[i])
            joint_msg.velocity.append(joint_data[i+12])
            joint_msg.effort.append(eff[i])
        joint_msg.header.stamp = rospy.Time.now()
        joint_msg.header.frame_id = "robot"
        self.pub_joint.publish(joint_msg)

    def get_data_from_sim(self):
        get_orientation = []
        get_matrix = []
        get_velocity = []
        get_invert = []
        imu_data = [0] * 10
        leg_data = [0] * 24
        body_data = [0] * 6

        pose_orn = p.getBasePositionAndOrientation(self.boxId)

        for i in range(4):
            get_orientation.append(pose_orn[1][i])
        get_euler = p.getEulerFromQuaternion(get_orientation)
        get_velocity = p.getBaseVelocity(self.boxId)
        get_invert = p.invertTransform(pose_orn[0], pose_orn[1])
        get_matrix = p.getMatrixFromQuaternion(get_invert[1])

        # Body pose data
        body_data[0] = pose_orn[0][0]
        body_data[1] = pose_orn[0][1]
        body_data[2] = pose_orn[0][2]
        body_data[3] = get_euler[0]
        body_data[4] = get_euler[1]
        body_data[5] = get_euler[2]

        # IMU data
        # orientation
        imu_data[3] = pose_orn[1][0]
        imu_data[4] = pose_orn[1][1]
        imu_data[5] = pose_orn[1][2]
        imu_data[6] = pose_orn[1][3]

        # angular_velocity
        imu_data[7] = get_matrix[0] * get_velocity[1][0] + get_matrix[1] * \
            get_velocity[1][1] + get_matrix[2] * get_velocity[1][2]
        imu_data[8] = get_matrix[3] * get_velocity[1][0] + get_matrix[4] * \
            get_velocity[1][1] + get_matrix[5] * get_velocity[1][2]
        imu_data[9] = get_matrix[6] * get_velocity[1][0] + get_matrix[7] * \
            get_velocity[1][1] + get_matrix[8] * get_velocity[1][2]

        # calculate the linear_acceleration of the robot
        linear_X = (get_velocity[0][0] - self.get_last_vel[0]) * self.freq
        linear_Y = (get_velocity[0][1] - self.get_last_vel[1]) * self.freq
        linear_Z = 9.8 + (get_velocity[0][2] - self.get_last_vel[2]) * self.freq
        imu_data[0] = get_matrix[0] * linear_X + \
            get_matrix[1] * linear_Y + get_matrix[2] * linear_Z
        imu_data[1] = get_matrix[3] * linear_X + \
            get_matrix[4] * linear_Y + get_matrix[5] * linear_Z
        imu_data[2] = get_matrix[6] * linear_X + \
            get_matrix[7] * linear_Y + get_matrix[8] * linear_Z

        # joint data
        joint_state = p.getJointStates(self.boxId, self.motor_id_list)
        # jointPosition 0:abad 1:hip 2:knee
        leg_data[0:12] = [joint_state[0][0], joint_state[1][0], joint_state[2][0],
                          joint_state[3][0], joint_state[4][0], joint_state[5][0],
                          joint_state[6][0], joint_state[7][0], joint_state[8][0],
                          joint_state[9][0], joint_state[10][0], joint_state[11][0]]
        # jointVelocity
        leg_data[12:24] = [joint_state[0][1], joint_state[1][1], joint_state[2][1],
                           joint_state[3][1], joint_state[4][1], joint_state[5][1],
                           joint_state[6][1], joint_state[7][1], joint_state[8][1],
                           joint_state[9][1], joint_state[10][1], joint_state[11][1]]
        com_velocity = [get_velocity[0][0],
                        get_velocity[0][1], get_velocity[0][2]]
        # get_last_vel.clear()
        self.get_last_vel = []
        self.get_last_vel = com_velocity

        # return imu_data, leg_data, pose_orn[0]
        return imu_data, leg_data, body_data


    def reset_robot(self):
        if self.terrain == "racetrack":
            robot_z = 0.4
        else:
            robot_z = self.robot_height
        p.resetBasePositionAndOrientation(
            self.boxId, [0, 0, robot_z], [0, 0, 0, 1])
        p.resetBaseVelocity(self.boxId, [0, 0, 0], [0, 0, 0])
        for j in range(12):
            p.resetJointState(self.boxId, self.motor_id_list[j], self.init_new_pos[j], self.init_new_pos[j+12])
        self.cpp_gait_ctrller.init_controller(
            convert_type(self.freq), convert_type([self.stand_kp, self.stand_kd, self.joint_kp, self.joint_kd]))

        for _ in range(10):
            p.stepSimulation()
            imu_data, leg_data, _ = self.get_data_from_sim()
            self.cpp_gait_ctrller.pre_work(convert_type(imu_data), convert_type(leg_data))

        for j in range(16):
            force = 0
            p.setJointMotorControl2(
                self.boxId, j, p.VELOCITY_CONTROL, force=force)

        self.cpp_gait_ctrller.set_robot_mode(convert_type(0))
        self.cpp_gait_ctrller.set_gait_type(convert_type(4))
        # for _ in range(200):
        #     run()
        #     p.stepSimulation
        # self.cpp_gait_ctrller.set_robot_mode(convert_type(0))


    def init_simulator(self):
        p.connect(p.GUI)  # or p.DIRECT for non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        p.resetSimulation()
        p.setTimeStep(1.0/self.freq)
        p.setGravity(0, 0, -9.8)
        self.reset = p.addUserDebugParameter("reset", 1, 0, 0)
        self.low_energy_mode = p.addUserDebugParameter("low_energy_mode", 1, 0, 0)
        self.high_performance_mode = p.addUserDebugParameter("high_performance_mode", 1, 0, 0)
        p.resetDebugVisualizerCamera(0.2, 45, -30, [1, -1, 1])

        heightPerturbationRange = 0.06
        numHeightfieldRows = 256
        numHeightfieldColumns = 256
        # set environment and terrain
        if self.terrain == "plane":
            planeShape = p.createCollisionShape(shapeType=p.GEOM_PLANE)
            ground_id = p.createMultiBody(0, planeShape)
            p.resetBasePositionAndOrientation(ground_id, [0, 0, 0], [0, 0, 0, 1])
            p.changeDynamics(ground_id, -1, lateralFriction=self.lateralFriction)
        elif self.terrain == "random1":
            heightfieldData = [0]*numHeightfieldRows*numHeightfieldColumns
            for j in range(int(numHeightfieldColumns/2)):
                for i in range(int(numHeightfieldRows/2)):
                    height = random.uniform(0, heightPerturbationRange)
                    heightfieldData[2*i+2*j*numHeightfieldRows] = height
                    heightfieldData[2*i+1+2*j*numHeightfieldRows] = height
                    heightfieldData[2*i+(2*j+1)*numHeightfieldRows] = height
                    heightfieldData[2*i+1+(2*j+1)*numHeightfieldRows] = height
            terrainShape = p.createCollisionShape(shapeType=p.GEOM_HEIGHTFIELD, meshScale=[.05, .05, 1], heightfieldTextureScaling=(
                numHeightfieldRows-1)/2, heightfieldData=heightfieldData, numHeightfieldRows=numHeightfieldRows, numHeightfieldColumns=numHeightfieldColumns)
            ground_id = p.createMultiBody(0, terrainShape)
            p.resetBasePositionAndOrientation(ground_id, [0, 0, 0], [0, 0, 0, 1])
            p.changeDynamics(ground_id, -1, lateralFriction=self.lateralFriction)
        elif self.terrain == "random2":
            terrain_shape = p.createCollisionShape(
                shapeType=p.GEOM_HEIGHTFIELD,
                meshScale=[.5, .5, .5],
                fileName="heightmaps/ground0.txt",
                heightfieldTextureScaling=128)
            ground_id = p.createMultiBody(0, terrain_shape)
            textureId = p.loadTexture(path+"/models/grass.png")
            p.changeVisualShape(ground_id, -1, textureUniqueId=textureId)
            p.resetBasePositionAndOrientation(ground_id, [1, 0, 0.2], [0, 0, 0, 1])
            p.changeDynamics(ground_id, -1, lateralFriction=self.lateralFriction)
        elif self.terrain == "stairs":
            planeShape = p.createCollisionShape(shapeType=p.GEOM_PLANE)
            ground_id = p.createMultiBody(0, planeShape)
            # p.resetBasePositionAndOrientation(ground_id, [0, 0, 0], [0, 0.0872, 0, 0.9962])
            p.resetBasePositionAndOrientation(ground_id, [0, 0, 0], [0, 0, 0, 1])
            # many box
            colSphereId = p.createCollisionShape(
                p.GEOM_BOX, halfExtents=[0.1, 0.4, 0.01])
            colSphereId1 = p.createCollisionShape(
                p.GEOM_BOX, halfExtents=[0.1, 0.4, 0.02])
            colSphereId2 = p.createCollisionShape(
                p.GEOM_BOX, halfExtents=[0.1, 0.4, 0.03])
            colSphereId3 = p.createCollisionShape(
                p.GEOM_BOX, halfExtents=[0.1, 0.4, 0.04])
            # colSphereId4 = p.createCollisionShape(
            #     p.GEOM_BOX, halfExtents=[0.03, 0.03, 0.03])
            p.createMultiBody(100, colSphereId, basePosition=[1.0, 1.0, 0.0])
            p.changeDynamics(colSphereId, -1, lateralFriction=self.lateralFriction)
            p.createMultiBody(100, colSphereId1, basePosition=[1.2, 1.0, 0.0])
            p.changeDynamics(colSphereId1, -1, lateralFriction=self.lateralFriction)
            p.createMultiBody(100, colSphereId2, basePosition=[1.4, 1.0, 0.0])
            p.changeDynamics(colSphereId2, -1, lateralFriction=self.lateralFriction)
            p.createMultiBody(100, colSphereId3, basePosition=[1.6, 1.0, 0.0])
            p.changeDynamics(colSphereId3, -1, lateralFriction=self.lateralFriction)
            # p.createMultiBody(10, colSphereId4, basePosition=[2.7, 1.0, 0.0])
            # p.changeDynamics(colSphereId4, -1, lateralFriction=0.5)
            p.changeDynamics(ground_id, -1, lateralFriction=self.lateralFriction)
        elif self.terrain == "racetrack":
            os.chdir(path)
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
            gazebo_world_parser.parseWorld(p, filepath = "worlds/racetrack_day.world")
            p.configureDebugVisualizer(shadowMapResolution = 8192)
            p.configureDebugVisualizer(shadowMapWorldSize = 25)
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

        # load robots and reset to default state
        self.boxId = p.loadURDF("mini_cheetah/mini_cheetah.urdf", self.robot_start_pos,
                           useFixedBase=False)
        # self.boxId = p.loadURDF(fileName="/home/zhenz/workspaces/quadruped_ctrl_ws/src/quadruped_ctrl/mini_cheetah/mini_cheetah1.urdf",
        #                    basePosition=self.robot_start_pos, useFixedBase=False)
        p.changeDynamics(self.boxId, 3, spinningFriction=self.spinningFriction)
        p.changeDynamics(self.boxId, 7, spinningFriction=self.spinningFriction)
        p.changeDynamics(self.boxId, 11, spinningFriction=self.spinningFriction)
        p.changeDynamics(self.boxId, 15, spinningFriction=self.spinningFriction)
        jointIds = []
        for j in range(p.getNumJoints(self.boxId)):
            p.getJointInfo(self.boxId, j)
            jointIds.append(j)

        self.reset_robot()


    def run(self):
        # get data from simulator
        imu_data, leg_data, body_data = self.get_data_from_sim()

        # call cpp function to calculate mpc tau
        tau = self.cpp_gait_ctrller.toque_calculator(convert_type(
            imu_data), convert_type(leg_data))
        eff_data = tuple(tau.contents.eff)

        # set tau to simulator
        if self.use_rl_controller is False:
            # rospy.loginfo("call robot MPC Controller.")
            p.setJointMotorControlArray(bodyUniqueId=self.boxId,
                                        jointIndices=self.motor_id_list,
                                        controlMode=p.TORQUE_CONTROL,
                                        forces=eff_data)
        else:
            rl_torque_results = tuple()
            # call RL server to calculate tau
            try:
                # rospy.loginfo("call robot RLController server...")
                robot_RLcontroller = rospy.ServiceProxy('quad_rl_controller', QuadRLController)
                rl_torque_results = robot_RLcontroller(body_data, imu_data, leg_data, eff_data).torque
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

            if len(rl_torque_results) == 12:
                # rospy.logwarn("call robot RLController done!")
                p.setJointMotorControlArray(bodyUniqueId=self.boxId,
                                            jointIndices=self.motor_id_list,
                                            controlMode=p.TORQUE_CONTROL,
                                            forces=rl_torque_results)
            else: rospy.logerr("call robot RLController failed.")

        # reset visual cam
        # p.resetDebugVisualizerCamera(2.5, 45, -30, body_data)

        # pub ros msg
        self.pub_nav_msg(body_data, imu_data)
        self.pub_imu_msg(imu_data)
        self.pub_joint_msg(leg_data, eff_data)

        p.stepSimulation()
        return


    def spin_job(self):
        rospy.spin()

    def camera_update_job(self):
        rate_1 = rospy.Rate(20)
        near = 0.1
        far = 1000
        step_index = 4
        pixelWidth = int(320/step_index)
        pixelHeight = int(240/step_index)
        cameraEyePosition = [0.3, 0, 0.26436384367425125]
        cameraTargetPosition = [1.0, 0, 0]
        cameraUpVector = [45, 45, 0]
        pub_pointcloud = PointCloud2()
        pub_image = Image()

        while not rospy.is_shutdown():
            cubePos, cubeOrn = p.getBasePositionAndOrientation(self.boxId)
            get_matrix = p.getMatrixFromQuaternion(cubeOrn)
            # T1 = numpy.mat([[0, -numpy.sqrt(2.0)/2.0, numpy.sqrt(2.0)/2.0, 0.25], [-1, 0, 0, 0],
            #                 [0, -numpy.sqrt(2.0)/2.0, -numpy.sqrt(2.0)/2.0, 0], [0, 0, 0, 1]])
            T1 = numpy.mat([[0, -1.0/2.0, numpy.sqrt(3.0)/2.0, 0.25], [-1, 0, 0, 0],
                            [0, -numpy.sqrt(3.0)/2.0, -1.0/2.0, 0], [0, 0, 0, 1]])

            T2 = numpy.mat([[get_matrix[0], get_matrix[1], get_matrix[2], cubePos[0]],
                            [get_matrix[3], get_matrix[4], get_matrix[5], cubePos[1]],
                            [get_matrix[6], get_matrix[7], get_matrix[8], cubePos[2]],
                            [0, 0, 0, 1]])

            T2_ = (T2.I)
            T3_ = numpy.array(T2*T1)

            cameraEyePosition[0] = T3_[0][3]
            cameraEyePosition[1] = T3_[1][3]
            cameraEyePosition[2] = T3_[2][3]
            cameraTargetPosition = (numpy.mat(T3_)*numpy.array([[0],[0],[1],[1]]))[0:3]

            q = pyquaternion.Quaternion(matrix=T3_)
            cameraQuat = [q[1], q[2], q[3], q[0]]

            self.robot_tf.sendTransform(cubePos, cubeOrn, rospy.Time.now(), "robot", "world")
            self.robot_tf.sendTransform(cameraEyePosition, cameraQuat, rospy.Time.now(), "cam", "world")
            self.robot_tf.sendTransform(cameraTargetPosition, cubeOrn, rospy.Time.now(), "tar", "world")

            cameraUpVector = [0, 0, 1]
            viewMatrix = p.computeViewMatrix(
                cameraEyePosition, cameraTargetPosition, cameraUpVector)
            aspect = float(pixelWidth) / float(pixelHeight)
            projectionMatrix = p.computeProjectionMatrixFOV(60, aspect, near, far)
            width, height, rgbImg, depthImg, _ = p.getCameraImage(pixelWidth,
                                       pixelHeight,
                                       viewMatrix=viewMatrix,
                                       projectionMatrix=projectionMatrix,
                                       shadow=1,
                                       lightDirection=[1, 1, 1],
                                       renderer=p.ER_BULLET_HARDWARE_OPENGL)

            # point cloud mehted 1
            # imgW = width
            # imgH = height
            # depth_img_buffer = numpy.reshape(depthImg, [imgH, imgW])
            # projectionMatrix1 = numpy.asarray(projectionMatrix).reshape([4,4],order='F')
            # viewMatrix1 = numpy.asarray(viewMatrix).reshape([4,4],order='F')
            # tran_pix_world = numpy.linalg.inv(numpy.matmul(projectionMatrix1, viewMatrix1))
            # pcl_data = pcl.PointCloud()
            # pc_list = [0]*(imgW*imgH)
            # pc = numpy.zeros(3)
            # pixPos = numpy.ones(4)
            # pixPosZ = (2.0*depth_img_buffer - 1.0)
            # for h in range(0, imgH):
            #     for w in range(0, imgW):
            #         pixPos[0] = (2.0*w - imgW)/imgW
            #         pixPos[1] = -(2.0*h - imgH)/imgH
            #         pixPos[2] = pixPosZ[h,w]
            #         position = tran_pix_world.dot(pixPos)
            #         for ii in range(3):
            #             pc[ii] = position[ii] / position[3]
            #         pc_list[h*imgW+w]=pc.tolist()

            # point cloud mehted 2
            pc_list = []
            pcl_data = pcl.PointCloud()
            fx = (pixelWidth*projectionMatrix[0])/2.0
            fy = (pixelHeight*projectionMatrix[5])/2.0
            cx = (1-projectionMatrix[2])*pixelWidth/2.0
            cy = (1+projectionMatrix[6])*pixelHeight/2.0
            cloud_point = [0]*pixelWidth*pixelHeight*3
            depthBuffer = numpy.reshape(depthImg,[pixelHeight,pixelWidth])
            depth = depthBuffer
            for h in range(0, pixelHeight):
                for w in range(0, pixelWidth):
                    depth[h][w] =float(depthBuffer[h,w])
                    depth[h][w] = far * near / (far - (far - near) * depthBuffer[h][w])
                    Z= float(depth[h][w])
                    if (Z >4):
                        continue
                    if (Z< 0.01):
                        continue
                    X=(w-cx)*Z/fx
                    Y=(h-cy)*Z/fy
                    XYZ_= numpy.mat([[X],[Y],[Z],[1]])
                    XYZ =numpy.array(T3_*XYZ_)
                    # XYZ = numpy.array(XYZ_)
                    X= float(XYZ[0])
                    Y= float(XYZ[1])
                    Z= float(XYZ[2])
                    cloud_point[h*pixelWidth*3+w*3+0] = float(X)
                    cloud_point[h*pixelWidth*3+w*3+1] = float(Y)
                    cloud_point[h*pixelWidth*3+w*3+2] = float(Z)
                    pc_list.append([X,Y,Z])

            pcl_data.from_list(pc_list)
            pub_pointcloud.header.stamp = rospy.Time().now()
            pub_pointcloud.header.frame_id = "world"
            pub_pointcloud.height = 1
            pub_pointcloud.width = len(pc_list)
            pub_pointcloud.point_step = 12
            pub_pointcloud.fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1)]
            pub_pointcloud.data = numpy.asarray(pc_list, numpy.float32).tostring()
            self.pointcloud_publisher.publish(pub_pointcloud)

            # grey image
            pub_image.header.stamp = rospy.Time().now()
            pub_image.header.frame_id = "cam"
            pub_image.width = width
            pub_image.height = height
            pub_image.encoding = "mono8"
            pub_image.step = width
            grey = pil.fromarray(rgbImg)
            pub_image.data = numpy.asarray(grey.convert('L')).reshape([1,-1]).tolist()[0]
            self.image_publisher.publish(pub_image)

            rate_1.sleep()


    def main(self):
        add_thread = threading.Thread(target=self.spin_job)
        add_thread.start()
        if self.camera:
            add_thread_1 = threading.Thread(target=self.camera_update_job)
            add_thread_1.start()

        cnt = 0
        rate = rospy.Rate(self.freq)  # hz
        reset_flag = p.readUserDebugParameter(self.reset)
        low_energy_flag = p.readUserDebugParameter(self.low_energy_mode)
        high_performance_flag = p.readUserDebugParameter(self.high_performance_mode)
        while not rospy.is_shutdown():
            if self.reset_flag1 is True:
                self.reset_flag1 = False
                rospy.logwarn("reset the robot")
                cnt = 0
                self.reset_robot()
            # check reset button state
            if(reset_flag < p.readUserDebugParameter(self.reset)):
                reset_flag = p.readUserDebugParameter(self.reset)
                rospy.logwarn("reset the robot")
                cnt = 0
                self.reset_robot()
            if(low_energy_flag < p.readUserDebugParameter(self.low_energy_mode)):
                low_energy_flag = p.readUserDebugParameter(self.low_energy_mode)
                rospy.logwarn("set robot to low energy mode")
                self.cpp_gait_ctrller.set_robot_mode(convert_type(1))
            if(high_performance_flag < p.readUserDebugParameter(self.high_performance_mode)):
                high_performance_flag = p.readUserDebugParameter(self.high_performance_mode)
                rospy.logwarn("set robot to high performance mode")
                self.cpp_gait_ctrller.set_robot_mode(convert_type(0))

            self.run()
            cnt += 1
            if cnt > 99999999:
                cnt = 99999999
            rate.sleep()


if __name__ == '__main__':
    sim_node = QuadSimulator()
    sim_node.main()
