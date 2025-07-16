import numpy as np
from rb5_action_manager.src.robot.rb5_wrapper import rb5Wrapper
from rb5_action_manager.src.trajectory.trajManager import jointTraj, SE3Traj
from rb5_action_manager.src.solver.qp_solver import RCI_QP_Solver
from rclpy.node import Node

from rclpy.logging import get_logger

import pinocchio as pin
from pinocchio import RobotWrapper
from pinocchio.utils import *

import time

from copy import deepcopy


class rb5Controller:
    def __init__(self, robot : rb5Wrapper, clock):
        self.robot = robot
        self.clock = clock

        # Control : Time
        self.stime = None
        self.ctime = None
        self.duration  = None

        # Control : Target
        self.targetJoint = None
        self.targetSE3 = pin.SE3()

        # Robot Data
        self.nq = self.robot.state.nq

        # Trajectory
        self.jointTraj = jointTraj(self.nq)
        self.se3Traj = SE3Traj()

        # solver
        self.qpSolver = RCI_QP_Solver(self.nq)

        # State : Result
        self.qdes : np.array

        # isControl
        self.controlFlag = False

        # qTmp 
        self.qTmp : np.array

        self.logger = get_logger("Controller")
        
        

        # TEST 250518 #
        self.initOmni = None
        self.initRb5 = None


    def initJointPosture(self, target : np.array, duration):
        self.controlFlag = True
        self.stime = self.clock.now().nanoseconds/1e9
        self.duration = duration
        self.targetJoint = np.copy(target)

        self.jointTraj.setStartTime(self.stime)
        self.jointTraj.setDuration(self.duration)
        self.jointTraj.setTargetSample(self.targetJoint)
        self.jointTraj.setInitSample(self.robot.state.q)



    def controlJointPosture(self):
        self.ctime = self.clock.now().nanoseconds/1e9
        self.jointTraj.setCurrentTime(self.ctime)
        qdes = self.jointTraj.computeNext()
        self.qdes = np.copy(qdes)

    def initSE3(self, target : pin.SE3, duration):
        self.controlFlag = True

        self.stime = self.clock.now().nanoseconds/1e9
        self.duration = duration
        self.targetSE3 = target

        self.se3Traj.setStartTime(self.stime)
        self.se3Traj.setDuration(self.duration)
        self.se3Traj.setTargetSample(self.targetSE3)
        self.se3Traj.setInitSample(self.robot.state.oMi)

        self.qTmp = np.copy(self.robot.state.q)


    def controlSe3(self):
        # self.ctime = self.clock.now().nanoseconds/1e9
        # self.se3Traj.setCurrentTime(self.ctime)
        # se3Ref = self.se3Traj.computeNext()
        se3Ref = self.targetSE3
        print("Target \n",se3Ref)
        oMi = self.robot.state.oMi
        dMi = oMi.inverse() * se3Ref
        dMi_6d_motion = pin.log(dMi)
        dMi_6d = np.zeros((6))
        dMi_6d[:3] = dMi_6d_motion.linear
        dMi_6d[3:] = dMi_6d_motion.angular
        
        
        # print(dMi_6d)
        
        p_gain = 10.0
        qdot = np.linalg.pinv(self.robot.state.J) @ (p_gain * dMi_6d)
        qdes = self.qTmp + qdot * 0.01

        # TEST
        # P = np.eye((self.nq))
        # q = np.zeros((self.nq))
        # A = np.copy(self.robot.state.J)
        # b = np.copy(p_gain * dMi_6d)

        # self.qpSolver.AddCost(P,q)
        # self.qpSolver.AddEqTask(A,b)

        # sol = self.qpSolver.solveQP()        
        # qdes = self.robot.state.q + sol * 0.01
        self.qdes = np.copy(qdes)
        self.qTmp = np.copy(qdes)



    def adjoint_matrix(self, R, p):
        px = np.array([
            [0, -p[2], p[1]],
            [p[2], 0, -p[0]],
            [-p[1], p[0], 0]
        ])
        upper = np.hstack((R, np.zeros((3, 3))))
        lower = np.hstack((px @ R, R))
        Ad = np.vstack((upper, lower))
        return Ad


    def init_teleop(self, omni):
        # self.logger.info(f"Init Omni : {omni}" )
        self.initOmni = deepcopy(omni)
        self.initRb5 = deepcopy(self.robot.state.oMi)
        


    # Frame 변환 해야함
    def teleop_Control(self, se3):
        tmp = pin.SE3()
        tmp = deepcopy(se3)

        # self.logger.info(f"Init Omni : {se3}" )


        trans_diff = tmp.translation - self.initOmni.translation
        rot_diff = self.initOmni.rotation.T @ tmp.rotation
        # self.logger.info(f"rot_diff: {rot_diff}")


        self.target = deepcopy(self.initRb5)
        self.target.translation += trans_diff * 1.8
        self.target.rotation = self.initRb5.rotation @ rot_diff



        currentRb5 = deepcopy(self.robot.state.oMi)
        # self.logger.info(f"init RB5 : {self.initRb5}")
        # self.logger.info(f"CURRENT RB5 : {currentRb5}")
        # self.logger.info(f"Target RB5 : {self.target}")
        
        dMi = currentRb5.inverse() * self.target
        err_6d = pin.log(dMi)
        dMi_6d = np.zeros((6))
        dMi_6d[:3] = err_6d.linear
        dMi_6d[3:] = err_6d.angular        
        # self.logger.warn(f"err : {dMi_6d}")

        des_vel =  1.0 * dMi_6d
        # self.logger.info(f"Jacobian : {np.linalg.det(self.robot.state.J)}")
        des_qdot = np.linalg.pinv(self.robot.state.J) @ des_vel

        # self.logger.warn(f"Trans Diff : {des_qdot}")


        # tmpAdj = self.adjoint_matrix(tmp.rotation, tmp.)


        # base = pin.SE3()
        # base.translation = np.array([0,0,0])



        # tmp2 = AdjMat * tmp
        # self.logger.info(f"Tmp1 : {tmp}")

        # self.logger.info(f"Tmp2 : {tmp2}")
        # self.logger.info(f"Adj : {AdjMat}")


        return self.robot.state.q + des_qdot * 0.01
