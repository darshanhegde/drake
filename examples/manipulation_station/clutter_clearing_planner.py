"""
Planning system that executes the pick and place open loop. 
"""
import numpy as np
from pydrake.multibody import inverse_kinematics
from pydrake.trajectories import PiecewisePolynomial

from pydrake.common.eigen_geometry import Quaternion
from pydrake.math import RollPitchYaw, RotationMatrix, RigidTransform, Slerp
from pydrake.examples.manipulation_station import ManipulationStation
from pydrake.solvers.mathematicalprogram import SolutionResult
import pydrake.solvers.mathematicalprogram as mp

from pydrake.systems.framework import BasicVector, LeafSystem
from pydrake.systems.framework import (BasicVector, LeafSystem, PortDataType,
    AbstractValue, LeafSystem, PublishEvent, TriggerType)
from drake.examples.manipulation_station.robot_plans import *
from drake.examples.manipulation_station.plan_utils import ConnectPointsWithCubicPolynomial


def get_ee_interpolator(init_pose, final_pose):
    """
    Uses a linear interpolation for positions and slerp for rotations 
    to interpolate ee pose from init_pose to final_pose. 
    """
    init_position = init_pose.translation()
    final_position = final_pose.translation()

    R_init = init_pose.rotation()
    R_final = final_pose.rotation()

    Q_init = R_init.ToQuaternion().wxyz()
    Q_final = R_final.ToQuaternion().wxyz()

    def ee_interpolator(t):
        """
        :param t: [0.0, 1.0]
        """
        assert 0.0 <= t <= 1.0, "Interpolation time step should be in [0.0, 1.0]."
        p_t = (1 - t) * init_position +  t * final_position
        rpy_t = RollPitchYaw(Quaternion(Slerp(Q_init, Q_final, t))) 
        return p_t, rpy_t

    return ee_interpolator


class PickAndDropTrajectoryGenerator(LeafSystem):

    def __init__(self, XW_home, XW_approach, XW_pick, XW_drop, start_time=1.0, end_time=11.0):
        LeafSystem.__init__(self)

        self.DeclareVectorOutputPort("rpy_xyz", BasicVector(6), 
                                     self.DoCalcPose)
        self.DeclareVectorOutputPort("gripper_position", BasicVector(1), 
                                     self.CalcGripperPosition)
        self.DeclareVectorOutputPort("force_limit", BasicVector(1), 
                                     self.CalcForceLimitOutput)

        self.DeclarePeriodicDiscreteUpdate(0.05, 0.0)

        self.start_time = start_time
        self.approach_time = start_time + 2.0
        self.pick_start_time = start_time + 3.0
        self.pick_end_time = start_time + 4.0
        self.home_time = start_time + 6.0
        self.drop_start_time = start_time + 9.0
        self.drop_end_time = start_time + 10.0
        self.back_home_time = start_time + 12.0
        self.end_time = end_time

        self.gripper_max = 0.107
        self.gripper_min = 0.001
        self.gripper_goal = self.gripper_max

        self.goto_approach_interpolator = get_ee_interpolator(XW_home, XW_approach)
        self.goto_pick_interpolator = get_ee_interpolator(XW_approach, XW_pick)
        self.goto_home_interpolator = get_ee_interpolator(XW_pick, XW_home)
        self.goto_drop_interpolator = get_ee_interpolator(XW_home, XW_drop)
        self.goback_home_interpolator = get_ee_interpolator(XW_drop, XW_home)

    def CalcGripperPosition(self, context, output):
        sim_time = context.get_time()
        if self.pick_start_time < sim_time <= self.pick_end_time: 
            self.gripper_goal = self.gripper_min
        elif self.drop_start_time < sim_time <= self.drop_end_time:
            self.gripper_goal = self.gripper_max
        elif sim_time > self.drop_end_time:
            self.gripper_goal = self.gripper_max

        output.SetAtIndex(0, self.gripper_goal)

    def CalcForceLimitOutput(self, context, output):
        self._force_limit = 15
        output.SetAtIndex(0, self._force_limit)

    def DoCalcPose(self, context, output):
        sim_time = context.get_time()

        if sim_time <= self.start_time:
            p, rpy = self.goto_approach_interpolator(0.0)
        elif self.start_time < sim_time <= self.approach_time:
            print("go to appraoch")
            t = (sim_time - self.start_time) / (self.approach_time - self.start_time)
            p, rpy = self.goto_approach_interpolator(t)
        elif self.approach_time < sim_time <= self.pick_start_time:
            print("go to pick")
            t = (sim_time - self.approach_time) / (self.pick_start_time - self.approach_time)
            p, rpy = self.goto_pick_interpolator(t)
        elif self.pick_start_time < sim_time <= self.pick_end_time:
            print("picking")
            p, rpy = self.goto_pick_interpolator(1.0)
        elif self.pick_end_time < sim_time <= self.home_time:
            print("go to home")
            t = (sim_time - self.pick_end_time) / (self.home_time - self.pick_end_time)
            p, rpy = self.goto_home_interpolator(t)
        elif self.home_time < sim_time <= self.drop_start_time:
            print("go to drop")
            t = (sim_time - self.home_time) / (self.drop_start_time - self.home_time)
            p, rpy = self.goto_drop_interpolator(t)
        elif self.drop_start_time < sim_time <= self.drop_end_time:
            print("dropping")
            t = (sim_time - self.drop_start_time) / (self.drop_start_time - self.drop_end_time)
            p, rpy = self.goto_drop_interpolator(1.0)
        elif self.drop_end_time < sim_time <= self.back_home_time:
            print("back home")
            t = (sim_time - self.drop_end_time) / (self.back_home_time - self.drop_end_time)
            p, rpy = self.goback_home_interpolator(t)
        elif sim_time > self.back_home_time:
            p, rpy = self.goback_home_interpolator(1.0)

        output.SetAtIndex(0, rpy.roll_angle())
        output.SetAtIndex(1, rpy.pitch_angle())
        output.SetAtIndex(2, rpy.yaw_angle())
        output.SetAtIndex(3, p[0])
        output.SetAtIndex(4, p[1])
        output.SetAtIndex(5, p[2])
