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

    def __init__(self, XW_home, XW_drop, pick_and_drop_period):
        LeafSystem.__init__(self)

        self.DeclareInputPort("rpy_xyz_object", PortDataType.kVectorValued, 6)

        self.DeclareVectorOutputPort("rpy_xyz_desired", BasicVector(6), 
                                     self.DoCalcPose)
        self.DeclareVectorOutputPort("gripper_position", BasicVector(1), 
                                     self.CalcGripperPosition)
        self.DeclareVectorOutputPort("force_limit", BasicVector(1), 
                                     self.CalcForceLimitOutput)

        self.DeclarePeriodicDiscreteUpdate(0.05, 0.0)

        self.start_time = 1.0
        self.approach_time = self.start_time + 2.0
        self.pick_start_time = self.start_time + 3.0
        self.pick_end_time = self.start_time + 3.5
        self.home_time = self.start_time + 6.0
        self.drop_start_time = self.start_time + 8.0
        self.drop_end_time = self.start_time + 8.5
        self.back_home_time = self.start_time + 10.0

        self.pick_and_drop_period = pick_and_drop_period

        self.gripper_max = 0.107
        self.gripper_min = 0.001
        self.gripper_goal = self.gripper_max

        self.XW_home = XW_home
        self.XW_drop = XW_drop
        self.XW_approach = None
        self.XW_pick = None
        self.approach_offset = np.array([0.0, 0.0, 0.2])
        

    def CalcGripperPosition(self, context, output):
        sim_time = context.get_time() % self.pick_and_drop_period

        if self.pick_start_time < sim_time <= self.pick_end_time: 
            t = (sim_time - self.pick_start_time) / (self.pick_end_time - self.pick_start_time)
            self.gripper_goal = (1 - t) * self.gripper_max + t * self.gripper_min
        elif self.drop_start_time < sim_time <= self.drop_end_time:
            t = (sim_time - self.drop_start_time) / (self.drop_end_time - self.drop_start_time)
            self.gripper_goal = (1 - t) * self.gripper_min + t * self.gripper_max
        elif sim_time > self.drop_end_time:
            self.gripper_goal = self.gripper_max

        self.gripper_goal = np.clip(self.gripper_goal, a_max=self.gripper_max, a_min=self.gripper_min)
        output.SetAtIndex(0, self.gripper_goal)

    def CalcForceLimitOutput(self, context, output):
        self._force_limit = 50
        output.SetAtIndex(0, self._force_limit)

    def DoCalcPose(self, context, output):
        sim_time = context.get_time() % self.pick_and_drop_period

        if self.XW_pick is None or sim_time == 0.0:
            rpy_xyz_object = np.copy(self.EvalVectorInput(context, 0).get_value())
            # gaurd against z-axis colliding with the gripper
            rpy_xyz_object[5] = np.maximum(rpy_xyz_object[5], 0.28)
            self.XW_pick = RigidTransform(p=rpy_xyz_object[3:], 
                                          rpy=RollPitchYaw(rpy_xyz_object[:3]))
            self.XW_approach = RigidTransform(p=rpy_xyz_object[3:] + self.approach_offset, 
                                              rpy=RollPitchYaw(rpy_xyz_object[:3]))
            self.goto_approach_interpolator = get_ee_interpolator(self.XW_home, self.XW_approach)
            self.goto_pick_interpolator = get_ee_interpolator(self.XW_approach, self.XW_pick)
            self.goto_home_interpolator = get_ee_interpolator(self.XW_pick, self.XW_home)
            self.goto_drop_interpolator = get_ee_interpolator(self.XW_home, self.XW_drop)
            self.goback_home_interpolator = get_ee_interpolator(self.XW_drop, self.XW_home)

        if sim_time <= self.start_time:
            p, rpy = self.XW_home.translation(), RollPitchYaw(self.XW_home.rotation())
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

        rpy_xyz = np.concatenate([rpy.vector(), p])
        output.SetFromVector(rpy_xyz)
