"""
Perception system that processes depth camera images / segmentations and outputs 
the target gripper pose for robust antipodal planar grasp. 
"""
import numpy as np
import open3d

from pydrake.common.value import AbstractValue
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.perception import PointCloud
from pydrake.systems.sensors import PixelType, ImageDepth16U, ImageRgba8U, ImageLabel16I
from pydrake.systems.framework import BasicVector, LeafSystem


NUM_CAMERAS = 3

class ClutterClearingPerception(LeafSystem):
    """
    Perception system that generates robust antipodal grasps given 
    the point cloud data. 

    Although Grasps are represented as full EE pose during pick up. 
    But this is still a planar grasp i.e only yaw angle changes as 
    for different objects and different configurations.

    @system{
        @input_port{point_cloud_FS}, 
        @output_port{rpy_xyz}
    }
    """

    # TODO(darshanhegde): Remove the object groundtruth position as input. 
    # Instead predict the planar grasp using heuristics. 
    def __init__(self, X_WOs=None, time_step=0.005, pick_and_drop_period=16.0):
        """
        @param X_WO is a math::RigidTransform for object to grasp. 
        """
        LeafSystem.__init__(self)
        self.X_WOs = X_WOs
        self.pick_and_drop_period = pick_and_drop_period

        self.DeclareAbstractInputPort("point_cloud_FS", 
                                      AbstractValue.Make(PointCloud()))

        self.DeclareVectorOutputPort("rpy_xyz_object", BasicVector(6), 
                                     self.ComputeGrasp)

        self.DeclarePeriodicDiscreteUpdate(time_step)
        self.DeclareDiscreteState(5)

    def compute_grasp(self, X_WO):
        """
        @param X_WO is a math::RigidTransform for object to grasp. 
        """
        position = X_WO.translation()
        rpy_object = RollPitchYaw(X_WO.rotation())
        rpy_ee = RollPitchYaw([np.pi, 0.025, rpy_object.yaw_angle()])
        return np.concatenate([rpy_ee.vector(), position])

    def ComputeGrasp(self, context, output):
        # Read point cloud and convert to Open3D format. 

        sim_time = context.get_time()

        if sim_time % self.pick_and_drop_period == 0.0:
            if self.X_WOs:
                # Compute the grasp in the beginning and output it every timestep. 
                self.grasp = self.compute_grasp(self.X_WOs.pop())

        output.SetFromVector(self.grasp)
