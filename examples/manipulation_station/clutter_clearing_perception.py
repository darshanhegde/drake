"""
Perception system that processes depth camera images / segmentations and outputs 
the target gripper pose for robust antipodal planar grasp. 
"""
import numpy as np

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

    Grasps as represented as planar graps: (x, y, z, w, yaw)

    (x, y, z): Position target for center of the gripper. 
    (w)      : Width of the gripper.
    (yaw)    : Yaw of the gripper. 

    @system{
        @input_port{point_cloud_FS}, 
        @output_port{xyz_w_yaw}
    }
    """

    # TODO(darshanhegde): Remove the object groundtruth position as input. 
    # Instead predict the planar grasp using heuristics. 
    def __init__(self, X_WO=None, time_step=0.005):
        """
        @param X_WO is a math::RigidTransform for object to grasp. 
        """
        LeafSystem.__init__(self)
        self.X_WO = X_WO

        self.DeclareAbstractInputPort("point_cloud_FS", 
                                      AbstractValue.Make(PointCloud()))

        self.DeclareVectorOutputPort("xyz_w_yaw", BasicVector(5), 
                                     self.ComputeGrasp)

        self.DeclarePeriodicDiscreteUpdate(time_step)
        self.DeclareDiscreteState(5)

        # Compute the grasp in the beginning and output it every timestep. 
        self.grasp = self.compute_grasp(X_WO)

    def compute_grasp(self, X_WO):
        """
        @param X_WO is a math::RigidTransform for object to grasp. 
        """
        position = X_WO.translation()
        width = 0.05
        yaw = RollPitchYaw(X_WO.rotation()).yaw_angle() - (np.pi / 2)
        return np.concatenate([position, [width], [yaw]])

    def ComputeGrasp(self, context, output):
        output.SetFromVector(self.grasp)
