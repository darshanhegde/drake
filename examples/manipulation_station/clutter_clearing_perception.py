"""
Perception system that processes depth camera images / segmentations and outputs 
the target gripper pose for robust antipodal planar grasp. 
"""
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation

from pydrake.common import FindResourceOrThrow
from pydrake.common.value import AbstractValue
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.perception import PointCloud, BaseField, Fields, kDescriptorLabel
from pydrake.systems.sensors import PixelType, ImageDepth16U, ImageRgba8U, ImageLabel16I
from pydrake.systems.framework import BasicVector, LeafSystem

cracker_box_mesh = FindResourceOrThrow("drake/manipulation/models/ycb/meshes/003_cracker_box_textured.obj")
sugar_box_mesh = FindResourceOrThrow("drake/manipulation/models/ycb/meshes/004_sugar_box_textured.obj")
tomato_soup_can_mesh = FindResourceOrThrow("drake/manipulation/models/ycb/meshes/005_tomato_soup_can_textured.obj")
mustard_bottle_mesh = FindResourceOrThrow("drake/manipulation/models/ycb/meshes/006_mustard_bottle_textured.obj")
gelatin_box_mesh = FindResourceOrThrow("drake/manipulation/models/ycb/meshes/009_gelatin_box_textured.obj")
potted_meat_can_mesh = FindResourceOrThrow("drake/manipulation/models/ycb/meshes/010_potted_meat_can_textured.obj")


SEGMENTATION_LABELS_TO_MESHES = {
    14: (cracker_box_mesh, RigidTransform(p=np.array([-0.014, 0.103, 0.013]), rpy=RollPitchYaw([1.57, -1.57, 0]))), 
    15: (sugar_box_mesh, RigidTransform(p=np.array([-0.018, 0.088, 0.0039]), rpy=RollPitchYaw([-0.77, -1.52, 2.36]))),  
    16: (tomato_soup_can_mesh, RigidTransform(p=np.array([-0.0018, 0.051, -0.084]), rpy=RollPitchYaw([1.57, 0.13, 0.0]))), 
    17: (mustard_bottle_mesh, RigidTransform(p=np.array([0.0049, 0.092, 0.027]), rpy=RollPitchYaw([1.57, -0.40, 0.0]))), 
    18: (gelatin_box_mesh, RigidTransform(p=np.array([-0.0029, 0.024, -0.015]), rpy=RollPitchYaw([-0.0085, -0.002, 1.34]))), 
    19: (potted_meat_can_mesh, RigidTransform(p=np.array([0.034, 0.039, 0.025]), rpy=RollPitchYaw([1.57, 0.052, 0.0])))
}


def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=150))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh


def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, distance_threshold,
        o3d.registration.TransformationEstimationPointToPoint(False), 4, [
            o3d.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.registration.RANSACConvergenceCriteria(4000000, 500))
    return result


def refine_registration(source, target, transform_init, voxel_size):
    distance_threshold = voxel_size * 0.25
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    result = o3d.registration.registration_icp(
        source, target, distance_threshold, transform_init,
        o3d.registration.TransformationEstimationPointToPlane())
    return result


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
    def __init__(self, time_step=0.005, pick_and_drop_period=1.0):
        """
        @param X_WO is a math::RigidTransform for object to grasp. 
        """
        LeafSystem.__init__(self)
        self.pick_and_drop_period = pick_and_drop_period

        output_fields = Fields(BaseField.kXYZs | BaseField.kRGBs, kDescriptorLabel)

        self.DeclareAbstractInputPort("point_cloud_FS", 
                                      AbstractValue.Make(PointCloud(fields=output_fields)))

        self.DeclareVectorOutputPort("rpy_xyz_object", BasicVector(6), 
                                     self.ComputeGrasp)

        self.DeclarePeriodicDiscreteUpdate(time_step)
        self.DeclareDiscreteState(5)

        # Read mesh files and convert to downsampled point cloud for ICP. 
        self.seg_label_and_pc = []
        for label, (mesh_file, object_trans) in SEGMENTATION_LABELS_TO_MESHES.items():
            mesh = o3d.io.read_triangle_mesh(mesh_file)
            object_pc = mesh.sample_points_uniformly(number_of_points=10000)
            self.seg_label_and_pc.append((label, object_pc, object_trans))

    def compute_grasp(self, X_WO):
        """
        @param X_WO is a math::RigidTransform for object to grasp. 
        """
        position = X_WO.translation()
        rpy_object = RollPitchYaw(X_WO.rotation())
        rpy_ee = RollPitchYaw([-np.pi, 0.025, rpy_object.yaw_angle()])
        return np.concatenate([rpy_ee.vector(), position])

    def ComputeGrasp(self, context, output):
        sim_time = context.get_time()

        if sim_time % self.pick_and_drop_period == 0.0:
            if self.seg_label_and_pc:
                # Read point cloud and convert to Open3D format. 
                point_cloud = self.GetInputPort("point_cloud_FS").Eval(context)
                label, source_obj_pc, source_obj_trans = self.seg_label_and_pc.pop(0)
                pc_seg_mask = (point_cloud.descriptors()[0] == label)
                scene_obj_pc_xyzs =  np.copy(point_cloud.xyzs()[:, pc_seg_mask])
                scene_obj_pc = o3d.geometry.PointCloud()
                scene_obj_pc.points = o3d.utility.Vector3dVector(scene_obj_pc_xyzs.T)

                voxel_size = 0.005
                source, target = source_obj_pc, scene_obj_pc
                source.transform(source_obj_trans.matrix())
                print("source: ", source)
                print("target: ", target)
                source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
                target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
                print("source_down: ", source_down)
                print("target_down: ", target_down)

                result_global = execute_global_registration(source_down, target_down,
                                                            source_fpfh, target_fpfh,
                                                            voxel_size)
                print(result_global)
                transformation = result_global.transformation
                R_estimate = Rotation.from_matrix(transformation[:3, :3])

                print("fast translation_estimate: ", transformation[:-1, -1])
                print("fast rotation estimate: ", R_estimate.as_euler('xyz'))

                result_refine = refine_registration(source, target, transformation, voxel_size)

                print(result_refine)
                transformation = result_refine.transformation
                R_estimate = Rotation.from_matrix(transformation[:3, :3])

                print("refine translation_estimate: ", transformation[:-1, -1])
                print("refine rotation estimate: ", R_estimate.as_euler('xyz'))

                X_WO = RigidTransform(p=transformation[:-1, -1], rpy=RollPitchYaw(R_estimate.as_euler('xyz')))
                print("X_WO_estimate: ", X_WO.translation(), np.rad2deg(RollPitchYaw(X_WO.rotation()).vector()))
                self.grasp = self.compute_grasp(X_WO)

        output.SetFromVector(self.grasp)
