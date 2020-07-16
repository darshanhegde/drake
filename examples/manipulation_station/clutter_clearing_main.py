"""
Clutter clearing station with grasp synthesis and execution. 
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

from pydrake.geometry.render import RenderLabel
from pydrake.systems.drawing import plot_system_graphviz
from pydrake.examples.manipulation_station import ManipulationStation, CreateClutterClearingYcbObjectList, \
    IiwaCollisionModel
from pydrake.math import RollPitchYaw, RigidTransform
from pydrake.systems.analysis import Simulator
from pydrake.systems.perception import PointCloudConcatenation
from pydrake.systems.sensors import PixelType, CameraInfo
from pydrake.systems.meshcat_visualizer import MeshcatPointCloudVisualizer, MeshcatVisualizer, AddTriad, MeshcatContactVisualizer
from pydrake.perception import DepthImageToPointCloud, Fields, BaseField, kDescriptorLabel
from pydrake.systems.framework import DiagramBuilder, AbstractValue

from drake.examples.manipulation_station.differential_ik import DifferentialIK
from pydrake.manipulation.planner import DifferentialInverseKinematicsParameters

from drake.examples.manipulation_station.clutter_clearing_perception import ClutterClearingPerception
from drake.examples.manipulation_station.clutter_clearing_planner import PickAndDropTrajectoryGenerator


XW_home = RigidTransform(p=np.array([3.53636362e-04, -4.44084375e-01,  0.7]), 
                         rpy=RollPitchYaw([3.14159265,  0.24159265,  1.57159265]))
XW_drop = RigidTransform(p=np.array([0.40316364, -0.11380088, 0.7]), 
                         rpy=RollPitchYaw([3.18059265, -0.33110735, -0.05200735]))


def main():
    # Setup clutter clearing station visualization. 
    builder = DiagramBuilder()

    station = builder.AddSystem(ManipulationStation())
    station.SetupClutterClearingStation(IiwaCollisionModel.kBoxCollision)

    # Add one of the objects to clutter clearing station. 
    ycb_objects = CreateClutterClearingYcbObjectList()

    XW_Objects = []
    for model_file, XW_Object in ycb_objects:
        station.AddManipulandFromFile(model_file, XW_Object)
        XW_Objects.append(XW_Object)

    station.Finalize()

    # Add depth cameras and combine them to produce full scene point clouds. 
    camera_name_list = station.get_camera_names()
    camera_poses = station.GetStaticCameraPosesInWorld()
    camera_info = CameraInfo(**{"width": 848, "height": 480, "fov_y": 0.712439311})
    pc_concat = builder.AddSystem(PointCloudConcatenation(camera_name_list))
    di2pcs = {}
    for camera_name in camera_name_list:
        # Convert depth images to point cloud data. 
        di2pcs[camera_name] = builder.AddSystem(DepthImageToPointCloud(
            camera_info, PixelType.kDepth16U, 1e-3,
            fields=Fields(BaseField.kXYZs | BaseField.kRGBs, kDescriptorLabel)))
        builder.Connect(
            station.GetOutputPort("camera_" + camera_name + "_rgb_image"),
            di2pcs[camera_name].color_image_input_port())
        builder.Connect(
            station.GetOutputPort("camera_" + camera_name + "_depth_image"),
            di2pcs[camera_name].depth_image_input_port())
        builder.Connect(
            station.GetOutputPort("camera_" + camera_name + "_label_image"), 
            di2pcs[camera_name].label_image_input_port())
        builder.Connect(di2pcs[camera_name].point_cloud_output_port(), 
                        pc_concat.GetInputPort("point_cloud_CiSi_{}".format(camera_name)))        

    # initialize meshcat visualizer 
    meshcat = builder.AddSystem(MeshcatVisualizer(
                station.get_scene_graph(), zmq_url="tcp://127.0.0.1:6000"))
    builder.Connect(station.GetOutputPort("pose_bundle"),
                    meshcat.get_input_port(0))

    # Add point cloud visualization
    scene_pc_vis = builder.AddSystem(MeshcatPointCloudVisualizer(meshcat, name="scene_point_cloud", 
                                                                filter_labels=range(14, 20)))
    builder.Connect(pc_concat.GetOutputPort("point_cloud_FS"),
                    scene_pc_vis.GetInputPort("point_cloud_P"))

    # Add viz markers for key locations on the scene. 
    AddTriad(meshcat.vis, "0", prefix="init", radius=0.007, length=0.15)
    meshcat.vis["init"]["0"].set_transform(XW_home.matrix())
    AddTriad(meshcat.vis, "0", prefix="target", radius=0.007, length=0.15)
    meshcat.vis["target"]["0"].set_transform(XW_drop.matrix())
    for i, XW_Object in enumerate(XW_Objects):
        AddTriad(meshcat.vis, str(i), prefix="object", radius=0.007, length=0.15)
        meshcat.vis["object"][str(i)].set_transform(XW_Object.matrix())

    # Add camera markers for visualization. 
    for camera_name in camera_name_list:
        AddTriad(meshcat.vis, camera_name, prefix="cameras", radius=0.007, length=0.15)
        meshcat.vis["cameras"][camera_name].set_transform(camera_poses[camera_name].matrix())

    # Add contact markeers. 
    contact_viz = MeshcatContactVisualizer(meshcat, plant=station.get_mutable_multibody_plant())
    builder.AddSystem(contact_viz)
    builder.Connect(station.GetOutputPort("pose_bundle"), contact_viz.GetInputPort("pose_bundle"))
    builder.Connect(station.GetOutputPort("contact_results"), contact_viz.GetInputPort("contact_results"))

    pick_and_drop_period = 9.0
    # Add clutter clearing perception system 
    perception = builder.AddSystem(ClutterClearingPerception(time_step=0.05, 
                                                             pick_and_drop_period=pick_and_drop_period))
    builder.Connect(pc_concat.GetOutputPort("point_cloud_FS"), 
                    perception.GetInputPort("point_cloud_FS"))

    # Add Pick and Drop Trajectory generator. 
    traj_gen = builder.AddSystem(PickAndDropTrajectoryGenerator(XW_home=XW_home, XW_drop=XW_drop, 
                                                                pick_and_drop_period=pick_and_drop_period))

    # Connect perception to trajectory generator. 
    builder.Connect(perception.GetOutputPort("rpy_xyz_object"), 
                    traj_gen.GetInputPort("rpy_xyz_object"))

    # Add IK solver 
    robot = station.get_controller_plant()
    params = DifferentialInverseKinematicsParameters(robot.num_positions(), robot.num_velocities())

    time_step = 0.05
    params.set_timestep(time_step)
    # True velocity limits for the IIWA14 (in rad / sec, rounded down to the first decimal)
    iiwa14_velocity_limits = np.array([1.4, 1.4, 1.7, 1.3, 2.2, 2.3, 2.3])
    # Stay within a small fraction of those limits for this demo.
    factor = 1.0
    params.set_joint_velocity_limits((-factor*iiwa14_velocity_limits,
                                      factor*iiwa14_velocity_limits))

    differential_ik = builder.AddSystem(DifferentialIK(
        robot, robot.GetFrameByName("iiwa_link_7"), params, time_step))


    # Connect IK and Trajectory Generator to Iiwa robot control ports.     
    builder.Connect(traj_gen.GetOutputPort("rpy_xyz_desired"), 
                    differential_ik.GetInputPort("rpy_xyz_desired"))

    builder.Connect(differential_ik.GetOutputPort("joint_position_desired"),
                    station.GetInputPort("iiwa_position"))

    builder.Connect(traj_gen.GetOutputPort("gripper_position"), 
                    station.GetInputPort("wsg_position"))

    builder.Connect(traj_gen.GetOutputPort("force_limit"),
                    station.GetInputPort("wsg_force_limit"))

    diagram = builder.Build()
    simulator = Simulator(diagram)

    # Set camera extrinsics needed for concatenating the point cloud. 
    pc_concat_context = diagram.GetMutableSubsystemContext(pc_concat, simulator.get_mutable_context())
    for camera_name in camera_name_list:
        X_WP = camera_poses[camera_name]
        pc_concat_context.FixInputPort(
            pc_concat.GetInputPort("X_FCi_{}".format(camera_name)).get_index(),
            AbstractValue.Make(X_WP))

    station_context = diagram.GetMutableSubsystemContext(station, simulator.get_mutable_context())

    # Initial joint angles of the robot and for IK solver. 
    q0 = station.GetOutputPort("iiwa_position_measured").Eval(station_context)
    differential_ik.parameters.set_nominal_joint_position(q0)
    differential_ik.SetPositions(diagram.GetMutableSubsystemContext(
        differential_ik, simulator.get_mutable_context()), q0)

    # Set feedforward_torque to zero since we are doing position based control. 
    station_context.FixInputPort(
        station.GetInputPort("iiwa_feedforward_torque").get_index(),
        np.zeros(7))
    
    # Initialize and run the simulator. 
    simulator.set_publish_every_time_step(False)
    simulator.set_target_realtime_rate(0.0)  
    simulator.Initialize()
    # simulator.AdvanceTo(1.0)
    simulator.AdvanceTo(len(ycb_objects) * pick_and_drop_period + 1.0)


if __name__ == '__main__':
    main()
