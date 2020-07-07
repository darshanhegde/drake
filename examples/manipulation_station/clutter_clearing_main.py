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
from pydrake.perception import DepthImageToPointCloud, BaseField
from pydrake.systems.framework import DiagramBuilder, AbstractValue

from drake.examples.manipulation_station.differential_ik import DifferentialIK
from pydrake.manipulation.planner import DifferentialInverseKinematicsParameters

from drake.examples.manipulation_station.clutter_clearing_perception import ClutterClearingPerception
from drake.examples.manipulation_station.clutter_clearing_planner import PickAndDropTrajectoryGenerator
from drake.examples.manipulation_station.robot_plans import JointSpacePlan
from drake.examples.manipulation_station.plan_utils import GetPlanStartingTimes

reserved_labels = [
    RenderLabel.kDoNotRender,
    RenderLabel.kDontCare,
    RenderLabel.kEmpty,
    RenderLabel.kUnspecified,
]


def colorize_labels(image):
    """Colorizes labels."""
    # TODO(eric.cousineau): Revive and use Kuni's palette.
    cc = mpl.colors.ColorConverter()
    color_cycle = plt.rcParams["axes.prop_cycle"]
    colors = np.array([cc.to_rgb(c["color"]) for c in color_cycle])
    bg_color = [0, 0, 0]
    image = np.squeeze(image)
    background = np.zeros(image.shape[:2], dtype=bool)
    for label in reserved_labels:
        background |= image == int(label)
    color_image = colors[image % len(colors)]
    color_image[background] = bg_color
    return color_image


XW_home = RigidTransform(p=np.array([3.53636362e-04, -4.44084375e-01,  7.66707814e-01]), 
                         rpy=RollPitchYaw([3.14159265,  0.24159265,  1.57159265]))
XW_drop = RigidTransform(p=np.array([0.40316364, -0.11380088, 0.76670781]), 
                         rpy=RollPitchYaw([3.18059265, -0.33110735, -0.05200735]))

# XW_approach and XW_pick needs to be generated from perception system. 
XW_approach = RigidTransform(p=np.array([-0.15, -0.62,  0.65]), 
                             rpy=RollPitchYaw([3.14159265,  0.0,  2.563]))
XW_pick = RigidTransform(p=np.array([-0.15, -0.62,  0.5]),
                         rpy=RollPitchYaw([3.14159265,  0.0,  2.563]))


def main():
    builder = DiagramBuilder()

    station = builder.AddSystem(ManipulationStation())
    station.SetupClutterClearingStation(IiwaCollisionModel.kBoxCollision)

    # Add one of the objects to clutter clearing station. 
    ycb_objects = CreateClutterClearingYcbObjectList()
    model_file, XW_Object = ycb_objects[-1]
    station.AddManipulandFromFile(model_file, XW_Object)
    station.Finalize()

    camera_name_list = station.get_camera_names()
    camera_poses = station.GetStaticCameraPosesInWorld()

    pc_concat = builder.AddSystem(PointCloudConcatenation(camera_name_list))

    camera_info = CameraInfo(**{"width": 848, "height": 480, "fov_y": 0.712439311})
    di2pcs = {}
    for camera_name in camera_name_list:
        # Convert depth images to point cloud data. 
        di2pcs[camera_name] = builder.AddSystem(DepthImageToPointCloud(
            camera_info, PixelType.kDepth16U, 1e-3,
            fields=BaseField.kXYZs | BaseField.kRGBs))
        builder.Connect(
            station.GetOutputPort("camera_" + camera_name + "_rgb_image"),
            di2pcs[camera_name].color_image_input_port())
        builder.Connect(
            station.GetOutputPort("camera_" + camera_name + "_depth_image"),
            di2pcs[camera_name].depth_image_input_port())
        builder.Connect(di2pcs[camera_name].point_cloud_output_port(), 
                        pc_concat.GetInputPort("point_cloud_CiSi_{}".format(camera_name)))        

    meshcat = builder.AddSystem(MeshcatVisualizer(
                station.get_scene_graph(), zmq_url="tcp://127.0.0.1:6000"))
    builder.Connect(station.GetOutputPort("pose_bundle"),
                    meshcat.get_input_port(0))

    scene_pc_vis = builder.AddSystem(MeshcatPointCloudVisualizer(meshcat, name="scene_point_cloud"))
    builder.Connect(pc_concat.GetOutputPort("point_cloud_FS"),
                    scene_pc_vis.GetInputPort("point_cloud_P"))

    # Add clutter clearing perception system 
    perception = builder.AddSystem(ClutterClearingPerception(time_step=0.0, X_WO=XW_Object))
    builder.Connect(pc_concat.GetOutputPort("point_cloud_FS"), 
                    perception.GetInputPort("point_cloud_FS"))

    AddTriad(meshcat.vis, "0", prefix="init", radius=0.007, length=0.15)
    meshcat.vis["init"]["0"].set_transform(XW_home.matrix())
    AddTriad(meshcat.vis, "0", prefix="target", radius=0.007, length=0.15)
    meshcat.vis["target"]["0"].set_transform(XW_drop.matrix())
    AddTriad(meshcat.vis, "0", prefix="approach", radius=0.007, length=0.15)
    meshcat.vis["approach"]["0"].set_transform(XW_approach.matrix())
    AddTriad(meshcat.vis, "0", prefix="pick", radius=0.007, length=0.15)
    meshcat.vis["pick"]["0"].set_transform(XW_pick.matrix())

    # p_inter, rpy_inter = get_ee_interpolators(XW_home, XW_drop)
    # for t in np.linspace(0.0, .99, 10):
    #     p = p_inter(t)
    #     rpy = rpy_inter(t)
    #     XW = RigidTransform(p=p, rpy=rpy)
    #     AddTriad(meshcat.vis, str(t), prefix="target", radius=0.007, length=0.15)
    #     meshcat.vis["target"][str(t)].set_transform(XW.matrix())

    for camera_name in camera_name_list:
        AddTriad(meshcat.vis, camera_name, prefix="cameras", radius=0.007, length=0.15)
        meshcat.vis["cameras"][camera_name].set_transform(camera_poses[camera_name].matrix())

    contact_viz = MeshcatContactVisualizer(meshcat, plant=station.get_mutable_multibody_plant())
    builder.AddSystem(contact_viz)
    builder.Connect(station.GetOutputPort("pose_bundle"), contact_viz.GetInputPort("pose_bundle"))
    builder.Connect(station.GetOutputPort("contact_results"), contact_viz.GetInputPort("contact_results"))

    robot = station.get_controller_plant()
    params = DifferentialInverseKinematicsParameters(robot.num_positions(), robot.num_velocities())

    time_step = 0.05
    params.set_timestep(time_step)
    # True velocity limits for the IIWA14 (in rad, rounded down to the first
    # decimal)
    iiwa14_velocity_limits = np.array([1.4, 1.4, 1.7, 1.3, 2.2, 2.3, 2.3])
    # Stay within a small fraction of those limits for this teleop demo.
    factor = 1.0
    params.set_joint_velocity_limits((-factor*iiwa14_velocity_limits,
                                      factor*iiwa14_velocity_limits))

    differential_ik = builder.AddSystem(DifferentialIK(
        robot, robot.GetFrameByName("iiwa_link_7"), params, time_step))

    builder.Connect(differential_ik.GetOutputPort("joint_position_desired"),
                    station.GetInputPort("iiwa_position"))

    traj_gen = builder.AddSystem(PickAndDropTrajectoryGenerator(XW_home=XW_home, XW_approach=XW_approach, 
                                                                XW_pick=XW_pick, XW_drop=XW_drop, start_time=1.0, end_time=7.0))

    builder.Connect(traj_gen.GetOutputPort("rpy_xyz"), 
                    differential_ik.GetInputPort("rpy_xyz_desired"))

    builder.Connect(traj_gen.GetOutputPort("gripper_position"), 
                    station.GetInputPort("wsg_position"))
    builder.Connect(traj_gen.GetOutputPort("force_limit"),
                    station.GetInputPort("wsg_force_limit"))


    diagram = builder.Build()
    simulator = Simulator(diagram)

    pc_concat_context = diagram.GetMutableSubsystemContext(pc_concat, simulator.get_mutable_context())
    for camera_name in camera_name_list:
        X_WP = camera_poses[camera_name]
        pc_concat_context.FixInputPort(
            pc_concat.GetInputPort("X_FCi_{}".format(camera_name)).get_index(),
            AbstractValue.Make(X_WP))

    station_context = diagram.GetMutableSubsystemContext(station, simulator.get_mutable_context())

    # Initial joint angles of the robot.
    q0 = station.GetOutputPort("iiwa_position_measured").Eval(station_context)
    differential_ik.parameters.set_nominal_joint_position(q0)
    differential_ik.SetPositions(diagram.GetMutableSubsystemContext(
        differential_ik, simulator.get_mutable_context()), q0)

    # Set initial state of the robot.
    station_context.FixInputPort(
        station.GetInputPort("iiwa_feedforward_torque").get_index(),
        np.zeros(7))
    
    simulator.set_publish_every_time_step(False)
    simulator.set_target_realtime_rate(0.0)  # go as fast as possible

    # # calculate starting time for all plans.
    # t_plan = GetPlanStartingTimes(iiwa_plans)
    # extra_time = 1.0
    # sim_duration = t_plan[-1] + extra_time

    simulator.Initialize()
    simulator.AdvanceTo(14.0)

    # station_context = diagram.GetMutableSubsystemContext(station, simulator.get_mutable_context())
    
    # color_0_image = station.GetOutputPort("camera_0_rgb_image").Eval(station_context)
    # depth_0_image = station.GetOutputPort("camera_0_depth_image").Eval(station_context)
    # label_0_image = station.GetOutputPort("camera_0_label_image").Eval(station_context)

    # color_1_image = station.GetOutputPort("camera_1_rgb_image").Eval(station_context)
    # depth_1_image = station.GetOutputPort("camera_1_depth_image").Eval(station_context)
    # label_1_image = station.GetOutputPort("camera_1_label_image").Eval(station_context)

    # color_2_image = station.GetOutputPort("camera_2_rgb_image").Eval(station_context)
    # depth_2_image = station.GetOutputPort("camera_2_depth_image").Eval(station_context)
    # label_2_image = station.GetOutputPort("camera_2_label_image").Eval(station_context)

    # fig, axis = plt.subplots(3, 3)
    # axis[0, 0].imshow(color_0_image.data)
    # axis[0, 1].imshow(np.squeeze(depth_0_image.data))
    # axis[0, 2].imshow(np.squeeze(colorize_labels(label_0_image.data)))
    # axis[1, 0].imshow(color_1_image.data)
    # axis[1, 1].imshow(np.squeeze(depth_1_image.data))
    # axis[1, 2].imshow(np.squeeze(colorize_labels(label_1_image.data)))
    # axis[2, 0].imshow(color_2_image.data)
    # axis[2, 1].imshow(np.squeeze(depth_2_image.data))
    # axis[2, 2].imshow(np.squeeze(colorize_labels(label_2_image.data)))
    # plt.show()    


if __name__ == '__main__':
    main()
