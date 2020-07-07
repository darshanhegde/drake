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
from pydrake.systems.meshcat_visualizer import MeshcatPointCloudVisualizer, MeshcatVisualizer, AddTriad
from pydrake.perception import DepthImageToPointCloud, BaseField
from pydrake.systems.framework import DiagramBuilder, AbstractValue

from drake.examples.manipulation_station.differential_ik import DifferentialIK
from pydrake.manipulation.planner import DifferentialInverseKinematicsParameters

from drake.examples.manipulation_station.clutter_clearing_perception import ClutterClearingPerception
from drake.examples.manipulation_station.clutter_clearing_planner import GeneratePickAndPlaceTrajectoriesAndGripperSetPoints, IiwaPlanRunner
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


XW_home = RigidTransform(p=np.array([0.493, 0.0, 0.312]), 
                         rpy=RollPitchYaw([np.pi, -0.2084, -np.pi]))
XW_target = RigidTransform(p=np.array([0.4238, 0.0, 0.594]), 
                           rpy=RollPitchYaw([np.pi, 0.39159265, np.pi]))

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

    # Add cluttter clearing planning and execution system. 
    q_traj_list, gripper_setpoint_list = GeneratePickAndPlaceTrajectoriesAndGripperSetPoints(station, 
                                                                                             XW_home=XW_home, 
                                                                                             XW_target=XW_target, 
                                                                                             XW_O=XW_Object)
    iiwa_plans = [JointSpacePlan(q_traj) for q_traj in q_traj_list]
    plan_runner = builder.AddSystem(IiwaPlanRunner(iiwa_plans, gripper_setpoint_list))

    # Connect plan_runner outputs to the the simulation stattion. 
    builder.Connect(plan_runner.GetOutputPort("gripper_setpoint"),
                    station.GetInputPort("wsg_position"))
    builder.Connect(plan_runner.GetOutputPort("force_limit"),
                    station.GetInputPort("wsg_force_limit"))
    builder.Connect(plan_runner.GetOutputPort("iiwa_position_command"),
                    station.GetInputPort("iiwa_position"))
    builder.Connect(plan_runner.GetOutputPort("iiwa_torque_command"),
                    station.GetInputPort("iiwa_feedforward_torque"))

    builder.Connect(station.GetOutputPort("iiwa_position_measured"),
                    plan_runner.GetInputPort("iiwa_position"))
    builder.Connect(station.GetOutputPort("iiwa_position_commanded"),
                    plan_runner.GetInputPort("iiwa_position_cmd"))
    builder.Connect(station.GetOutputPort("iiwa_velocity_estimated"),
                    plan_runner.GetInputPort("iiwa_velocity"))
    builder.Connect(station.GetOutputPort("iiwa_torque_external"),
                    plan_runner.GetInputPort("iiwa_torque_external"))

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
    q0 = np.array([0, 0.6, 0, -1.75, 0, 1.0, 0])

    # robot = station.get_controller_plant()
    # params = DifferentialInverseKinematicsParameters(robot.num_positions(),
    #                                                  robot.num_velocities())

    # time_step = 0.005
    # params.set_timestep(time_step)
    # # True velocity limits for the IIWA14 (in rad, rounded down to the first
    # # decimal)
    # iiwa14_velocity_limits = np.array([1.4, 1.4, 1.7, 1.3, 2.2, 2.3, 2.3])
    # # Stay within a small fraction of those limits for this teleop demo.
    # factor = 1.0
    # params.set_joint_velocity_limits((-factor*iiwa14_velocity_limits,
    #                                   factor*iiwa14_velocity_limits))

    # differential_ik = builder.AddSystem(DifferentialIK(
    #     robot, robot.GetFrameByName("iiwa_link_7"), params, time_step))

    # ee_pose = differential_ik.ForwardKinematics(q0)
    # print(ee_pose.translation())
    # print(RollPitchYaw(ee_pose.rotation()).vector())

    # Set initial state of the robot.
    station_context.FixInputPort(
        station.GetInputPort("iiwa_position").get_index(), q0)
    station_context.FixInputPort(
        station.GetInputPort("iiwa_feedforward_torque").get_index(),
        np.zeros(7))
    station_context.FixInputPort(
        station.GetInputPort("wsg_position").get_index(), [0.05])
    station_context.FixInputPort(
        station.GetInputPort("wsg_force_limit").get_index(), [50])

    simulator.set_publish_every_time_step(False)
    simulator.set_target_realtime_rate(1.0)  # go as fast as possible

    # calculate starting time for all plans.
    t_plan = GetPlanStartingTimes(iiwa_plans)
    extra_time = 1.0
    sim_duration = t_plan[-1] + extra_time

    simulator.Initialize()
    # simulator.AdvanceTo(sim_duration)

    # for camera_name in camera_name_list:
    #     AddTriad(meshcat.vis, camera_name, prefix="cameras", radius=0.007, length=0.15)
    #     meshcat.vis["cameras"][camera_name].set_transform(camera_poses[camera_name].matrix())

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
