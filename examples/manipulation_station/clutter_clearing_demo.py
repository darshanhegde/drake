"""
Provides some insight into the ManipulationStation model by printing out the
contents of its (default) Context.
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


def main():
    builder = DiagramBuilder()

    station = builder.AddSystem(ManipulationStation())
    station.SetupClutterClearingStation(IiwaCollisionModel.kBoxCollision)

    ycb_objects = CreateClutterClearingYcbObjectList()
    for model_file, X_WObject in ycb_objects:
        station.AddManipulandFromFile(model_file, X_WObject)
    station.Finalize()

    camera_name_list = station.get_camera_names()
    camera_poses = station.GetStaticCameraPosesInWorld()

    pc_concat = builder.AddSystem(PointCloudConcatenation(camera_name_list))

    camera_info = CameraInfo(**{"width": 848, "height": 480, "fov_y": 0.712439311})
    di2pcs = {}
    for camera_name in camera_name_list:
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
    q0 = np.array([0, 0, 0, -1.75, 0, 1.0, 0])

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

    # simulator.set_publish_every_time_step(False)
    simulator.set_target_realtime_rate(0.0)  # go as fast as possible

    simulator.Initialize()

    for camera_name in camera_name_list:
        AddTriad(meshcat.vis, camera_name, prefix="cameras", radius=0.007, length=0.15)
        meshcat.vis["cameras"][camera_name].set_transform(camera_poses[camera_name].matrix())

    station_context = diagram.GetMutableSubsystemContext(station, simulator.get_mutable_context())
    
    color_0_image = station.GetOutputPort("camera_0_rgb_image").Eval(station_context)
    depth_0_image = station.GetOutputPort("camera_0_depth_image").Eval(station_context)
    label_0_image = station.GetOutputPort("camera_0_label_image").Eval(station_context)

    color_1_image = station.GetOutputPort("camera_1_rgb_image").Eval(station_context)
    depth_1_image = station.GetOutputPort("camera_1_depth_image").Eval(station_context)
    label_1_image = station.GetOutputPort("camera_1_label_image").Eval(station_context)

    color_2_image = station.GetOutputPort("camera_2_rgb_image").Eval(station_context)
    depth_2_image = station.GetOutputPort("camera_2_depth_image").Eval(station_context)
    label_2_image = station.GetOutputPort("camera_2_label_image").Eval(station_context)

    fig, axis = plt.subplots(3, 3)
    axis[0, 0].imshow(color_0_image.data)
    axis[0, 1].imshow(np.squeeze(depth_0_image.data))
    axis[0, 2].imshow(np.squeeze(colorize_labels(label_0_image.data)))
    axis[1, 0].imshow(color_1_image.data)
    axis[1, 1].imshow(np.squeeze(depth_1_image.data))
    axis[1, 2].imshow(np.squeeze(colorize_labels(label_1_image.data)))
    axis[2, 0].imshow(color_2_image.data)
    axis[2, 1].imshow(np.squeeze(depth_2_image.data))
    axis[2, 2].imshow(np.squeeze(colorize_labels(label_2_image.data)))
    plt.show()    


if __name__ == '__main__':
    main()
