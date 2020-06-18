import argparse
import os
import sys

import numpy as np

from pydrake.examples.manipulation_station import ManipulationStation, CreateClutterClearingYcbObjectList
from pydrake.geometry import ConnectDrakeVisualizer
from pydrake.multibody.plant import MultibodyPlant
from pydrake.manipulation.planner import (
    DifferentialInverseKinematicsParameters)
from pydrake.geometry.render import (
    DepthCameraProperties,
    RenderLabel,
    MakeRenderEngineVtk,
    RenderEngineVtkParams,
)
from pydrake.math import RigidTransform, RollPitchYaw, RotationMatrix
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import (BasicVector, DiagramBuilder,
                                       LeafSystem)
from pydrake.systems.meshcat_visualizer import MeshcatVisualizer


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--target_realtime_rate", type=float, default=1.0,
        help="Desired rate relative to real time.  See documentation for "
             "Simulator::set_target_realtime_rate() for details.")
    parser.add_argument(
        "--duration", type=float, default=np.inf,
        help="Desired duration of the simulation in seconds.")
    parser.add_argument(
        "--velocity_limit_factor", type=float, default=1.0,
        help="This value, typically between 0 and 1, further limits the "
             "iiwa14 joint velocities. It multiplies each of the seven "
             "pre-defined joint velocity limits. "
             "Note: The pre-defined velocity limits are specified by "
             "iiwa14_velocity_limits, found in this python file.")
    MeshcatVisualizer.add_argparse_argument(parser)
    args = parser.parse_args()

    builder = DiagramBuilder()

    station = builder.AddSystem(ManipulationStation())

    station.SetupClutterClearingStation()
    ycb_objects = CreateClutterClearingYcbObjectList()
    for model_file, X_WObject in ycb_objects:
        station.AddManipulandFromFile(model_file, X_WObject)

    station.Finalize()
    ConnectDrakeVisualizer(builder, station.get_scene_graph(),
                            station.GetOutputPort("pose_bundle"))
    meshcat = builder.AddSystem(MeshcatVisualizer(
        station.get_scene_graph(), zmq_url=args.meshcat))
    builder.Connect(station.GetOutputPort("pose_bundle"),
                    meshcat.get_input_port(0))

    scene_graph = station.get_scene_graph()

    renderer_name = "renderer"
    scene_graph.AddRenderer(renderer_name, MakeRenderEngineVtk(RenderEngineVtkParams()))

    depth_prop = DepthCameraProperties(width=640, height=480, fov_y=np.pi/4,
                                       renderer_name=renderer_name,
                                       z_near=0.01, z_far=10.)

    world_id = station.GetBodyFrameIdOrThrow(station.world_body().index())
    X_WB = xyz_rpy_deg([4, 0, 0], [-90, 0, 90])
    sensor = RgbdSensor(
        world_id, X_PB=X_WB,
        color_properties=depth_prop, depth_properties=depth_prop)
    builder.AddSystem(sensor)
    builder.Connect(scene_graph.get_query_output_port(), sensor.query_object_input_port())

    robot = station.get_controller_plant()
    params = DifferentialInverseKinematicsParameters(robot.num_positions(),
                                                     robot.num_velocities())

    time_step = 0.005
    params.set_timestep(time_step)
    # True velocity limits for the IIWA14 (in rad, rounded down to the first
    # decimal)
    iiwa14_velocity_limits = np.array([1.4, 1.4, 1.7, 1.3, 2.2, 2.3, 2.3])
    # Stay within a small fraction of those limits for this teleop demo.
    factor = args.velocity_limit_factor
    params.set_joint_velocity_limits((-factor*iiwa14_velocity_limits,
                                      factor*iiwa14_velocity_limits))

    diagram = builder.Build()
    simulator = Simulator(diagram)

    # This is important to avoid duplicate publishes to the hardware interface:
    simulator.set_publish_every_time_step(False)

    station_context = diagram.GetMutableSubsystemContext(
        station, simulator.get_mutable_context())

    print(station_context)

    station.GetInputPort("iiwa_feedforward_torque").FixValue(
        station_context, np.zeros(7))

    simulator.set_target_realtime_rate(args.target_realtime_rate)
    simulator.AdvanceTo(args.duration)


if __name__ == '__main__':
    main()
