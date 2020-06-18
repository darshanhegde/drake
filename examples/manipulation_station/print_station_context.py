"""
Provides some insight into the ManipulationStation model by printing out the
contents of its (default) Context.
"""

import numpy as np
import matplotlib.pyplot as plt

from pydrake.examples.manipulation_station import ManipulationStation
from pydrake.systems.analysis import Simulator
from pydrake.systems.perception import PointCloudConcatenation
from pydrake.perception import DepthImageToPointCloud, BaseField
from pydrake.systems.framework import DiagramBuilder


def main():
    builder = DiagramBuilder()

    station = builder.AddSystem(ManipulationStation())
    station.SetupClutterClearingStation()
    station.Finalize()

    camera_name_list = station.get_camera_names()

    diagram = builder.Build()

    simulator = Simulator(diagram)

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
    simulator.AdvanceTo(5.00)

    station_context = diagram.GetMutableSubsystemContext(station, simulator.get_mutable_context())
    
    color_image = station.GetOutputPort("camera_0_rgb_image").Eval(station_context)
    depth_image = station.GetOutputPort("camera_0_depth_image").Eval(station_context)
    label_image = station.GetOutputPort("camera_0_label_image").Eval(station_context)

    print("color image mean: ", np.mean(color_image.data), color_image)
    print("depth image mean: ", np.mean(depth_image.data), depth_image)
    print("label image mean: ", np.mean(label_image.data), label_image)
    plt.subplot(131)
    plt.imshow(color_image.data)
    plt.title('Color image')
    plt.subplot(132)
    plt.imshow(np.squeeze(depth_image.data))
    plt.title('Depth image')
    plt.subplot(133)
    plt.imshow(np.squeeze(label_image.data))
    plt.title('Label image')
    plt.show()    


if __name__ == '__main__':
    main()
