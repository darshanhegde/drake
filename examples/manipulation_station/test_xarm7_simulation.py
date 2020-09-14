
from pydrake.all import (
    AddMultibodyPlantSceneGraph, MultibodyPlant, FindResourceOrThrow, Parser, Simulator, 
    DiagramBuilder, ConnectMeshcatVisualizer, InverseDynamicsController

)
import matplotlib.pyplot as plt
import numpy as np
import pydot
from PIL import Image
import io

def main():
    builder = DiagramBuilder()

    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-4)
    xarm_model = Parser(plant).AddModelFromFile(
                    FindResourceOrThrow("drake/manipulation/models/xarm7_description/urdf/xarm7.urdf"))
    # xarm_gripper = Parser(plant).AddModelFromFile(
    #                 FindResourceOrThrow("drake/manipulation/models/xarm7_description/urdf/xarm_gripper.urdf"))
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("link_base"))
    # plant.WeldFrames(plant.GetFrameByName("link7"), plant.GetFrameByName("xarm_gripper_base_link"))

    plant.Finalize()

    plant_plot_str = pydot.graph_from_dot_data(plant.GetTopologyGraphvizString())[0].create_jpg()
    plant_plot_image = Image.open(io.BytesIO(plant_plot_str))
    # plant_plot_image.show()
    
    context = plant.CreateDefaultContext()

    # Adds the MeshcatVisualizer and wires it to the SceneGraph.
    meshcat = ConnectMeshcatVisualizer(builder, scene_graph, delete_prefix_on_load=False)

    Kp = np.full(7, 100)
    Ki = 2 * np.sqrt(Kp)
    Kd = np.full(7, 1)
    xarm_controller = builder.AddSystem(InverseDynamicsController(plant, Kp, Ki, Kd, False))
    xarm_controller.set_name("xarm_controller")
    builder.Connect(plant.get_state_output_port(xarm_model),
                    xarm_controller.get_input_port_estimated_state())
    builder.Connect(xarm_controller.get_output_port_control(),
                    plant.get_actuation_input_port())

    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyMutableContextFromRoot(context)
    # plant.get_actuation_input_port().FixValue(plant_context, np.zeros(9))
    q0 = np.array([0., 0., 0., 0., 0., 0., 0.])
    x0 = np.hstack((q0, 0*q0))
    plant.SetPositions(plant_context, q0)
    xarm_controller.GetInputPort('desired_state').FixValue(xarm_controller.GetMyMutableContextFromRoot(context), x0)

    simulator = Simulator(diagram, context)
    simulator.set_target_realtime_rate(1.0)

    meshcat.start_recording()
    simulator.AdvanceTo(5.0)
    meshcat.stop_recording()
    meshcat.publish_recording()


if __name__ == '__main__':
    main()
