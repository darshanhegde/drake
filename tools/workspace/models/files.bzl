# -*- python -*-

# Keep the macros sorted alphabetically by macro name.
# Keep the lists of files sorted alphabetically by filename.

def dishes_files():
    return [
        "dishes/bowls/evo_bowl_no_mtl.obj",
    ]

def franka_description_mesh_files():
    return [
        "franka_description/LICENSE",
        "franka_description/meshes/visual/finger.mtl",
        "franka_description/meshes/visual/finger.obj",
        "franka_description/meshes/visual/hand.mtl",
        "franka_description/meshes/visual/hand.obj",
        "franka_description/meshes/visual/link0.mtl",
        "franka_description/meshes/visual/link0.obj",
        "franka_description/meshes/visual/link1.mtl",
        "franka_description/meshes/visual/link1.obj",
        "franka_description/meshes/visual/link2.mtl",
        "franka_description/meshes/visual/link2.obj",
        "franka_description/meshes/visual/link3.mtl",
        "franka_description/meshes/visual/link3.obj",
        "franka_description/meshes/visual/link4.mtl",
        "franka_description/meshes/visual/link4.obj",
        "franka_description/meshes/visual/link5.mtl",
        "franka_description/meshes/visual/link5.obj",
        "franka_description/meshes/visual/link6.mtl",
        "franka_description/meshes/visual/link6.obj",
        "franka_description/meshes/visual/link7.mtl",
        "franka_description/meshes/visual/link7.obj",
    ]

def jaco_description_mesh_files():
    return [
        "jaco_description/LICENSE",
        "jaco_description/meshes/arm.obj",
        "jaco_description/meshes/arm_half_1.obj",
        "jaco_description/meshes/arm_half_2.obj",
        "jaco_description/meshes/arm_mico.obj",
        "jaco_description/meshes/base.obj",
        "jaco_description/meshes/finger_distal.obj",
        "jaco_description/meshes/finger_proximal.obj",
        "jaco_description/meshes/forearm.obj",
        "jaco_description/meshes/forearm_mico.obj",
        "jaco_description/meshes/hand_2finger.obj",
        "jaco_description/meshes/hand_3finger.obj",
        "jaco_description/meshes/ring_big.obj",
        "jaco_description/meshes/ring_small.obj",
        "jaco_description/meshes/shoulder.obj",
        "jaco_description/meshes/wrist.obj",
        "jaco_description/meshes/wrist_spherical_1.obj",
        "jaco_description/meshes/wrist_spherical_2.obj",
    ]

def skydio_2_mesh_files():
    return [
        "skydio_2/skydio_2_1000_poly.mtl",
        "skydio_2/skydio_2_1000_poly.obj",
        "skydio_2/skydio_2.png",
        "skydio_2/LICENSE",
    ]

def wsg_50_description_mesh_files():
    return [
        "wsg_50_description/meshes/finger_without_tip.obj",
        "wsg_50_description/meshes/finger_with_tip.obj",
        "wsg_50_description/meshes/wsg_body.obj",
    ]


def xarm7_description_mesh_files():
    return [
        "xarm7_description/meshes/xarm7/visuals/link1.mtl",
        "xarm7_description/meshes/xarm7/visuals/link1.obj",
        "xarm7_description/meshes/xarm7/visuals/link2.mtl",
        "xarm7_description/meshes/xarm7/visuals/link2.obj",
        "xarm7_description/meshes/xarm7/visuals/link3.mtl",
        "xarm7_description/meshes/xarm7/visuals/link3.obj",
        "xarm7_description/meshes/xarm7/visuals/link4.mtl",
        "xarm7_description/meshes/xarm7/visuals/link4.obj",
        "xarm7_description/meshes/xarm7/visuals/link5.mtl",
        "xarm7_description/meshes/xarm7/visuals/link5.obj",
        "xarm7_description/meshes/xarm7/visuals/link6.mtl",
        "xarm7_description/meshes/xarm7/visuals/link6.obj",
        "xarm7_description/meshes/xarm7/visuals/link7.mtl",
        "xarm7_description/meshes/xarm7/visuals/link7.obj",
        "xarm7_description/meshes/xarm7/visuals/link_base.mtl",
        "xarm7_description/meshes/xarm7/visuals/link_base.obj",
        "xarm7_description/meshes/xarm_gripper/visuals/base_link.mtl",
        "xarm7_description/meshes/xarm_gripper/visuals/base_link.obj",
        "xarm7_description/meshes/xarm_gripper/visuals/left_finger.mtl", 
        "xarm7_description/meshes/xarm_gripper/visuals/left_finger.obj",
        "xarm7_description/meshes/xarm_gripper/visuals/left_inner_knuckle.mtl",
        "xarm7_description/meshes/xarm_gripper/visuals/left_inner_knuckle.obj", 
        "xarm7_description/meshes/xarm_gripper/visuals/left_outer_knuckle.mtl", 
        "xarm7_description/meshes/xarm_gripper/visuals/left_outer_knuckle.obj", 
        "xarm7_description/meshes/xarm_gripper/visuals/right_finger.mtl", 
        "xarm7_description/meshes/xarm_gripper/visuals/right_finger.obj", 
        "xarm7_description/meshes/xarm_gripper/visuals/right_inner_knuckle.mtl", 
        "xarm7_description/meshes/xarm_gripper/visuals/right_inner_knuckle.obj", 
        "xarm7_description/meshes/xarm_gripper/visuals/right_outer_knuckle.mtl", 
        "xarm7_description/meshes/xarm_gripper/visuals/right_outer_knuckle.obj"
    ]

def ycb_mesh_files():
    """Manual enumeration of mesh files, to avoid needing to write extra Bazel
    logic.

    Recipe to reproduce:
        $ cd models
        $ find ycb/meshes -type f | \
            python3 -c 'import sys; print(repr(sys.stdin.read().split()))'
    """
    return [
        "ycb/meshes/003_cracker_box_textured.mtl",
        "ycb/meshes/003_cracker_box_textured.obj",
        "ycb/meshes/003_cracker_box_textured.png",
        "ycb/meshes/004_sugar_box_textured.mtl",
        "ycb/meshes/004_sugar_box_textured.obj",
        "ycb/meshes/004_sugar_box_textured.png",
        "ycb/meshes/005_tomato_soup_can_textured.mtl",
        "ycb/meshes/005_tomato_soup_can_textured.obj",
        "ycb/meshes/005_tomato_soup_can_textured.png",
        "ycb/meshes/006_mustard_bottle_textured.mtl",
        "ycb/meshes/006_mustard_bottle_textured.obj",
        "ycb/meshes/006_mustard_bottle_textured.png",
        "ycb/meshes/009_gelatin_box_textured.mtl",
        "ycb/meshes/009_gelatin_box_textured.obj",
        "ycb/meshes/009_gelatin_box_textured.png",
        "ycb/meshes/010_potted_meat_can_textured.mtl",
        "ycb/meshes/010_potted_meat_can_textured.obj",
        "ycb/meshes/010_potted_meat_can_textured.png",
        "ycb/meshes/LICENSE.txt",
    ]
