import math
from pathlib import Path
import importlib.util
import xml.etree.ElementTree as ET

import numpy as np


DESCRIPTION_ROOT = Path(__file__).resolve().parents[1]


def _rotation_x(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.asarray([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def _rotation_y(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.asarray([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def _rotation_z(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.asarray([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def _origin_transform(origin):
    xyz = np.asarray([float(value) for value in origin.attrib["xyz"].split()])
    roll, pitch, yaw = [float(value) for value in origin.attrib["rpy"].split()]
    result = np.eye(4)
    result[:3, :3] = _rotation_z(yaw) @ _rotation_y(pitch) @ _rotation_x(roll)
    result[:3, 3] = xyz
    return result


def _urdf_fk(positions):
    root = ET.parse(DESCRIPTION_ROOT / "urdf" / "piper_description.xacro").getroot()
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    result = np.eye(4)
    for index, position in enumerate(positions, start=1):
        result = result @ _origin_transform(joints[f"joint{index}"].find("origin"))
        rotation = np.eye(4)
        rotation[:3, :3] = _rotation_z(position)
        result = result @ rotation
    return result


def _sdk_mode_0_fk(positions):
    a = np.asarray([0, 0, 285.03, -21.98, 0, 0]) / 1000.0
    alpha = np.asarray([0, -math.pi / 2, 0, math.pi / 2, -math.pi / 2, math.pi / 2])
    theta_offset = np.asarray([0, -math.radians(174.22), -math.radians(100.78), 0, 0, 0])
    d = np.asarray([123, 0, 0, 250.75, 0, 91]) / 1000.0
    result = np.eye(4)
    for index, position in enumerate(positions):
        ca, sa = math.cos(alpha[index]), math.sin(alpha[index])
        angle = position + theta_offset[index]
        ct, st = math.cos(angle), math.sin(angle)
        result = result @ np.asarray([
            [ct, -st, 0, a[index]],
            [st * ca, ct * ca, -sa, -sa * d[index]],
            [st * sa, ct * sa, ca, ca * d[index]],
            [0, 0, 0, 1],
        ])
    return result


def test_urdf_chain_matches_controller_mode_0_fk():
    poses = [
        np.zeros(6),
        np.asarray([0.4, 0.8, -0.7, 0.3, -0.4, 0.5]),
        np.asarray([-0.8, 1.5, -1.2, -0.5, 0.6, -1.0]),
    ]
    for pose in poses:
        assert np.allclose(_urdf_fk(pose), _sdk_mode_0_fk(pose), atol=1e-9)


def test_live_launch_is_feedback_only():
    launch_text = (DESCRIPTION_ROOT / "launch" / "display_live_robot.launch.py").read_text()
    assert 'remappings=[("joint_states", "/joint_states_single")]' in launch_text
    assert "joint_state_publisher_gui" not in launch_text
    assert "joint_ctrl_single" not in launch_text


def test_live_launch_pins_driver_domain_and_udp_transport():
    launch_text = (
        DESCRIPTION_ROOT / "launch" / "display_live_robot.launch.py"
    ).read_text()
    assert 'default_value=os.environ.get("ROS_DOMAIN_ID", "42")' in launch_text
    assert '"FASTRTPS_DEFAULT_PROFILES_FILE"' in launch_text
    assert 'SetEnvironmentVariable("RMW_FASTRTPS_USE_QOS_FROM_XML", "0")' in launch_text
    assert 'SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "0")' in launch_text
    assert (
        DESCRIPTION_ROOT / "config" / "fastdds_gui_udp_only.xml"
    ).is_file()


def test_camera_holder_mesh_is_fixed_visual_only_and_scaled_from_mm():
    root = ET.parse(DESCRIPTION_ROOT / "urdf" / "piper_description.xacro").getroot()
    links = {link.attrib["name"]: link for link in root.findall("link")}
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}

    holder = links["camera_holder"]
    visual_origin = holder.find("visual/origin")
    mesh = holder.find("visual/geometry/mesh")
    assert mesh.attrib["filename"] == (
        "package://piper_description/meshes/camera_holder.STL"
    )
    assert mesh.attrib["scale"] == "0.001 0.001 0.001"
    assert holder.find("collision") is None
    assert (DESCRIPTION_ROOT / "meshes" / "camera_holder.STL").is_file()

    mount = joints["gripper_base_to_camera_holder"]
    assert mount.attrib["type"] == "fixed"
    assert mount.find("parent").attrib["link"] == "gripper_base"
    assert mount.find("child").attrib["link"] == "camera_holder"
    assert np.allclose(
        [float(value) for value in mount.find("origin").attrib["xyz"].split()],
        [-0.036, 0.0, 0.044],
        atol=1e-12,
    )
    assert np.allclose(
        [float(value) for value in mount.find("origin").attrib["rpy"].split()],
        [0.0, 0.0, 0.0],
        atol=1e-12,
    )

    holder_from_mesh = _origin_transform(visual_origin)
    assert np.allclose(
        holder_from_mesh[:3, 3],
        [0.1765, -0.0565314679146, 0.0612789535522],
        atol=1e-12,
    )
    assert np.allclose(
        [float(value) for value in visual_origin.attrib["rpy"].split()],
        [-math.pi / 2.0, 0.0, math.pi / 2.0],
        atol=1e-12,
    )

    # Preserve the user's locked holder placement: its nominal close-hole datum
    # carries the intentional -4 mm visual Z trim.
    arm_anchor_midpoint_on_mating_face_m = np.asarray(
        [0.0565314679146, 0.0652789535522, 0.1765, 1.0]
    )
    assert np.allclose(
        holder_from_mesh @ arm_anchor_midpoint_on_mating_face_m,
        [0.0, 0.0, -0.004, 1.0],
        atol=1e-12,
    )

    # The close lug holes remain the arm anchors and keep their exact 12 mm
    # spacing; the locked visual trim places their rendered centres at z=40 mm.
    close_arm_anchors_m = np.asarray(
        [
            [0.0505314679146, 0.0652789535522, 0.1765, 1.0],
            [0.0625314679146, 0.0652789535522, 0.1765, 1.0],
        ]
    )
    anchors_in_holder = (holder_from_mesh @ close_arm_anchors_m.T).T
    assert np.allclose(
        anchors_in_holder[:, :3],
        [[0.0, -0.006, -0.004], [0.0, 0.006, -0.004]],
        atol=1e-12,
    )

    gripper_from_holder = _origin_transform(mount.find("origin"))
    anchors_in_gripper = (gripper_from_holder @ anchors_in_holder.T).T
    assert np.allclose(
        anchors_in_gripper[:, :3],
        [[-0.036, -0.006, 0.040], [-0.036, 0.006, 0.040]],
        atol=1e-12,
    )

    # The L515 uses the 40 mm pair through the lower raw circular cradle.  The
    # installed rotations place that cradle above the gripper without moving
    # the holder itself.
    l515_fastener_axes_m = np.asarray(
        [
            [0.0565314679146, 0.0911539535522, 0.0565, 1.0],
            [0.0565314679146, 0.0911539535522, 0.0965, 1.0],
        ]
    )
    camera_fasteners_in_holder = (holder_from_mesh @ l515_fastener_axes_m.T).T
    assert np.allclose(
        camera_fasteners_in_holder[:, :3],
        [[0.12, 0.0, -0.029875], [0.08, 0.0, -0.029875]],
        atol=1e-12,
    )


def test_reversed_physical_gripper_and_l515_visual_mount_are_explicit():
    root = ET.parse(DESCRIPTION_ROOT / "urdf" / "piper_description.xacro").getroot()
    links = {link.attrib["name"]: link for link in root.findall("link")}
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}

    gripper_mount = joints["joint6_to_gripper_base"]
    assert gripper_mount.attrib["type"] == "fixed"
    assert gripper_mount.find("parent").attrib["link"] == "link6"
    assert gripper_mount.find("child").attrib["link"] == "gripper_base"
    assert np.allclose(
        [float(value) for value in gripper_mount.find("origin").attrib["rpy"].split()],
        [0.0, 0.0, math.pi],
        atol=1e-12,
    )

    camera = links["l515_visual"]
    camera_visual = camera.find("visual")
    camera_mesh = camera_visual.find("geometry/mesh")
    assert camera_mesh.attrib["filename"] == (
        "package://piper_description/meshes/Intel_RealSense_L515_CAD_external.STL"
    )
    assert camera_mesh.attrib["scale"] == "0.001 0.001 0.001"
    assert camera.find("collision") is None
    assert camera.find("inertial") is None
    assert (DESCRIPTION_ROOT / "meshes" / "Intel_RealSense_L515_CAD_external.STL").is_file()
    assert camera_visual.find("material").attrib["name"] == "l515_silver"
    assert np.allclose(
        [
            float(value)
            for value in camera_visual.find("material/color").attrib["rgba"].split()
        ],
        [0.55, 0.58, 0.62, 1.0],
        atol=1e-12,
    )
    assert np.allclose(
        [
            float(value)
            for value in camera_visual.find("origin").attrib["xyz"].split()
        ],
        [-0.030503129, -0.030503409, -0.029628554],
        atol=1e-12,
    )

    camera_mount = joints["camera_holder_to_l515_visual"]
    assert camera_mount.attrib["type"] == "fixed"
    assert camera_mount.find("parent").attrib["link"] == "camera_holder"
    assert camera_mount.find("child").attrib["link"] == "l515_visual"
    holder_from_camera = _origin_transform(camera_mount.find("origin"))
    assert np.allclose(
        holder_from_camera[:3, 3],
        [0.1, 0.0, 0.0],
        atol=1e-12,
    )
    assert np.allclose(
        holder_from_camera[:3, :3],
        np.asarray([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]),
        atol=1e-12,
    )

    # The L515 rear screw axes are 40 mm apart and land on the holder's matching
    # 40 mm axes, including the axial seating depth of both CAD models.
    camera_from_mesh = _origin_transform(camera_visual.find("origin"))
    l515_fastener_axes_in_mesh_m = np.asarray(
        [
            [0.030503129, 0.010503409, 0.002253556, 1.0],
            [0.030503129, 0.050503409, 0.002253556, 1.0],
        ]
    )
    fasteners_in_holder = (
        holder_from_camera
        @ camera_from_mesh
        @ l515_fastener_axes_in_mesh_m.T
    ).T
    assert np.allclose(
        fasteners_in_holder[:, :3],
        [[0.08, 0.0, -0.027374998], [0.12, 0.0, -0.027374998]],
        atol=1e-9,
    )
    assert math.isclose(
        np.linalg.norm(fasteners_in_holder[1, :3] - fasteners_in_holder[0, :3]),
        0.04,
        abs_tol=1e-12,
    )


def test_legacy_control_and_simulation_surfaces_are_absent():
    assert not (DESCRIPTION_ROOT / "launch" / "display_xacro.launch.py").exists()
    assert not (
        DESCRIPTION_ROOT / "config" / "piper_gazebo_control.yaml"
    ).exists()
    assert not (
        DESCRIPTION_ROOT / "config" / "joint_names_agx_arm_description.yaml"
    ).exists()
    package_text = (DESCRIPTION_ROOT / "package.xml").read_text()
    model_text = (
        DESCRIPTION_ROOT / "urdf" / "piper_description.xacro"
    ).read_text()
    assert "joint_state_publisher_gui" not in package_text
    assert "xmlns:xacro" not in model_text
    assert "<transmission" not in model_text
    assert "<gazebo" not in model_text


def _preview_module():
    path = DESCRIPTION_ROOT / "scripts" / "piper_joint_preview_node.py"
    spec = importlib.util.spec_from_file_location("piper_joint_preview_node", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_preview_joint_angle_round_trip_and_limits():
    module = _preview_module()
    fixed = module.rpy_quaternion(0.4, -0.2, 1.1)
    for angle in (-2.0, -0.3, 0.0, 0.8, 2.7):
        marker = module.quaternion_multiply(fixed, module.z_rotation_quaternion(angle))
        assert math.isclose(module.relative_z_angle(fixed, marker), angle, abs_tol=1e-9)
    assert module.clamp(4.0, -1.0, 2.0) == 2.0


def test_joint_preview_launch_never_names_real_command_topic():
    launch_text = (DESCRIPTION_ROOT / "launch" / "joint_preview.launch.py").read_text()
    node_text = (DESCRIPTION_ROOT / "scripts" / "piper_joint_preview_node.py").read_text()
    assert "/piper_gui/preview_joint_states" in launch_text
    assert "/piper_gui/preview_set" in node_text
    assert "joint_ctrl_single" not in launch_text
    assert "joint_ctrl_single" not in node_text


def test_joint_preview_has_large_visible_rotation_grab_ring():
    node_text = (DESCRIPTION_ROOT / "scripts" / "piper_joint_preview_node.py").read_text()
    assert "marker.scale = 0.30" in node_text
    assert "Marker.LINE_STRIP" in node_text
    assert "ring_radius = 0.115" in node_text
