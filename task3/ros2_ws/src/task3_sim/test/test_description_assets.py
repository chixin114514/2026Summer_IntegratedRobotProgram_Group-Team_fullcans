import unittest
from pathlib import Path
from xml.etree import ElementTree


TASK3_ROOT = Path(__file__).resolve().parents[4]
ROS2_SRC = TASK3_ROOT / "ros2_ws" / "src"
TASK2_ROOT = TASK3_ROOT.parent / "tak2"
TASK2_URDF = (
    TASK2_ROOT
    / "ros2_ws"
    / "src"
    / "task2_sim"
    / "urdf"
    / "mecharm_270_gazebo.urdf"
)
TASK3_URDF = ROS2_SRC / "task3_sim" / "urdf" / "mecharm_270_gazebo.urdf"
TASK2_MESH_DIR = (
    TASK2_ROOT
    / "ros2_ws"
    / "src"
    / "mycobot_description"
    / "urdf"
    / "mecharm_270_m5"
)
TASK3_MESH_DIR = (
    ROS2_SRC
    / "mycobot_description"
    / "urdf"
    / "mecharm_270_m5"
)
TASK2_LICENSE = TASK2_ROOT / "ros2_ws" / "src" / "mycobot_description" / "LICENSE"
TASK3_DESCRIPTION_ROOT = ROS2_SRC / "mycobot_description"
TASK3_LICENSE = TASK3_DESCRIPTION_ROOT / "LICENSE"
TASK3_DESCRIPTION_PACKAGE = TASK3_DESCRIPTION_ROOT / "package.xml"
TASK3_DESCRIPTION_SETUP = TASK3_DESCRIPTION_ROOT / "setup.py"
MESH_NAMES = ("base.dae", "link1.dae", "link2.dae", "link3.dae", "link4.dae", "link5.dae", "link6.dae")
TEXT_EXTENSIONS = {".cfg", ".py", ".sdf", ".xml", ".yaml", ".yml"}


class DescriptionAssetTests(unittest.TestCase):
    def test_gazebo_urdf_is_a_byte_for_byte_copy_of_vendor_asset(self):
        self.assertTrue(TASK2_URDF.is_file(), f"missing source URDF: {TASK2_URDF}")
        self.assertTrue(TASK3_URDF.is_file(), f"missing Task3 URDF: {TASK3_URDF}")
        self.assertEqual(TASK3_URDF.read_bytes(), TASK2_URDF.read_bytes())

    def test_required_meshes_are_byte_for_byte_copies(self):
        for name in MESH_NAMES:
            source = TASK2_MESH_DIR / name
            target = TASK3_MESH_DIR / name
            self.assertTrue(source.is_file(), f"missing source mesh: {source}")
            self.assertTrue(target.is_file(), f"missing Task3 mesh: {target}")
            self.assertEqual(target.read_bytes(), source.read_bytes(), name)

    def test_description_license_is_a_byte_for_byte_copy_and_declared(self):
        self.assertTrue(TASK2_LICENSE.is_file(), f"missing source LICENSE: {TASK2_LICENSE}")
        self.assertTrue(TASK3_LICENSE.is_file(), f"missing Task3 LICENSE: {TASK3_LICENSE}")
        self.assertEqual(TASK3_LICENSE.read_bytes(), TASK2_LICENSE.read_bytes())

        package_root = ElementTree.parse(TASK3_DESCRIPTION_PACKAGE).getroot()
        self.assertEqual(package_root.findtext("license"), "BSD-2-Clause")
        self.assertIn('license="BSD-2-Clause"', TASK3_DESCRIPTION_SETUP.read_text(encoding="utf-8"))

    def test_urdf_package_mesh_uris_resolve_inside_task3_description_package(self):
        self.assertTrue(TASK3_URDF.is_file(), f"missing Task3 URDF: {TASK3_URDF}")
        root = ElementTree.parse(TASK3_URDF).getroot()
        uris = [
            mesh.attrib["filename"]
            for mesh in root.findall(".//mesh")
            if "filename" in mesh.attrib
        ]
        self.assertTrue(uris, "URDF has no mesh package URIs")
        for uri in uris:
            self.assertTrue(uri.startswith("package://mycobot_description/"), uri)
            relative = uri.removeprefix("package://mycobot_description/")
            resolved = ROS2_SRC / "mycobot_description" / Path(*relative.split("/"))
            self.assertTrue(resolved.is_file(), f"unresolved package URI {uri}: {resolved}")

    def test_task3_owned_files_do_not_publish_task2_topics(self):
        owned_roots = (
            ROS2_SRC / "task3_sim" / "launch",
            ROS2_SRC / "task3_sim" / "worlds",
            ROS2_SRC / "task3_sim" / "config",
            ROS2_SRC / "task3_sim" / "task3_sim",
            ROS2_SRC / "task3_sim" / "package.xml",
            ROS2_SRC / "task3_sim" / "setup.py",
            ROS2_SRC / "mycobot_description" / "package.xml",
            ROS2_SRC / "mycobot_description" / "setup.py",
        )
        files = []
        for root in owned_roots:
            if (
                root.is_file()
                and root.suffix.lower() in TEXT_EXTENSIONS
                and "__pycache__" not in root.parts
            ):
                files.append(root)
            elif root.is_dir():
                files.extend(
                    path
                    for path in root.rglob("*")
                    if path.is_file()
                    and path.suffix.lower() in TEXT_EXTENSIONS
                    and "__pycache__" not in path.parts
                )
        for path in files:
            self.assertNotIn("/task2/", path.read_text(encoding="utf-8"), path)


if __name__ == "__main__":
    unittest.main()
