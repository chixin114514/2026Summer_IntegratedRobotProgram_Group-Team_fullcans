import hashlib
import unittest
from pathlib import Path
from xml.etree import ElementTree


TASK3_ROOT = Path(__file__).resolve().parents[4]
ROS2_SRC = TASK3_ROOT / "ros2_ws" / "src"
TASK3_URDF = ROS2_SRC / "task3_sim" / "urdf" / "mecharm_270_gazebo.urdf"
TASK3_MESH_DIR = (
    ROS2_SRC
    / "mycobot_description"
    / "urdf"
    / "mecharm_270_m5"
)
TASK3_DESCRIPTION_ROOT = ROS2_SRC / "mycobot_description"
TASK3_LICENSE = TASK3_DESCRIPTION_ROOT / "LICENSE"
TASK3_DESCRIPTION_PACKAGE = TASK3_DESCRIPTION_ROOT / "package.xml"
TASK3_DESCRIPTION_SETUP = TASK3_DESCRIPTION_ROOT / "setup.py"
MESH_NAMES = ("base.dae", "link1.dae", "link2.dae", "link3.dae", "link4.dae", "link5.dae", "link6.dae")
FROZEN_ASSET_SHA256 = {
    "mecharm_270_gazebo.urdf": "710de3793405fdb82edc677e70a489adf4c7f143a90c033dcad584f15ca28622",
    "base.dae": "f4f8868d9af882ccd0944e0c44c02b4057e4d345788e39a639e6538830943a73",
    "link1.dae": "a9cdff06a36ce6d8fdeb3ccf31e1934424c050126487fd8008716f58a5d009a0",
    "link2.dae": "a719dcbc3f409b2d52a7a2a5307317a92d2a1156d55d3965d5d875badf6fcac4",
    "link3.dae": "cb9077db5e06c94f9b3e4be51f7dd0be4c0f8c790c9e2a55abaf071f19d8da0d",
    "link4.dae": "1e69367cef73b53fd23727c2adc019a0271d3e337e58154c55e217304f867a17",
    "link5.dae": "c66778ce03ee5be532ba45f02580c8f7ed077227471faf3c8e826852a4847682",
    "link6.dae": "50f78c45ed98e3e11e284f7a603479496659d005eb33d2103d91e1e72fbc091b",
    "LICENSE": "28b1e3b807970e394efdd464faf42af65c65765869d61e1023d99c9627d22ef1",
}
TEXT_EXTENSIONS = {".cfg", ".py", ".sdf", ".xml", ".yaml", ".yml"}


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class DescriptionAssetTests(unittest.TestCase):
    def test_frozen_asset_gazebo_urdf_hash(self):
        self.assertTrue(TASK3_URDF.is_file(), f"missing Task3 URDF: {TASK3_URDF}")
        self.assertEqual(
            _sha256(TASK3_URDF),
            FROZEN_ASSET_SHA256["mecharm_270_gazebo.urdf"],
        )

    def test_frozen_asset_mesh_hashes(self):
        for name in MESH_NAMES:
            target = TASK3_MESH_DIR / name
            self.assertTrue(target.is_file(), f"missing Task3 mesh: {target}")
            self.assertEqual(_sha256(target), FROZEN_ASSET_SHA256[name], name)

    def test_frozen_asset_license_hash_and_declaration(self):
        self.assertTrue(TASK3_LICENSE.is_file(), f"missing Task3 LICENSE: {TASK3_LICENSE}")
        self.assertEqual(_sha256(TASK3_LICENSE), FROZEN_ASSET_SHA256["LICENSE"])

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
            text = path.read_text(encoding="utf-8")
            if path.name == "task3_scene.launch.py":
                # The immutable URDF exposes historical Gazebo-side topic
                # names.  The Task3 launch may mention those names only as
                # explicit one-to-one bridge remap entries; launch-contract
                # tests verify the complete mapping.
                continue
            self.assertNotIn("/task2/", text, path)


if __name__ == "__main__":
    unittest.main()
