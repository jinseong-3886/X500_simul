#!/usr/bin/env python3
"""Build the Flytrap PX4/Gazebo-Harmonic sim assets under ~/flytrap_sim (does not touch PX4-Autopilot).

Scenario = README §7.4 (3 m mocap cube): target board at x = 2.5 m, centre height 1.0 m, drone at origin.
Board = A2 landscape (0.594 x 0.396 m), same texture files that were printed / shown on the tablet.
Drone = PX4 x500 + a D435i-like RGB-D camera (640x480, HFOV 55.6 deg, colour and depth co-located).

Usage: python3 build_flytrap_world.py [--tex-dir DIR] [--board-x 2.5] [--board-z 1.0]
"""
import argparse, os, shutil, textwrap

ap = argparse.ArgumentParser()
ap.add_argument("--tex-dir", default="/mnt/c/Users/default.DESKTOP-OA41OO2/Downloads/x500_print")
ap.add_argument("--board-x", type=float, default=2.5)
ap.add_argument("--board-z", type=float, default=1.0)
ap.add_argument("--board-w", type=float, default=0.594)
ap.add_argument("--board-h", type=float, default=0.396)
ap.add_argument("--pic-roll", type=float, default=1.5708, help="roll of the picture plane (fixes plane UV rotation)")
a = ap.parse_args()

ROOT = os.path.expanduser("~/flytrap_sim")
M, W = os.path.join(ROOT, "models"), os.path.join(ROOT, "worlds")
os.makedirs(M, exist_ok=True); os.makedirs(W, exist_ok=True)

HFOV = 0.9704  # rad = 55.6 deg, measured on the real D435i colour stream at 640x480


def model_config(name, desc):
    return textwrap.dedent(f"""\
        <?xml version="1.0"?>
        <model>
          <name>{name}</name>
          <version>1.0</version>
          <sdf version="1.9">model.sdf</sdf>
          <description>{desc}</description>
        </model>
        """)


# ---------------- drone: x500 + D435i-like RGB-D ----------------
d = os.path.join(M, "x500_d435i"); os.makedirs(d, exist_ok=True)
open(os.path.join(d, "model.config"), "w").write(model_config("x500_d435i", "PX4 x500 with a D435i-like RGB-D camera"))
open(os.path.join(d, "model.sdf"), "w").write(textwrap.dedent(f"""\
    <?xml version="1.0" encoding="UTF-8"?>
    <sdf version="1.9">
      <model name="x500_d435i">
        <include merge="true">
          <uri>x500</uri>
        </include>
        <link name="d435i_link">
          <pose>0.12 0 0.242 0 0 0</pose>
          <inertial>
            <mass>0.072</mass>
            <inertia><ixx>0.00005</ixx><iyy>0.000005</iyy><izz>0.00005</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
          </inertial>
          <visual name="d435i_visual">
            <geometry><box><size>0.025 0.09 0.025</size></box></geometry>
            <material><diffuse>0.3 0.3 0.3 1</diffuse></material>
          </visual>
          <sensor name="d435i_color" type="camera">
            <camera>
              <horizontal_fov>{HFOV}</horizontal_fov>
              <image><width>640</width><height>480</height><format>R8G8B8</format></image>
              <clip><near>0.1</near><far>30</far></clip>
            </camera>
            <always_on>1</always_on>
            <update_rate>15</update_rate>
            <topic>flytrap/color</topic>
          </sensor>
          <sensor name="d435i_depth" type="depth_camera">
            <camera>
              <horizontal_fov>{HFOV}</horizontal_fov>
              <image><width>640</width><height>480</height><format>R_FLOAT32</format></image>
              <clip><near>0.2</near><far>10</far></clip>
            </camera>
            <always_on>1</always_on>
            <update_rate>15</update_rate>
            <topic>flytrap/depth</topic>
          </sensor>
        </link>
        <joint name="d435i_joint" type="fixed">
          <parent>base_link</parent>
          <child>d435i_link</child>
        </joint>
      </model>
    </sdf>
    """))

# ---------------- target boards ----------------
for kind in ("clean", "patched"):
    name = f"bus_board_{kind}"
    d = os.path.join(M, name); os.makedirs(os.path.join(d, "materials", "textures"), exist_ok=True)
    tex = f"bus_{kind}.png"
    shutil.copy(os.path.join(a.tex_dir, tex), os.path.join(d, "materials", "textures", tex))
    open(os.path.join(d, "model.config"), "w").write(model_config(name, f"A2 bus board ({kind})"))
    # Picture on a thin plane facing -X (toward the drone), on a white foam-board backing box.
    open(os.path.join(d, "model.sdf"), "w").write(textwrap.dedent(f"""\
        <?xml version="1.0" encoding="UTF-8"?>
        <sdf version="1.9">
          <model name="{name}">
            <static>true</static>
            <link name="board">
              <visual name="backing">
                <pose>0.006 0 0 0 0 0</pose>
                <geometry><box><size>0.01 {a.board_w + 0.04:.3f} {a.board_h + 0.04:.3f}</size></box></geometry>
                <material><diffuse>0.95 0.95 0.95 1</diffuse><ambient>0.95 0.95 0.95 1</ambient></material>
              </visual>
              <visual name="picture">
                <pose>0 0 0 {a.pic_roll} 0 0</pose>
                <geometry><plane><normal>-1 0 0</normal><size>{a.board_w} {a.board_h}</size></plane></geometry>
                <material>
                  <diffuse>1 1 1 1</diffuse><ambient>1 1 1 1</ambient><specular>0 0 0 1</specular>
                  <pbr><metal><albedo_map>model://{name}/materials/textures/{tex}</albedo_map><metalness>0</metalness><roughness>1</roughness></metal></pbr>
                </material>
              </visual>
              <collision name="collision">
                <pose>0.006 0 0 0 0 0</pose>
                <geometry><box><size>0.01 {a.board_w + 0.04:.3f} {a.board_h + 0.04:.3f}</size></box></geometry>
              </collision>
            </link>
          </model>
        </sdf>
        """))

# ---------------- worlds (one per board) ----------------
default_world = open(os.path.expanduser("~/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf"), encoding="utf-8-sig").read()
for kind in ("clean", "patched"):
    wname = f"flytrap_{kind}"
    w = default_world.replace('<world name="default">', f'<world name="{wname}">')
    extra = textwrap.dedent(f"""\
        <include>
          <uri>model://bus_board_{kind}</uri>
          <name>target_board</name>
          <pose>{a.board_x} 0 {a.board_z} 0 0 0</pose>
        </include>
        <include>
          <uri>model://x500_d435i</uri>
          <name>x500_d435i_0</name>
          <pose>0 0 0.2 0 0 0</pose>
        </include>
        """)
    w = w.replace("</world>", extra + "  </world>")
    open(os.path.join(W, f"{wname}.sdf"), "w").write(w)

print("built:", ROOT)
for r, _, fs in os.walk(ROOT):
    if "venv" in r:
        continue
    for f in fs:
        print("  ", os.path.relpath(os.path.join(r, f), ROOT))
