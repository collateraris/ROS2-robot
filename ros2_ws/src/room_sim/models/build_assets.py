#!/usr/bin/env python3
"""Rebuild the self-contained Gazebo assets (no downloads; stdlib only).

This writes only models/, urdf/, and worlds/ in this room_sim package.
The images are procedural textures, not edited or generated photographs.
"""
from __future__ import annotations

from pathlib import Path
import math
import random
import struct
import zlib
from xml.sax.saxutils import escape

ROOT = Path(__file__).resolve().parent.parent
MODELS = ROOT / "models"
PI = math.pi


def write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def png(path: Path, pixels: bytes, width: int = 256, height: int = 256) -> None:
    def chunk(kind: bytes, data: bytes) -> bytes:
        return (struct.pack(">I", len(data)) + kind + data
                + struct.pack(">I", zlib.crc32(kind + data) & 0xffffffff))
    raw = b"".join(b"\0" + pixels[y * width * 3:(y + 1) * width * 3]
                   for y in range(height))
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
                     + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))


def textures() -> None:
    rng = random.Random(7851)
    for style in ("wood", "plaster", "fabric", "floor"):
        out = bytearray()
        for y in range(256):
            for x in range(256):
                noise = rng.randrange(-5, 6)
                if style == "wood":
                    wave = int(8 * math.sin(x * .18 + 1.7 * math.sin(y * .025)))
                    seam = -35 if x % 64 < 2 else 0
                    color = (161 + wave + seam, 111 + wave + seam, 65 + wave + seam)
                elif style == "plaster":
                    color = (219, 218, 205)
                elif style == "fabric":
                    weave = 6 if (x + y) % 4 == 0 else -3
                    color = (56 + weave, 95 + weave, 113 + weave)
                else:
                    if x % 64 < 2 or y % 64 < 2:
                        color = (104, 110, 109)
                    else:
                        tile = 6 if ((x // 64) + (y // 64)) % 2 else -6
                        color = (161 + tile, 165 + tile, 158 + tile)
                out.extend(max(0, min(255, channel + noise)) for channel in color)
        png(MODELS / "room_environment/materials/textures" / f"{style}.png", bytes(out))


def config(name: str, description: str) -> str:
    return f'''<?xml version="1.0"?>
<model><name>{name}</name><version>1.0</version><sdf version="1.9">model.sdf</sdf>
  <author><name>Room Robot project</name></author>
  <description>{escape(description)}</description>
</model>
'''


def material(color: str = ".6 .6 .6 1", texture: str | None = None) -> str:
    pbr = ""
    if texture:
        pbr = f'''<pbr><metal><albedo_map>model://room_environment/materials/textures/{texture}.png</albedo_map>
          <metalness>0</metalness><roughness>0.85</roughness></metal></pbr>'''
    return f"<material><ambient>{color}</ambient><diffuse>{color}</diffuse>{pbr}</material>"


def box(name: str, size: str, pose: str, color: str = ".6 .6 .6 1",
        texture: str | None = None, collision: bool = True) -> str:
    geo = f"<geometry><box><size>{size}</size></box></geometry>"
    col = f'<collision name="{name}_collision"><pose>{pose}</pose>{geo}</collision>' if collision else ""
    return f'''{col}<visual name="{name}_visual"><pose>{pose}</pose>{geo}
      {material(color, texture)}</visual>'''


def cylinder(name: str, radius: float, length: float, pose: str, color: str,
             collision: bool = True) -> str:
    geo = f"<geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>"
    col = f'<collision name="{name}_collision"><pose>{pose}</pose>{geo}</collision>' if collision else ""
    return f'{col}<visual name="{name}_visual"><pose>{pose}</pose>{geo}{material(color)}</visual>'


def static_model(name: str, pose: str, parts: str) -> str:
    return f'''<model name="{name}"><static>true</static><pose>{pose}</pose>
      <link name="body">{parts}</link></model>'''


def robot() -> None:
    wheel_links = []
    wheel_joints = []
    urdf_wheels = []
    for end, x in (("front", .085), ("rear", -.085)):
        for side, y in (("left", .105), ("right", -.105)):
            name = f"{end}_{side}_wheel"
            # Cylinder geometry is rotated, not its link frame: the joint axis is +Y.
            geo = "<geometry><cylinder><radius>0.033</radius><length>0.025</length></cylinder></geometry>"
            wheel_links.append(f'''<link name="{name}">
      <pose relative_to="base_link">{x} {y} 0 0 0 0</pose>
      <inertial><mass>0.06</mass><inertia><ixx>0.00001946</ixx><iyy>0.00003267</iyy><izz>0.00001946</izz></inertia></inertial>
      <collision name="{name}_collision"><pose>0 0 0 {PI/2} 0 0</pose>{geo}
        <surface><friction><ode><mu>1.0</mu><mu2>0.1</mu2><fdir1>1 0 0</fdir1></ode></friction></surface>
      </collision>
      <visual name="{name}_visual"><pose>0 0 0 {PI/2} 0 0</pose>{geo}{material('.04 .045 .05 1')}</visual>
      <!-- Real wheel/ground contacts keep the contact stream observable on clean runs.
           Each sensor references only its own link's collision; no synthetic heartbeat. -->
      <sensor name="{name}_contact" type="contact">
        <always_on>true</always_on><update_rate>20</update_rate>
        <contact><collision>{name}_collision</collision><topic>/sim/contacts</topic></contact>
      </sensor>
    </link>''')
            wheel_joints.append(f'''<joint name="{name}_joint" type="revolute">
      <parent>base_link</parent><child>{name}</child><axis><xyz>0 1 0</xyz>
        <limit><lower>-1e16</lower><upper>1e16</upper><effort>2</effort><velocity>30</velocity></limit>
        <dynamics><damping>0.002</damping><friction>0</friction></dynamics>
      </axis></joint>''')
            urdf_wheels.append(f'''  <!-- Fixed visualization only; Gazebo wheel articulation lives in SDF. -->
  <link name="{name}"><visual><origin rpy="{PI/2} 0 0"/><geometry><cylinder radius="0.033" length="0.025"/></geometry><material name="rubber"/></visual></link>
  <joint name="{name}_visual_joint" type="fixed"><parent link="base_link"/><child link="{name}"/><origin xyz="{x} {y} 0"/></joint>''')
    sdf = f'''<?xml version="1.0"?>
<!-- Native Gazebo Harmonic SDF. Operational odometry comes ONLY from RF2O. -->
<sdf version="1.9"><model name="room_robot" canonical_link="base_link">
  <self_collide>false</self_collide>
  <!-- Model origin = base_link at axle plane. Spawn z=0.034: radius 0.033 + 1 mm. -->
  <link name="base_link">
    <inertial><pose>0 0 0.02 0 0 0</pose><mass>1.6</mass>
      <inertia><ixx>0.004186667</ixx><iyy>0.008013333</iyy><izz>0.011533333</izz></inertia></inertial>
    {box('chassis', '.24 .17 .05', '0 0 .02 0 0 0', '.18 .42 .22 1')}
    {box('bumper', '.27 .25 .025', '0 0 .015 0 0 0', '.11 .12 .13 1')}
    {box('battery', '.11 .065 .025', '-.035 0 .057 0 0 0', '.12 .12 .15 1', collision=False)}
    {box('compute', '.07 .055 .012', '.065 0 .058 0 0 0', '.05 .32 .14 1', collision=False)}
    <sensor name="bumper_contact" type="contact">
      <always_on>true</always_on><update_rate>100</update_rate>
      <contact><collision>bumper_collision</collision><topic>/sim/contacts</topic></contact>
    </sensor>
  </link>
  {''.join(wheel_links)}
  {''.join(wheel_joints)}
  <link name="laser"><pose relative_to="base_link">0.035 0 0.12 0 0 0</pose>
    <inertial><mass>0.1</mass><inertia><ixx>0.00002</ixx><iyy>0.00002</iyy><izz>0.00004</izz></inertia></inertial>
    {cylinder('lidar_housing', .026, .025, '0 0 -.018 0 0 0', '.12 .15 .19 1')}
    <sensor name="c1_like_lidar" type="gpu_lidar">
      <pose>0 0 0 0 0 0</pose><topic>/scan</topic><gz_frame_id>laser</gz_frame_id>
      <always_on>true</always_on><update_rate>10</update_rate><visualize>true</visualize>
      <!-- An idealized 720-ray approximation, not a calibrated C1 digital twin. -->
      <lidar><scan><horizontal><samples>720</samples><resolution>1</resolution>
        <min_angle>{-PI}</min_angle><max_angle>{PI}</max_angle></horizontal></scan>
        <range><min>0.05</min><max>12</max><resolution>0.001</resolution></range>
        <noise><type>gaussian</type><mean>0</mean><stddev>0.01</stddev></noise>
      </lidar>
    </sensor>
  </link>
  <joint name="laser_fixed_joint" type="fixed"><parent>base_link</parent><child>laser</child></joint>
  <link name="camera_link"><pose relative_to="base_link">0.12 0 0.10 0 0 0</pose>
    <inertial><mass>0.05</mass><inertia><ixx>0.00001</ixx><iyy>0.00001</iyy><izz>0.00001</izz></inertia></inertial>
    {box('camera_housing', '.025 .045 .025', '0 0 0 0 0 0', '.11 .12 .13 1')}
    <!-- Gazebo renders along camera_link +X. ROS header uses the optical frame. -->
    <sensor name="rgb_camera" type="camera">
      <topic>/camera/image_raw</topic><gz_frame_id>camera_optical_frame</gz_frame_id>
      <always_on>true</always_on><update_rate>15</update_rate><visualize>false</visualize>
      <camera><horizontal_fov>1.0471975511965976</horizontal_fov>
        <image><width>640</width><height>480</height><format>R8G8B8</format></image>
        <clip><near>0.03</near><far>12</far></clip>
        <camera_info_topic>/camera/camera_info</camera_info_topic><optical_frame_id>camera_optical_frame</optical_frame_id>
      </camera>
    </sensor>
  </link>
  <joint name="camera_fixed_joint" type="fixed"><parent>base_link</parent><child>camera_link</child></joint>
  <frame name="camera_optical_frame" attached_to="camera_link"><pose relative_to="camera_link">0 0 0 {-PI/2} 0 {-PI/2}</pose></frame>
  <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
    <left_joint>front_left_wheel_joint</left_joint><left_joint>rear_left_wheel_joint</left_joint>
    <right_joint>front_right_wheel_joint</right_joint><right_joint>rear_right_wheel_joint</right_joint>
    <wheel_separation>0.21</wheel_separation><wheel_radius>0.033</wheel_radius>
    <topic>/sim/cmd_vel</topic><odom_publish_frequency>20</odom_publish_frequency>
    <!-- NEVER bridge these wheel-integration outputs into the operational stack. -->
    <odom_topic>/sim/wheel_odom</odom_topic><tf_topic>/sim/wheel_tf</tf_topic>
    <frame_id>sim_wheel_odom</frame_id><child_frame_id>sim_wheel_base</child_frame_id>
    <min_linear_velocity>-0.12</min_linear_velocity><max_linear_velocity>0.12</max_linear_velocity>
    <min_angular_velocity>-0.5</min_angular_velocity><max_angular_velocity>0.5</max_angular_velocity>
    <min_linear_acceleration>-0.3</min_linear_acceleration><max_linear_acceleration>0.3</max_linear_acceleration>
    <min_angular_acceleration>-1.0</min_angular_acceleration><max_angular_acceleration>1.0</max_angular_acceleration>
  </plugin>
  <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
    <!-- True model pose. Evaluation ONLY; never remap to /odom or bridge its TF. -->
    <odom_frame>sim_world</odom_frame><robot_base_frame>base_link</robot_base_frame><dimensions>2</dimensions>
    <odom_publish_frequency>20</odom_publish_frequency><odom_topic>/sim/odom</odom_topic>
    <odom_covariance_topic>/sim/odom_covariance</odom_covariance_topic><tf_topic>/sim/ground_truth_tf</tf_topic>
  </plugin>
</model></sdf>
'''
    write(MODELS / "room_robot/model.sdf", sdf)
    write(MODELS / "room_robot/model.config", config("room_robot", "Four-wheel skid-steer robot; no encoder odometry in navigation; RF2O required."))
    urdf = f'''<?xml version="1.0"?>
<!-- TF/visual companion to models/room_robot/model.sdf. No control plugins or odometry. -->
<robot name="room_robot">
  <material name="rubber"><color rgba=".04 .045 .05 1"/></material>
  <material name="chassis_green"><color rgba=".18 .42 .22 1"/></material>
  <material name="sensor_dark"><color rgba=".11 .12 .13 1"/></material>
  <link name="base_link"><visual><origin xyz="0 0 .02"/><geometry><box size=".24 .17 .05"/></geometry><material name="chassis_green"/></visual><visual><origin xyz="0 0 .015"/><geometry><box size=".27 .25 .025"/></geometry><material name="sensor_dark"/></visual></link>
  {''.join(urdf_wheels)}
  <link name="laser"><visual><origin xyz="0 0 -.018"/><geometry><cylinder radius=".026" length=".025"/></geometry><material name="sensor_dark"/></visual></link>
  <joint name="laser_fixed_joint" type="fixed"><parent link="base_link"/><child link="laser"/><origin xyz=".035 0 .12"/></joint>
  <link name="camera_link"><visual><geometry><box size=".025 .045 .025"/></geometry><material name="sensor_dark"/></visual></link>
  <joint name="camera_fixed_joint" type="fixed"><parent link="base_link"/><child link="camera_link"/><origin xyz=".12 0 .10"/></joint>
  <link name="camera_optical_frame"/>
  <joint name="camera_optical_joint" type="fixed"><parent link="camera_link"/><child link="camera_optical_frame"/><origin xyz="0 0 0" rpy="{-PI/2} 0 {-PI/2}"/></joint>
</robot>
'''
    write(ROOT / "urdf/room_robot.urdf", urdf)


def human() -> None:
    # Deliberately simple and visibly synthetic. Not a detector accuracy test.
    parts = "".join(cylinder(f"{side}_leg", .065, .72, f"0 {y} .39 0 0 0", '.12 .15 .22 1')
                    for side, y in (("left", .095), ("right", -.095)))
    parts += box("torso", ".24 .38 .60", "0 0 1.05 0 0 0", '.65 .15 .11 1')
    parts += "".join(cylinder(f"{side}_arm", .055, .58, f"0 {y} 1.04 0 0 0", '.65 .15 .11 1')
                     for side, y in (("left", .245), ("right", -.245)))
    parts += f'''<collision name="head_collision"><pose>0 0 1.53 0 0 0</pose><geometry><sphere><radius>.13</radius></sphere></geometry></collision>
      <visual name="head_visual"><pose>0 0 1.53 0 0 0</pose><geometry><sphere><radius>.13</radius></sphere></geometry>{material('.72 .51 .36 1')}</visual>'''
    write(MODELS / "human_proxy/model.sdf", f'''<?xml version="1.0"?>
<!-- Procedural geometry proxy: collision and pipeline debugging, NOT a detector benchmark. -->
<sdf version="1.9">{static_model('human_proxy', '0 0 0 0 0 0', parts)}</sdf>
''')
    write(MODELS / "human_proxy/model.config", config("human_proxy", "Synthetic primitive human proxy for geometry and plumbing tests only. Not a perception benchmark or realistic person."))


def world() -> None:
    # All furniture coordinates are directly in this world for easy scene variation.
    objects = []
    objects.append(static_model("floor", "0 0 -.05 0 0 0", box("floor", "8.2 6.2 .1", "0 0 0 0 0 0", "1 1 1 1", "floor")))
    for name, size, pose in (("north_wall", "8.2 .12 2.4", "0 3.06 1.2 0 0 0"),
                             ("south_wall", "8.2 .12 2.4", "0 -3.06 1.2 0 0 0"),
                             ("east_wall", ".12 6.0 2.4", "4.06 0 1.2 0 0 0"),
                             ("west_wall", ".12 6.0 2.4", "-4.06 0 1.2 0 0 0")):
        objects.append(static_model(name, pose, box(name, size, "0 0 0 0 0 0", "1 1 1 1", "plaster")))
    table = box("table_top", "1.4 .8 .06", "0 0 .73 0 0 0", "1 1 1 1", "wood")
    for i, (x, y) in enumerate(((.61, .31), (.61, -.31), (-.61, .31), (-.61, -.31))):
        table += box(f"table_leg_{i}", ".07 .07 .70", f"{x} {y} .35 0 0 0", "1 1 1 1", "wood")
    objects.append(static_model("desk", "2.0 1.75 0 0 0 0", table))
    chair = box("chair_seat", ".45 .45 .07", "0 0 .43 0 0 0", "1 1 1 1", "wood")
    chair += box("chair_back", ".45 .06 .47", "0 .20 .69 0 0 0", "1 1 1 1", "wood")
    for i, (x, y) in enumerate(((.16, .16), (.16, -.16), (-.16, .16), (-.16, -.16))):
        chair += box(f"chair_leg_{i}", ".045 .045 .40", f"{x} {y} .20 0 0 0", ".13 .14 .15 1")
    objects.append(static_model("chair", "1.8 .93 0 0 0 .18", chair))
    sofa = box("sofa_base", "1.8 .80 .36", "0 0 .24 0 0 0", "1 1 1 1", "fabric")
    sofa += box("sofa_back", "1.8 .16 .48", "0 -.32 .62 0 0 0", "1 1 1 1", "fabric")
    for x in (-.82, .82):
        sofa += box(f"sofa_arm_{str(x).replace('-', 'minus')}", ".16 .8 .26", f"{x} 0 .51 0 0 0", "1 1 1 1", "fabric")
    for i, (x, y) in enumerate(((.7, .25), (.7, -.25), (-.7, .25), (-.7, -.25))):
        sofa += box(f"sofa_foot_{i}", ".1 .1 .12", f"{x} {y} .06 0 0 0", ".12 .11 .09 1")
    objects.append(static_model("sofa", "-2.2 -2.0 0 0 0 0", sofa))
    cabinet = box("cabinet_body", ".55 1.25 1.5", "0 0 .75 0 0 0", "1 1 1 1", "wood")
    for z in (.3, .75, 1.2):
        cabinet += box(f"drawer_line_{z}", ".012 1.18 .012", f".282 0 {z} 0 0 0", ".2 .15 .1 1", collision=False)
    objects.append(static_model("cabinet", "-3.45 1.65 0 0 0 0", cabinet))
    objects.append(static_model("small_box", "-.8 1.5 .15 0 0 -.2", box("box", ".40 .35 .30", "0 0 0 0 0 0", ".64 .44 .22 1", "wood")))
    # Low and tall obstacles deliberately exercise collision geometry at scanner height.
    objects.append(static_model("plant_proxy", "3.3 -2.15 0 0 0 0", cylinder("pot", .18, .35, "0 0 .175 0 0 0", ".48 .26 .13 1")
                                + cylinder("foliage", .30, .50, "0 0 .60 0 0 0", ".12 .38 .13 1")))
    sdf = f'''<?xml version="1.0"?>
<!-- Room fixtures use generated textures; human_proxy is NOT a detector benchmark. -->
<sdf version="1.9"><world name="room">
  <physics name="room_physics" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics>
  <gravity>0 0 -9.80665</gravity>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
  <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
  <scene><ambient>.45 .45 .45 1</ambient><background>.72 .76 .8 1</background><shadows>true</shadows></scene>
  <light name="ceiling_main" type="point"><pose>0 0 2.25 0 0 0</pose><diffuse>.9 .88 .82 1</diffuse><specular>.2 .2 .2 1</specular>
    <attenuation><range>15</range><constant>.9</constant><linear>.01</linear><quadratic>.001</quadratic></attenuation><cast_shadows>true</cast_shadows></light>
  <light name="window_fill" type="directional"><pose>2 -2 2.3 0 0 0</pose><diffuse>.5 .57 .65 1</diffuse><specular>.1 .1 .1 1</specular><direction>-.4 .3 -1</direction><cast_shadows>true</cast_shadows></light>
  {''.join(objects)}
  <include><uri>model://human_proxy</uri><name>person_proxy</name><pose>2 -1 0 0 0 3.141592653589793</pose></include>
  <!-- Keep one robot only; launch must not spawn another copy. -->
  <include><uri>model://room_robot</uri><name>room_robot</name><pose>0 0 0.034 0 0 0</pose></include>
</world></sdf>
'''
    write(ROOT / "worlds/room.sdf", sdf)
    write(MODELS / "room_environment/model.config", config("room_environment", "Locally generated, redistributable procedural texture resources for room.sdf."))
    # A minimal valid model makes the texture resource directory discoverable by Gazebo.
    write(MODELS / "room_environment/model.sdf", '''<?xml version="1.0"?>
<sdf version="1.9"><model name="room_environment"><static>true</static><link name="resource_origin"/></model></sdf>
''')


if __name__ == "__main__":
    textures()
    robot()
    human()
    world()
    print("Rebuilt room_sim models, procedural textures, URDF, and worlds/room.sdf")
