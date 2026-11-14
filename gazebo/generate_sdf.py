#!/usr/bin/env python3
"""Generate the Gazebo (gz-sim 8) quadcopter model + worlds from gazebo/params.yaml.

Outputs:
  gazebo/models/quadcopter/model.sdf   - X-frame quad, 4 MulticopterMotorModel rotors, IMU
  gazebo/worlds/free_flight.sdf        - open-air world (ground plane, quad at 0.3 m)
  gazebo/worlds/tether_rig.sdf         - spherical joint at the CG (approx. of the pole rig)

Frame: Gazebo FLU body (x fwd, y left, z up). Motor numbering = firmware:
  M0 front-right CCW, M1 front-left CW, M2 rear-left CCW, M3 rear-right CW.
"""
import math
import os
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
P = yaml.safe_load(open(os.path.join(HERE, "params.yaml")))
pl, sn = P["plant"], P["sensors"]

d = pl["arm_L"] / math.sqrt(2.0)          # lever arm about roll/pitch axes
OMEGA_MAX = 1000.0                        # rad/s (actuator range; thrust scaled to Tmax)
K_T = pl["Tmax"] / OMEGA_MAX ** 2         # N/(rad/s)^2
rm = pl["rotor_mass"]
base_m = pl["mass"] - 4 * rm
# rotors are point-ish masses at (+-d, +-d, 0); reduce base inertia so total J matches params
Ixx_b = pl["Ixx"] - 4 * rm * d * d
Iyy_b = pl["Iyy"] - 4 * rm * d * d
Izz_b = pl["Izz"] - 4 * rm * 2 * d * d
assert min(Ixx_b, Iyy_b, Izz_b) > 0

# (name, x, y, direction)  firmware M0..M3
ROTORS = [
    (0, +d, -d, "ccw"),
    (1, +d, +d, "cw"),
    (2, -d, +d, "ccw"),
    (3, -d, -d, "cw"),
]
NS = "quadcopter"
gyro_sd = math.radians(sn["sig_gyro_dps"])


def model_sdf(rig=False):
    s = [f'<model name="{NS}">', "  <pose>0 0 0.3 0 0 0</pose>"]
    s.append(f"""  <link name="base_link">
    <inertial><mass>{base_m:.5f}</mass>
      <inertia><ixx>{Ixx_b:.6f}</ixx><iyy>{Iyy_b:.6f}</iyy><izz>{Izz_b:.6f}</izz>
        <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
    <visual name="hub"><geometry><box><size>0.11 0.11 0.04</size></box></geometry>
      <material><ambient>0.15 0.15 0.15 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material></visual>
    <visual name="arm_fr_rl"><pose>0 0 0 0 0 -0.785398</pose><geometry><box><size>{2*pl['arm_L']:.3f} 0.025 0.015</size></box></geometry>
      <material><ambient>0.8 0.1 0.1 1</ambient><diffuse>0.9 0.1 0.1 1</diffuse></material></visual>
    <visual name="arm_fl_rr"><pose>0 0 0 0 0 0.785398</pose><geometry><box><size>{2*pl['arm_L']:.3f} 0.025 0.015</size></box></geometry>
      <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.9 0.9 0.9 1</diffuse></material></visual>
    <collision name="hub_col"><geometry><box><size>0.11 0.11 0.04</size></box></geometry></collision>
    <sensor name="imu" type="imu">
      <always_on>1</always_on><update_rate>{P['controller']['loop_hz']}</update_rate><topic>{NS}/imu</topic>
      <imu>
        <angular_velocity>
          <x><noise type="gaussian"><mean>0</mean><stddev>{gyro_sd:.6f}</stddev></noise></x>
          <y><noise type="gaussian"><mean>0</mean><stddev>{gyro_sd:.6f}</stddev></noise></y>
          <z><noise type="gaussian"><mean>0</mean><stddev>{gyro_sd:.6f}</stddev></noise></z>
        </angular_velocity>
      </imu>
    </sensor>
  </link>""")
    for i, x, y, dr in ROTORS:
        # prop disc collision-free; inertia of a thin disc about its axis
        s.append(f"""  <link name="rotor_{i}">
    <pose>{x:.5f} {y:.5f} 0.03 0 0 0</pose>
    <inertial><mass>{rm}</mass>
      <inertia><ixx>1e-5</ixx><iyy>1e-5</iyy><izz>2e-5</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
    <visual name="prop"><geometry><cylinder><radius>0.127</radius><length>0.004</length></cylinder></geometry>
      <material><ambient>0.1 0.4 0.9 0.5</ambient><diffuse>0.1 0.4 0.9 0.5</diffuse></material></visual>
  </link>
  <joint name="rotor_{i}_joint" type="revolute">
    <parent>base_link</parent><child>rotor_{i}</child>
    <axis><xyz>0 0 1</xyz><limit><lower>-1e12</lower><upper>1e12</upper><effort>1e6</effort></limit><dynamics><damping>0.0</damping></dynamics></axis>
  </joint>""")
    for i, x, y, dr in ROTORS:
        s.append(f"""  <plugin filename="gz-sim-multicopter-motor-model-system" name="gz::sim::systems::MulticopterMotorModel">
    <jointName>rotor_{i}_joint</jointName><linkName>rotor_{i}</linkName>
    <turningDirection>{dr}</turningDirection>
    <timeConstantUp>0.060</timeConstantUp><timeConstantDown>0.120</timeConstantDown>
    <maxRotVelocity>{OMEGA_MAX}</maxRotVelocity>
    <motorConstant>{K_T:.6e}</motorConstant>
    <momentConstant>{pl['kq']}</momentConstant>
    <commandSubTopic>command/motor_speed</commandSubTopic>
    <motorNumber>{i}</motorNumber>
    <rotorDragCoefficient>0</rotorDragCoefficient><rollingMomentCoefficient>0</rollingMomentCoefficient>
    <motorSpeedPubTopic>motor_speed/{i}</motorSpeedPubTopic>
    <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
    <motorType>velocity</motorType>
  </plugin>""")
    s.append("""  <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
    <dimensions>3</dimensions><odom_publish_frequency>200</odom_publish_frequency>
  </plugin>""")
    s.append("</model>")
    return "\n".join(s)


WORLD_HEAD = """<?xml version="1.0"?>
<sdf version="1.9">
<world name="{name}">
  <physics name="1ms" type="dart"><max_step_size>0.001</max_step_size><real_time_factor>{rtf}</real_time_factor></physics>
  <gravity>0 0 -{g}</gravity>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
  <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
  <plugin filename="gz-sim-apply-link-wrench-system" name="gz::sim::systems::ApplyLinkWrench"/>
  <light type="directional" name="sun"><cast_shadows>true</cast_shadows><pose>0 0 10 0 0 0</pose>
    <diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.5 0.1 -0.9</direction></light>
  <model name="ground"><static>true</static><link name="g">
    <collision name="c"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry></collision>
    <visual name="v"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
      <material><ambient>0.4 0.45 0.4 1</ambient><diffuse>0.5 0.55 0.5 1</diffuse></material></visual></link></model>
"""


def write(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write(text)
    print("wrote", path)


def main():
    m = model_sdf()
    write(os.path.join(HERE, "models", NS, "model.sdf"),
          '<?xml version="1.0"?>\n<sdf version="1.9">\n' + m + "\n</sdf>\n")
    write(os.path.join(HERE, "models", NS, "model.config"),
          '<?xml version="1.0"?>\n<model><name>quadcopter</name><version>1.0</version>'
          '<sdf version="1.9">model.sdf</sdf><description>EE302 F450 quad (matlab/params.m)</description></model>\n')

    free = WORLD_HEAD.format(name="free_flight", rtf=1.0, g=pl["g"]) + m + "\n</world>\n</sdf>\n"
    write(os.path.join(HERE, "worlds", "free_flight.sdf"), free)

    # Tether rig: ball joint 4.5 cm below the CG (matches pole mount under bottom frame plate).
    rig = WORLD_HEAD.format(name="tether_rig", rtf=1.0, g=pl["g"]) + m + """
  <model name="anchor"><pose>0 0 0.255 0 0 0</pose><link name="a">
    <inertial><mass>10</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial></link>
    <joint name="pin" type="fixed"><parent>world</parent><child>a</child></joint></model>
  <joint name="ball" type="ball"><pose>0 0 -0.045 0 0 0</pose><parent>anchor::a</parent><child>quadcopter::base_link</child></joint>
</world>
</sdf>
"""
    write(os.path.join(HERE, "worlds", "tether_rig.sdf"), rig)
    print(f"K_T={K_T:.3e}  d={d:.4f}  base_m={base_m:.3f}  base_I=({Ixx_b:.5f},{Iyy_b:.5f},{Izz_b:.5f})")


if __name__ == "__main__":
    main()
