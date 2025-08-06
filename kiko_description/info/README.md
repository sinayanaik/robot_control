# Kikobot Robot Description

## Overview
Kikobot is a 6-DOF (Degrees of Freedom) robotic arm designed for precise manipulation tasks. The robot consists of a base and five arm segments, culminating in an end coupler for tool attachment.

## Robot Structure

### Links
1. **world**
   - Fixed world reference frame
   - Used as the root frame for visualization and control

2. **Base**
   - Main base structure
   - Connected to world via fixed joint
   - Mass: 0.0 (fixed base)
   - Mesh: Base.stl
   - Oriented vertically using RPY: [1.5708, 0, 0]

3. **Arm-1**
   - First movable segment
   - Mass: 1.0688 kg
   - Connected to Base via Base_Revolute-1 joint
   - Mesh: Arm-1.stl

4. **Arm-2**
   - Second arm segment
   - Mass: 2.5174 kg
   - Connected to Arm-1 via Arm-1_Revolute-2 joint
   - Mesh: Arm-2.stl

5. **Arm-3**
   - Third arm segment
   - Mass: 2.1518 kg
   - Connected to Arm-2 via Arm-2_Revolute-3 joint
   - Mesh: Arm-3.stl

6. **Arm-4**
   - Fourth arm segment
   - Mass: 0.8427 kg
   - Connected to Arm-3 via Arm-3_Revolute-4 joint
   - Mesh: Arm-4.stl

7. **Arm-5**
   - Fifth arm segment
   - Mass: 0.9001 kg
   - Connected to Arm-4 via Arm-4_Revolute-5 joint
   - Mesh: Arm-5.stl

8. **End-Coupler-v1**
   - End effector mounting point
   - Mass: 0.1163 kg
   - Connected to Arm-5 via Arm-5_Revolute-6 joint
   - Mesh: End-Coupler-v1.stl

## Joint Configuration

### Joint Limits and Specifications

1. **world_joint** (world → Base)
   - Type: Fixed
   - Origin: [0, 0, 0]
   - RPY: [0, 0, 0]

2. **Base_Revolute-1** (Base → Arm-1)
   - Type: Revolute
   - Axis: [0, 0, 1.0]
   - Limits: -2.879793 to 2.879793 rad (±165°)
   - Origin: [0.00065556, 0.0, 0.08255024]
   - RPY: [0, 0, 0]

3. **Arm-1_Revolute-2** (Arm-1 → Arm-2)
   - Type: Revolute
   - Axis: [1.0, 8.53e-33, 0.0]
   - Limits: -2.181662 to 2.181662 rad (±125°)
   - Origin: [3.33e-18, 0.0417, 0.04975]
   - RPY: [-1.5708, -2.78e-17, -1.5708]

4. **Arm-2_Revolute-3** (Arm-2 → Arm-3)
   - Type: Revolute
   - Axis: [0.0, 0.0, -1.0]
   - Limits: -2.443461 to 2.443461 rad (±140°)
   - Origin: [0.003, -0.133, 0.0]
   - RPY: [1.97e-31, -1.5708, 0]

5. **Arm-3_Revolute-4** (Arm-3 → Arm-4)
   - Type: Revolute
   - Axis: [1.23e-32, -1.97e-31, 1.0]
   - Limits: -2.443461 to 2.443461 rad (±140°)
   - Origin: [0.0, -0.116, -0.003]
   - RPY: [-1.97e-31, -1.23e-32, 1.45e-33]

6. **Arm-4_Revolute-5** (Arm-4 → Arm-5)
   - Type: Revolute
   - Axis: [0.0, 1.0, 0.0]
   - Limits: -2.443461 to 2.443461 rad (±140°)
   - Origin: [0.0, -0.0368, 0.0488]
   - RPY: [-3.1416, 1.5708, 0]

7. **Arm-5_Revolute-6** (Arm-5 → End-Coupler-v1)
   - Type: Revolute
   - Axis: [-1.0, -9.68e-33, 0.0]
   - Limits: -3.054326 to 3.054326 rad (±175°)
   - Origin: [-0.04, 0.0488, 0.0]
   - RPY: [5.81e-66, -6.00e-34, 9.68e-33]

## Kinematic Chain
```
world [⚓]
    └── world_joint [⚓] => /Base/
        └── Base_Revolute-1 [⚙] => /Arm-1/
            └── Arm-1_Revolute-2 [⚙] => /Arm-2/
                └── Arm-2_Revolute-3 [⚙-Z] => /Arm-3/
                    └── Arm-3_Revolute-4 [⚙] => /Arm-4/
                        └── Arm-4_Revolute-5 [⚙+Y] => /Arm-5/
                            └── Arm-5_Revolute-6 [⚙] => /End-Coupler-v1/
```

## Visualization
- All meshes are in STL format
- Scale factor for all meshes: 0.001 (millimeter to meter conversion)
- Collision geometries use the same meshes as visual geometries
- Base link is oriented vertically for proper robot stance
- World frame is used as the fixed reference frame in RViz

## Usage
1. Launch with RViz visualization:
   ```bash
   ros2 launch kiko_description rviz_launch.py
   ```

2. View with URDF visualizer:
   ```bash
   urdf-viz src/kiko_description/urdf/Kikobot.urdf
   ```

## Technical Specifications
- Total DOF: 6
- Total Mass: ~7.6 kg
- All joints have high torque capacity (effort: 1000000)
- All joints have high velocity limits (velocity: 1000000)
- Primary materials: Rigid body construction with precise inertial properties

## Notes
- The world link serves as the fixed reference frame
- The Base link is oriented vertically (90° around X-axis) for proper robot stance
- Each joint has specific orientation and movement constraints
- The robot follows a standard serial manipulator configuration
- End effector can be mounted on the End-Coupler-v1 link
- The Base has zero mass as it's designed to be fixed