# Third-party sources

## ICP hand-eye solver

`third_party/eye_hand_calib_pointcloud` is the Linux RegHEC fork from
https://github.com/payam-nourizadeh/eye_hand_calib_pointcloud at commit
`2cb4f0a98f595b5731490771fb551e13d20339d8`. Its README declares MIT but the
referenced LICENSE file is absent from this checkout. `scripts/reghec_compat.patch`
adapts current Sophus and accepts an optional initial matrix; the AA-ICP optimizer
is otherwise retained. Limbo (`43c67a6999a40a3349aa6d0f152f853e028ed509`,
CeCILL-C) and Sophus (`d0b7315a0d90fc6143defa54596a3a95d9fa10ec`, MIT)
retain their upstream license files. NLopt runtime is extracted locally from
Ubuntu packages, not installed system-wide.

## Hand-eye RViz components

`calibration_ws/src/moveit_calibration` and `calibration_ws/src/moveit_visual_tools`
are imported from the local ArmHub source snapshot. Upstream repositories and
commits are recorded in `calibration_ws/sources.json`; original package licenses,
copyright headers, and existing Humble compatibility changes (`imported.patch`)
are retained. NERO changes retain configured TF frame names during discovery and
initialize the sensor-mount selection index.

`scripts/realsense_rectify.py` reuses the local ArmHub SDK-deprojection map helper.

## AgileX RANGER AIR

- Source: https://github.com/agilexrobotics/ugv_gazebo_sim
- Commit: `27633a956c845903ee630538afeb17fe70afdd84`
- Source path: `ranger_air/ranger_air_description`
- Local path: `sim_ws/src/ranger_air_description`
- The original package.xml declares the license as BSD. No additional license text was present in the inspected AIR package; this notice records the upstream declaration without inventing additional terms.
- Original CAD URDF and meshes retained. Local ament/launch/RViz files added. The motion variant is generated separately with simplified collision shapes and ros2_control interfaces.

## Dual NERO model

- Source: https://github.com/smalleha/isaac_so_arm101
- Commit: `53ab3a9d5cdb3246d6a1ea5a35947173d883e7f0`
- Source path: `src/isaac_so_arm101/robots/dual_nero`
- Local path: `sim_ws/src/dual_nero_moveit_config`
- Original license text retained in `sim_ws/src/dual_nero_moveit_config/UPSTREAM_LICENSE`.
- Additional provenance and adaptation details: `sim_ws/src/dual_nero_moveit_config/ASSET_SOURCE.md`.

## AgileX arm examples (submodule)

- Source: https://github.com/agilexrobotics/agx_arm_sim
- Pinned commit: `f8cd8b147c75d59e14f90fb0646770eefa268ed0`
- Nested agx_arm_urdf source: https://github.com/agilexrobotics/agx_arm_urdf
- Nested pinned commit: `1ee5659d02b33b9379fd647a2e5647800d67f0f4`
- Local optional build patch: `deps/agx_arm_sim-cmake.patch`.
- Submodules retain their own upstream license notices and history.

Source and runtime validation evidence are separate: the snapshots and reports in sim_ws/validation document local simulator runs and do not establish real-hardware correctness or calibrated payload limits.

## AgileX official ROS driver and SDK (submodules)

Local changes to the pinned ROS driver are preserved in `deps/agx_arm_ros-local.patch`;
`scripts/build_official_control.sh` applies the patch when preparing a checkout.

- ROS driver: https://github.com/agilexrobotics/agx_arm_ros
  - Pinned commit: `bc485ab8e3d25abbcd3a1b58c105be12bad5dadc`
  - Local path: `third_party/agx_arm_ros`
  - Upstream LICENSE and package manifests declare Apache-2.0.
- Python SDK: https://github.com/agilexrobotics/pyAgxArm
  - Pinned commit: `1ae40ad4755ca25ad32bece3ab42325698185fc7`
  - Local path: `third_party/pyAgxArm`
  - Upstream LICENSE retains LGPL v3; setup.py declares LGPL-3.0-only.
- Upstream source files are retained without edits. `hardware_ws/src/agx_arm_ctrl`
  and `agx_arm_msgs` link to the original ROS packages. Our C++ input examples are
  adapted from the upstream `test/nero/test_move_l.yaml` and `test_move_c.yaml`.
