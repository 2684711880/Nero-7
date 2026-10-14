# Third-party sources

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
