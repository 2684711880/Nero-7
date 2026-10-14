# Model provenance

Source: https://github.com/smalleha/isaac_so_arm101
Commit: 53ab3a9d5cdb3246d6a1ea5a35947173d883e7f0
Path: src/isaac_so_arm101/robots/dual_nero
Upstream repository license retained in UPSTREAM_LICENSE.

Adaptations: package mesh URIs; camera mount locked at zero; world fixed joint; Gazebo ros2_control and MoveIt configs. Arm geometry, inertias, joint limits and mount transforms retained. Cameras are geometry only, with no image streams.

Same-arm collision exclusions are inherited from the installed AgileX single-arm SRDF. Fixed assemblies and adjacent assemblies are excluded; moving left-vs-right links remain checked. This is a simulation starting configuration, not measured hardware calibration or validated contact dynamics.

Recovery fix: register the Gazebo z=0 floor in the MoveIt scene before opening RViz; allow only fixed base_link to touch it. Refuse duplicate Gazebo master launches. No change to original gravity, solver, inertias or execution start tolerance.
