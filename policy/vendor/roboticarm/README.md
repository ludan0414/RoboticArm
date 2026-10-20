# Vendored Gazebo assets

This directory contains the Gazebo-side assets required by the Newton loading-cell demo.
The files are kept under the original ROS package layout so the Newton importer can
resolve the same S20 URDF and luggage model paths without a ROS installation or an
external RoboticArm checkout.

Included assets:

- `elfin_humble_ws/src/elfin_description`: S20 URDF source and link meshes.
- `elfin_humble_ws/src/luggage_description/config`: scene transforms and end-effector origins.
- `elfin_humble_ws/src/luggage_gazebo/models`: airport-container meshes, suction-panel collision meshes, and the arm adapter mesh.

The Newton scripts load this directory by default relative to their own source file.
Use `--source-root` only when intentionally selecting another asset tree.
