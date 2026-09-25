# Bugs I found in the code

I found them by building, running and measuring the robot and I got the numbers from the simulation.

## 1. Workspace doesn't build

`testbed_description/CMakeLists.txt` ends with `ament_package` without `()`. CMake reads that as a
parse error, so `testbed_description` fails and `testbed_bringup` is aborted.

Fix: `ament_package()`.

## 2. Map yaml points to a missing image

`testbed_world.yaml` has `image: wrong_path_testbed_world.pgm`. The file is `testbed_world.pgm`, so
`map_server` fails to activate.

Fix: corrected the filename.

## 3. Map not installed

`testbed_bringup/CMakeLists.txt` only installs `launch/`, so there is no
`share/testbed_bringup/maps` after a build.

Fix: install `maps/` as well.

## 4. Lidar range is 1.5 m

`testbed.gazebo` sets `<max>1.5</max>`. In a 20 x 20 m arena that leaves AMCL almost nothing to
match against and the costmaps nearly empty.

Fix: 12 m.

## 5. Wrong wheel separation

The diff drive plugin uses 0.35 m. Working it out from the wheel meshes and joint origins gives
0.376 m, with the wheel centres at y = +0.150 and y = -0.226 in `base_link`.

It doesn't show up in odometry, because this plugin publishes the true pose. It shows up in the
turn rate. Commanding 0.6 rad/s for 12 s gave a steady 0.549 rad/s before the fix and 0.590 rad/s
after. The ratio, 0.931, matches 0.35 / 0.376. Linear speed isn't affected.

Fix: `0.376`. The wheel diameter, 0.1 m, was already right.

## 6. base_footprint isn't at the rotation centre

The chassis and the drive axis are both centred at (-0.051, -0.038) in `base_link`, and
`fixed_base_joint` is `0 0 0`. So `base_footprint`, which odometry and the costmaps use, sits 63 mm
from the point the robot turns about. Spinning in place swings it around a 63 mm radius.

Fix: `fixed_base_joint` origin set to `0.0507 0.0379 0`. The same spin now moves it 2 mm. This is
also why `robot_radius` is 0.25 m; with the old frame it would have to be 0.32 m.

## 7. IMU and dummy frames swapped

`imu_joint` puts `imu_link_1` where the dummy mesh is and the other way round, so the IMU frame is
about 0.17 m from the IMU. Navigation doesn't use the IMU, but the TF tree was wrong.

Fix: the joint and visual origins now put each frame on its own mesh.

## 8. Missing package dependencies

`testbed_description` only declared `urdf` but uses `xacro`, `robot_state_publisher`,
`joint_state_publisher` and `rviz2`. `testbed_gazebo` didn't declare `gazebo_ros`.
`testbed_bringup` didn't declare the packages it launches, and had an unused `find_package(rclpy)`.
`rosdep` can't set up a machine from that.

Fix: added the missing entries and removed the unused `find_package`.

## 9. Gazebo models not installed

`spawn_playground.launch.py` adds `share/testbed_gazebo/models` to `GAZEBO_MODEL_PATH`, but
`models/` was never installed. Nothing breaks today because the world inlines its geometry.

Fix: install `models/`.

## 10. robot_state_publisher on wall time

No `use_sim_time`, while Gazebo publishes `/clock`. It doesn't break anything here: every joint
below `base_link` is fixed, so those transforms go out on `/tf_static`, and `odom -> base_footprint`
comes from Gazebo. It was still the only node on the wrong clock.

Fix: `use_sim_time: True`.

## 11. ROS 1 leftovers

`testbed.gazebo` loads `libgazebo_ros_control.so`, which doesn't exist in Humble, and
`testbed.trans` has ROS 1 transmissions for it. The model spawns fine without them, since the diff
drive plugin drives the wheels.

Fix: removed both.

## 12. Stray `>` in the IMU plugin

`</initial_orientation_as_reference>>`. A bare `>` is valid XML text, so it parsed and xacro dropped
it. Cosmetic.

Fix: removed.

## Things that pose as bugs, but still work correctly

- `spawn_playground.launch.py` declares `world` and never passes it to `gazebo.launch.py`. It still
  works, because included launch files share the parent's launch configurations. `gzserver` starts
  with `testbed_playground.world`.
- The map origin, `[-10.2, -9.94]`. I rasterized the world's collision geometry at lidar height and
  matched it against the map. The best fit is within one pixel of zero offset.
- The four casters are fixed joints, so they drag instead of rolling. The robot still tracks commands
  (0.20 m/s commanded, 0.200 m/s measured), so I left them as they are.
