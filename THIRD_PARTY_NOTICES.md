# Third-Party Notices

This repository contains original work, modified third-party source, generated
configuration, and references to separately installed ROS packages. It is not
accurately described by a single license covering every file.

## License scope

| Scope | Governing license |
|---|---|
| Original repository material outside the ROS packages, unless a file says otherwise | [MIT](LICENSE) |
| Original content in `ur5e_pick_place` | [Apache-2.0](LICENSES/Apache-2.0.txt), as declared in its `package.xml` |
| Original integration content in `ur5e_robotiq_description` | [Apache-2.0](LICENSES/Apache-2.0.txt), as declared in its `package.xml` |
| The two modified Robotiq xacros listed below | [BSD-3-Clause](LICENSES/robotiq_description-BSD-3-Clause.txt) |
| `ur5e_robotiq_moveit_config` | [BSD-3-Clause](LICENSES/moveit2-BSD-3-Clause.txt), as declared in its `package.xml` |

The root `LICENSE` is therefore the default for original root-level material,
not a relicensing of package-scoped or third-party work. A more specific file
notice, package manifest, or entry in this document takes precedence for its
stated scope.

## Robotiq description files

Upstream project: [PickNik Robotics `ros2_robotiq_gripper`](https://github.com/PickNikRobotics/ros2_robotiq_gripper)

Upstream package: `robotiq_description`, version `0.0.1` in the ROS 2 Jazzy
binary package used when the local copies were made.

Upstream files and local derivatives:

| Upstream file | Local modified file |
|---|---|
| `robotiq_description/urdf/robotiq_2f_85_macro.urdf.xacro` | `ur5e_robotiq_description/urdf/vendor/robotiq_2f_85_macro.urdf.xacro` |
| `robotiq_description/urdf/robotiq_gripper.ros2_control.xacro` | `ur5e_robotiq_description/urdf/vendor/robotiq_gripper.ros2_control.xacro` |

The local copies preserve the upstream structure and add project-specific
simulation changes. At a high level these changes cover linkage velocity and
effort limits, fingertip-joint topology, and ros2_control effort/state
interfaces. The detailed changes and their dates are documented inline at the
top of each local file.

This repository records the ROS package version but not the exact
upstream Git revision. No commit SHA is asserted here. The audited Jazzy binary
package identifies PickNik's repository as upstream and declares BSD; the
upstream repository carries the BSD-3-Clause license reproduced in
[`LICENSES/robotiq_description-BSD-3-Clause.txt`](LICENSES/robotiq_description-BSD-3-Clause.txt).

Robotiq mesh files are not copied into this repository. The local xacros keep
`package://robotiq_description/...` mesh references, so those assets remain a
separately installed runtime dependency governed by their upstream package.

## Universal Robots description

Upstream project: [Universal Robots ROS 2 Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)

`ur5e_robotiq_description/urdf/ur5e_robotiq.urdf.xacro` composes the installed
`ur_description` xacro macros. UR meshes and `ur_description` source files are
not copied into this repository. In the audited environment the installed
package was `ur_description` 3.5.1; the repository itself does not pin an
upstream Git revision.

The installed `ur_description` package declares BSD-3-Clause for its software
and separately identifies Universal Robots terms for specified graphical
documentation. Consumers should use the notices delivered with their installed
`ur_description` version. Because this repository does not redistribute those
assets, it does not reproduce or reinterpret their terms here.

## MoveIt-generated configuration

Upstream project: [MoveIt 2](https://github.com/moveit/moveit2)

`ur5e_robotiq_moveit_config` began as output from the MoveIt Setup Assistant
and was subsequently adapted for this simulated platform. The retained
`.setup_assistant` file records the input URDF/SRDF and generator metadata; it
is standard regeneration metadata, not a runtime credential. The exact
upstream MoveIt Git revision was not recorded. The audited environment used
MoveIt Setup Assistant 2.12.4.

The package declares BSD-3-Clause. The MoveIt 2 license text is reproduced in
[`LICENSES/moveit2-BSD-3-Clause.txt`](LICENSES/moveit2-BSD-3-Clause.txt).

## Repository media

The files currently under `docs/assets/` were added in repository history as
project demonstration and perception visuals. They are simulation captures,
not copies of UR or Robotiq mesh source files. They do render models supplied
by the external description packages identified above, so this statement does
not replace any notice that accompanies those installed packages.

The planned `docs/assets/d3_demo_thumbnail.png` is not present in this release
preparation commit. No D3 thumbnail or full demonstration video is being
distributed here.

## Dependency boundary

ROS 2, Gazebo, MoveIt 2, OpenCV, OMPL, `ros2_control`, `gz_ros2_control`, and
the installed robot-description packages remain external dependencies. This
notice focuses on source or generated material present in this repository; it
does not replace the license notices distributed with those dependencies.
