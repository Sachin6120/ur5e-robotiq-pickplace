"""full_cycle.launch.py — single-command orchestrator for the complete
UR5e + Robotiq pick-and-place demonstration.

    ros2 launch ur5e_pick_place full_cycle.launch.py

This is an ORCHESTRATION launch only: it does not implement any robot
behavior itself. It sequences the project's existing, already-validated
launch files and scripts in the same order proven by
docs/m3_run_full_cycle_trial_live.sh, replacing that script's bash `until`
loops with real ROS 2 launch readiness gates (RegisterEventHandler /
OnProcessExit) wherever a genuine signal exists, instead of arbitrary
sleeps.

SEQUENCE (each stage starts only after the previous stage's real
readiness signal, not a guessed delay):

  1. ur5e_robotiq_sim_control.launch.py
       Gazebo, robot_state_publisher, the three ros2_control controllers,
       and the table spawn (config/scene.yaml-driven, unmodified).
  2. Wait for gripper_controller and arm_controller to report `active`.
       Reuses scripts/lib/gz_settle.sh's own
       gz_wait_controller_active_bounded (sourced, not reimplemented) --
       the same function scripts/11_m3_cycles.sh's trial harness relies on.
  3. ur5e_robotiq_moveit_config/launch/move_group.launch.py
  4. Wait for the /move_group node to appear in `ros2 node list`, plus the
       same 3s settle buffer every existing trial script in this project
       takes after that (see docs/m3_run_full_cycle_trial_live.sh section
       2) -- move_group's action/service servers take a moment to finish
       registering after the node itself is listable.
  5. scripts/08_spawn_pick_object.sh -- the project's one existing
       object-spawn mechanism (derives the object SDF from
       config/scene.yaml's object: block, waits for it to settle onto the
       table via gz_settle_pose). Not reimplemented here.
  6. ur5e_pick_place/launch/m3_grasp.launch.py -- starts static_scene_tf
       and the m3_grasp node, which runs the full pick -> lift -> transport
       -> place -> release -> retreat cycle (transport.cpp). This is the
       actual demonstration; everything above it exists only to get here
       in a known-good state.

Any stage that exits nonzero stops the whole launch immediately (Shutdown)
rather than starting the next stage on top of a bad precondition -- this
project's standing rule is to record and stop on a failed precondition,
not retry or continue past it silently (see scripts/11_m3_cycles.sh's own
header on this point).

DELIBERATELY NOT INCLUDED: gz_assert_gripper_responsive (the
sim-degradation gate scripts/lib/gz_settle.sh also provides). That check
protects MEASUREMENT trust for research sweeps and has a documented,
harmless-to-the-robot false-failure rate on a perfectly healthy fresh sim
(docs/HANDOFF_M3.md, "A freshly-launched sim occasionally fails its own
preflight for an unknown reason"). The two gates used here (controllers
active, /move_group present) are the actual functional preconditions the
m3_grasp node needs; that third one is a research-trust check, not a
functional one, and would make a plain demo intermittently fail for no
behavioral reason.

M1 and M2 are standalone milestone checks (joint-goal-only,
approach-only) and are intentionally not part of this sequence -- the full
pick-and-place behavior lives entirely in m3_grasp.launch.py. Run
m1_joint_goal.launch.py / m2_cartesian_approach.launch.py directly, same
as before, for isolated debugging.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

REPO_ROOT = os.path.expanduser("~/ur5e_pickplace")


def _stop_or_continue(step_name, next_actions):
    # Shared on_exit handler: proceed to next_actions only on a clean exit;
    # otherwise stop the whole launch tree with a clear message naming the
    # stage that failed, rather than starting a downstream stage on top of
    # an unmet precondition.
    def _handler(event, context):
        if event.returncode != 0:
            return [
                LogInfo(
                    msg=f"[full_cycle] STOP: '{step_name}' exited "
                    f"{event.returncode} -- aborting rather than starting "
                    "the next stage on an unmet precondition."
                ),
                Shutdown(reason=f"full_cycle: {step_name} failed"),
            ]
        return next_actions

    return _handler


def generate_launch_description():
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    csv_path = LaunchConfiguration("csv_path")

    # --- Stage 1: sim + controllers + table --------------------------------
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ur5e_robotiq_description"),
                    "launch",
                    "ur5e_robotiq_sim_control.launch.py",
                ]
            )
        ),
        launch_arguments={"gazebo_gui": gazebo_gui}.items(),
    )

    # --- Stage 2: wait for controllers active -------------------------------
    wait_for_controllers = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            f'source "{REPO_ROOT}/scripts/lib/gz_settle.sh" && '
            "gz_wait_controller_active_bounded gripper_controller 30 && "
            "gz_wait_controller_active_bounded arm_controller 10",
        ],
        cwd=REPO_ROOT,
        name="full_cycle_wait_controllers",
        output="screen",
    )

    # --- Stage 3: MoveIt ------------------------------------------------
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ur5e_robotiq_moveit_config"),
                    "launch",
                    "move_group.launch.py",
                ]
            )
        )
    )

    # --- Stage 4: wait for /move_group to appear ----------------------------
    wait_for_move_group = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            "SECONDS=0; "
            "until ros2 node list 2>/dev/null | grep -q '^/move_group$'; do "
            'if [ "$SECONDS" -gt 40 ]; then '
            'echo "[full_cycle] STOP: /move_group node never appeared after 40s" >&2; '
            "exit 1; "
            "fi; "
            "sleep 0.5; "
            "done; "
            "sleep 3; "
            'echo "[full_cycle] /move_group node present after ${SECONDS}s (+3s settle buffer)"',
        ],
        cwd=REPO_ROOT,
        name="full_cycle_wait_move_group",
        output="screen",
    )

    # --- Stage 5: spawn the pick object -------------------------------------
    spawn_object = ExecuteProcess(
        cmd=["bash", os.path.join(REPO_ROOT, "scripts", "08_spawn_pick_object.sh")],
        cwd=REPO_ROOT,
        name="full_cycle_spawn_object",
        output="screen",
    )

    # --- Stage 6: the actual pick-and-place cycle ---------------------------
    pick_place_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur5e_pick_place"), "launch", "m3_grasp.launch.py"]
            )
        ),
        launch_arguments={"csv_path": csv_path}.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gazebo_gui",
                default_value="true",
                description="Show the Gazebo GUI. This is a demonstration "
                "launch, so it defaults to visible, unlike "
                "ur5e_robotiq_sim_control.launch.py's own headless default. "
                "Pass gazebo_gui:=false for a faster/headless run.",
            ),
            DeclareLaunchArgument(
                "csv_path",
                default_value="m3_grasp.csv",
                description="Passed straight through to m3_grasp.launch.py's "
                "own csv_path arg -- where the per-run evidence CSV is written.",
            ),
            sim_launch,
            wait_for_controllers,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=wait_for_controllers,
                    on_exit=_stop_or_continue(
                        "wait_for_controllers", [move_group_launch, wait_for_move_group]
                    ),
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=wait_for_move_group,
                    on_exit=_stop_or_continue("wait_for_move_group", [spawn_object]),
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_object,
                    on_exit=_stop_or_continue("spawn_object", [pick_place_launch]),
                )
            ),
        ]
    )
