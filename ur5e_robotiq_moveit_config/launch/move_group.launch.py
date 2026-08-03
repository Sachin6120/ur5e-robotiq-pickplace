from launch import LaunchDescription
from launch_ros.actions import SetParameter
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ur5e_robotiq", package_name="ur5e_robotiq_moveit_config").to_moveit_configs()
    move_group_ld = generate_move_group_launch(moveit_config)

    # CORRECTED (2026-08-04): generate_move_group_launch() does not set
    # use_sim_time and exposes no launch arg for it, so move_group defaulted
    # to wall-clock time while /clock (and everything downstream of the
    # controller_manager) runs on sim time. That mismatch doesn't show up as a
    # missing-joint problem — it breaks trajectory_execution_manager's
    # timestamp-freshness check on /joint_states: every message looks
    # impossibly stale when compared against wall-clock "now" versus a sim
    # clock only ~600s past epoch. Symptom was 20/20 plans succeeding, then
    # every execute() aborting with "couldn't receive full current joint
    # state within 1s" — confirmed live via `ros2 param get /move_group
    # use_sim_time` returning False while /clock was already running.
    #
    # SetParameter must be placed BEFORE the move_group node action in
    # execution order, not appended after — launch actions apply to nodes
    # launched after them, and generate_move_group_launch() already has the
    # node action built into its returned LaunchDescription. Prepending via
    # .entities, rather than adding after, is what makes it actually take
    # effect on this specific node.
    return LaunchDescription(
        [
            SetParameter(name="use_sim_time", value=True),
            *move_group_ld.entities,
        ]
    )
