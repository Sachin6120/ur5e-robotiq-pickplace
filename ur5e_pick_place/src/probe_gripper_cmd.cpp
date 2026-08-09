// probe_gripper_cmd.cpp — minimal standalone rclcpp_action GripperCommand
// client, for one specific question: does a C++ rclcpp_action::Client
// behave differently from `ros2 action send_goal` (CLI) or a persistent
// rclpy ActionClient, sending the IDENTICAL goal? See docs/HANDOFF_M3.md,
// "check the CLI vs node action-client timing difference." Deliberately
// as close as possible to gripper_close_and_hold()'s own send_and_wait
// lambda in m3_grasp.cpp, minus everything else that node does (no
// MoveIt, no TF, no scene.yaml) — isolating the ACTION CLIENT itself as
// the only variable under test.
//
// USAGE: probe_gripper_cmd <target_position> [max_effort]
#include <chrono>
#include <cstdio>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/gripper_command.hpp>

using GripperCommand = control_msgs::action::GripperCommand;
using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  double target = argc > 1 ? std::stod(argv[1]) : 0.096;
  double max_effort = argc > 2 ? std::stod(argv[2]) : 50.0;

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("probe_gripper_cmd");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  auto t_start = std::chrono::steady_clock::now();
  auto client = rclcpp_action::create_client<GripperCommand>(node, "/gripper_controller/gripper_cmd");

  fprintf(stderr, "waiting for action server...\n");
  if (!client->wait_for_action_server(10s)) {
    fprintf(stderr, "SERVER_NOT_AVAILABLE\n");
    return 1;
  }
  auto t_server_ready = std::chrono::steady_clock::now();

  GripperCommand::Goal goal;
  goal.command.position = target;
  goal.command.max_effort = max_effort;

  auto send_goal_future = client->async_send_goal(goal);
  if (send_goal_future.wait_for(10s) != std::future_status::ready) {
    fprintf(stderr, "SEND_GOAL_TIMEOUT\n");
    return 1;
  }
  auto goal_handle = send_goal_future.get();
  auto t_accepted = std::chrono::steady_clock::now();
  if (!goal_handle) {
    fprintf(stderr, "GOAL_REJECTED\n");
    return 1;
  }

  auto result_future = client->async_get_result(goal_handle);
  if (result_future.wait_for(90s) != std::future_status::ready) {
    fprintf(stderr, "RESULT_TIMEOUT\n");
    return 1;
  }
  auto wrapped = result_future.get();
  auto t_result = std::chrono::steady_clock::now();

  auto secs = [](auto a, auto b) {
      return std::chrono::duration<double>(b - a).count();
    };
  printf(
    "server_ready_after=%.3fs accepted_after=%.3fs result_after=%.3fs total=%.3fs\n",
    secs(t_start, t_server_ready), secs(t_server_ready, t_accepted),
    secs(t_accepted, t_result), secs(t_start, t_result));
  if (wrapped.result) {
    printf(
      "position=%.6f stalled=%s reached_goal=%s\n",
      wrapped.result->position, wrapped.result->stalled ? "true" : "false",
      wrapped.result->reached_goal ? "true" : "false");
  } else {
    printf("NO_RESULT_MESSAGE\n");
  }

  executor.cancel();
  if (spinner.joinable()) { spinner.join(); }
  rclcpp::shutdown();
  return 0;
}
