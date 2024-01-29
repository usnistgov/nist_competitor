#include <nist_competitor/nist_competitor.hpp>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto nist_competitor = std::make_shared<NistCompetitor>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(nist_competitor);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Start Competition
  nist_competitor->StartCompetition();

  nist_competitor->AddModelsToPlanningScene();

  sleep(2);

  // Move Robots to Home Poses
  nist_competitor->FloorRobotSendHome();
  nist_competitor->CeilingRobotSendHome();

  // Complete Orders
  nist_competitor->CompleteOrders();

  nist_competitor->EndCompetition();

  rclcpp::shutdown();
}