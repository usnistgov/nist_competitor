#include <test_competitor/test_competitor.hpp>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto test_competitor = std::make_shared<TestCompetitor>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(test_competitor);
  std::thread t([&executor]() { executor.spin(); });

  // Start Competition
  test_competitor->StartCompetition();

  test_competitor->LockAGVTray(1);

  t.join();

  executor.spin();

  rclcpp::shutdown();
}