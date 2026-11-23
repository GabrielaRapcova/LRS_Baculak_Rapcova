#ifndef ASTAR_PLANNER__PLANNER_NODE_HPP_
#define ASTAR_PLANNER__PLANNER_NODE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

#include "astar_planner/search.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "uav_navigation_msgs/action/compute_path3_d.hpp"
#include "uav_navigation_msgs/msg/voxel_costmap.hpp"

namespace astar_planner
{

/**
 * @brief Serve planning requests against the latest complete costmap.
 *
 * ROS callbacks run serially; search runs on one worker thread. The worker owns
 * a shared immutable snapshot, so costmap updates do not change an active search.
 */
class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode();
  ~PlannerNode() override;

private:
  using ComputePath3D = uav_navigation_msgs::action::ComputePath3D;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ComputePath3D>;

  void declareParameters();
  void configureInterfaces();
  void costmapCallback(uav_navigation_msgs::msg::VoxelCostmap::ConstSharedPtr message);
  rclcpp_action::GoalResponse handlePlanningGoal(
    const rclcpp_action::GoalUUID & goal_id, std::shared_ptr<const ComputePath3D::Goal> goal);
  rclcpp_action::CancelResponse handlePlanningCancellation(std::shared_ptr<GoalHandle> goal_handle);
  void startPlanning(std::shared_ptr<GoalHandle> goal_handle);
  void executePlanning(const std::shared_ptr<GoalHandle> & goal_handle);

  SearchLimits search_limits_;
  std::mutex costmap_mutex_;
  std::shared_ptr<const VoxelGrid> costmap_;
  std::atomic<bool> planning_active_{false};
  std::atomic<bool> stop_requested_{false};
  std::thread planning_thread_;
  rclcpp::Subscription<uav_navigation_msgs::msg::VoxelCostmap>::SharedPtr costmap_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp_action::Server<ComputePath3D>::SharedPtr planning_server_;
};

}  // namespace astar_planner

#endif  // ASTAR_PLANNER__PLANNER_NODE_HPP_
