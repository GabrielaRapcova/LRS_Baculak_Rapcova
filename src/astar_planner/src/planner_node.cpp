#include "astar_planner/planner_node.hpp"

#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace astar_planner
{
using std::placeholders::_1;
using std::placeholders::_2;

PlannerNode::PlannerNode()
: Node("astar_planner")
{
  declareParameters();
  configureInterfaces();
  RCLCPP_INFO(get_logger(), "Ready for ComputePath3D requests; waiting for /costmap");
}

PlannerNode::~PlannerNode()
{
  stop_requested_ = true;
  if (planning_thread_.joinable()) {
    planning_thread_.join();
  }
}

void PlannerNode::declareParameters()
{
  const auto max_expansions = declare_parameter<int64_t>("max_expansions", 200000);
  const auto max_search_nodes = declare_parameter<int64_t>("max_search_nodes", 500000);
  const auto max_los_voxels = declare_parameter<int64_t>("max_los_voxels", 100000);
  search_limits_.max_planning_time = declare_parameter("max_planning_time", 10.0);
  search_limits_.cost_weight = declare_parameter("cost_weight", 1.0);
  if (max_expansions <= 0 || max_expansions > UINT32_MAX ||
    max_search_nodes <= 0 || max_search_nodes > UINT32_MAX || max_los_voxels <= 0 ||
    !std::isfinite(search_limits_.max_planning_time) || search_limits_.max_planning_time <= 0 ||
    !std::isfinite(search_limits_.cost_weight) || search_limits_.cost_weight < 0)
  {
    throw std::invalid_argument(
        "Planner limits must be positive; cost_weight finite and nonnegative");
  }
  search_limits_.max_expansions = max_expansions;
  search_limits_.max_search_nodes = max_search_nodes;
  search_limits_.max_line_of_sight_voxels = max_los_voxels;
  declare_parameter("publish_plan", true);
}

void PlannerNode::configureInterfaces()
{
  auto snapshot_qos = rclcpp::QoS(1).reliable().transient_local();
  if (get_parameter("publish_plan").as_bool()) {
    path_publisher_ = create_publisher<nav_msgs::msg::Path>("plan", snapshot_qos);
  }
  costmap_subscription_ = create_subscription<uav_navigation_msgs::msg::VoxelCostmap>(
    "costmap", snapshot_qos, std::bind(&PlannerNode::costmapCallback, this, _1));
  planning_server_ = rclcpp_action::create_server<ComputePath3D>(
    this, "compute_path_3d",
    std::bind(&PlannerNode::handlePlanningGoal, this, _1, _2),
    std::bind(&PlannerNode::handlePlanningCancellation, this, _1),
    std::bind(&PlannerNode::startPlanning, this, _1));
}

void PlannerNode::costmapCallback(uav_navigation_msgs::msg::VoxelCostmap::ConstSharedPtr message)
{
  try {
    auto next_costmap = std::make_shared<const VoxelGrid>(*message);
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    if (costmap_ && next_costmap->map_id == costmap_->map_id &&
      next_costmap->sequence < costmap_->sequence)
    {
      return;
    }
    costmap_ = std::move(next_costmap);
  } catch (const std::exception & error) {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    costmap_.reset();
    RCLCPP_ERROR(get_logger(), "Invalid costmap: %s", error.what());
  }
}

rclcpp_action::GoalResponse PlannerNode::handlePlanningGoal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const ComputePath3D::Goal>)
{
  if (planning_active_.exchange(true)) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PlannerNode::handlePlanningCancellation(std::shared_ptr<GoalHandle>)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PlannerNode::startPlanning(std::shared_ptr<GoalHandle> goal_handle)
{
  if (planning_thread_.joinable()) {
    planning_thread_.join();
  }
  planning_thread_ = std::thread(&PlannerNode::executePlanning, this, goal_handle);
}

void PlannerNode::executePlanning(const std::shared_ptr<GoalHandle> & goal_handle)
{
  auto output = std::make_shared<ComputePath3D::Result>();
  try {
    std::shared_ptr<const VoxelGrid> costmap;
    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      costmap = costmap_;
    }
    auto goal = goal_handle->get_goal();
    if (!costmap) {
      output->status = ComputePath3D::Result::NO_MAP;
      output->message = "No valid costmap received";
    } else if (goal->start.header.frame_id != costmap->frame_id) {
      output->status = ComputePath3D::Result::INVALID_START;
      output->message = "Start frame must match costmap frame";
    } else if (goal->goal.header.frame_id != costmap->frame_id) {
      output->status = ComputePath3D::Result::INVALID_GOAL;
      output->message = "Goal frame must match costmap frame";
    } else {
      const auto & start_position = goal->start.pose.position;
      const auto & goal_position = goal->goal.pose.position;
      auto result = findPath(
        *costmap, {start_position.x, start_position.y, start_position.z},
        {goal_position.x, goal_position.y, goal_position.z}, search_limits_,
        [this, goal_handle]() {
          return stop_requested_ || !rclcpp::ok() || goal_handle->is_canceling();
        },
        [goal_handle](size_t expanded_nodes) {
          auto feedback = std::make_shared<ComputePath3D::Feedback>();
          feedback->expanded_nodes = expanded_nodes;
          goal_handle->publish_feedback(feedback);
        });
      output->status = result.status;
      output->message = result.message;
      output->planning_time = result.planning_time;
      output->expanded_nodes = result.expanded_nodes;
      output->raw_point_count = result.raw_point_count;
      output->refined_point_count = result.path.size();
      output->path.header.frame_id = costmap->frame_id;
      output->path.header.stamp = now();
      for (auto point : result.path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = output->path.header;
        pose.pose.position.x = point.x;
        pose.pose.position.y = point.y;
        pose.pose.position.z = point.z;
        pose.pose.orientation.w = 1;
        output->path.poses.push_back(pose);
      }
      RCLCPP_INFO(
        get_logger(),
        "Planning status %u, %.3f s, %u expanded, points %u -> %u (map %lu, sequence %lu)",
        output->status, output->planning_time, output->expanded_nodes, output->raw_point_count,
        output->refined_point_count, costmap->map_id, costmap->sequence);
    }
    if (goal_handle->is_canceling()) {
      output->status = ComputePath3D::Result::CANCELLED;
      output->message = "Planning cancelled";
      output->path.poses.clear();
      output->refined_point_count = 0;
      goal_handle->canceled(output);
    } else if (output->status == ComputePath3D::Result::SUCCESS) {
      if (path_publisher_) {
        path_publisher_->publish(output->path);
      }
      goal_handle->succeed(output);
    } else {
      goal_handle->abort(output);
    }
  } catch (const std::exception & error) {
    output->status = ComputePath3D::Result::INTERNAL_ERROR;
    output->message = error.what();
    output->path.poses.clear();
    output->refined_point_count = 0;
    try {
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(output);
      } else {
        goal_handle->abort(output);
      }
    } catch (...) {
      // The action may already be unavailable during shutdown; still release the worker.
    }
    RCLCPP_ERROR(get_logger(), "Planning failed: %s", error.what());
  }
  planning_active_ = false;
}

}  // namespace astar_planner
