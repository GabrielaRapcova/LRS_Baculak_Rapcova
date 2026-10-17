#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <uav_navigation_msgs/action/compute_path3_d.hpp>
#include "astar_planner/search.hpp"

class Planner : public rclcpp::Node
{
  using Action = uav_navigation_msgs::action::ComputePath3D;
  using Handle = rclcpp_action::ServerGoalHandle<Action>;
public:
  Planner() : Node("astar_planner")
  {
    auto expansions=declare_parameter<int64_t>("max_expansions",200000);
    auto nodes=declare_parameter<int64_t>("max_search_nodes",500000);
    auto los=declare_parameter<int64_t>("max_los_voxels",100000);
    limits_.seconds=declare_parameter("max_planning_time",10.0);
    limits_.cost_weight=declare_parameter("cost_weight",1.0);
    if (expansions<=0 || expansions>UINT32_MAX || nodes<=0 || nodes>UINT32_MAX || los<=0 || !std::isfinite(limits_.seconds) ||
      limits_.seconds<=0 || !std::isfinite(limits_.cost_weight) || limits_.cost_weight<0) {
      throw std::invalid_argument("Planner limits must be positive; cost_weight finite and nonnegative");
    }
    limits_.expansions=expansions; limits_.nodes=nodes; limits_.los_voxels=los;
    if (declare_parameter("publish_plan",true)) {
      publisher_=create_publisher<nav_msgs::msg::Path>("plan",rclcpp::QoS(1).reliable().transient_local());
    }
    subscription_=create_subscription<uav_navigation_msgs::msg::VoxelCostmap>("costmap",
      rclcpp::QoS(1).reliable().transient_local(),
      [this](uav_navigation_msgs::msg::VoxelCostmap::ConstSharedPtr message) {
        try {
          auto next=std::make_shared<const astar_planner::Grid>(*message);
          std::lock_guard<std::mutex> lock(mutex_);
          if (grid_ && next->map_id==grid_->map_id && next->sequence<grid_->sequence) {return;}
          grid_=std::move(next);
        } catch (const std::exception & error) {
          std::lock_guard<std::mutex> lock(mutex_); grid_.reset();
          RCLCPP_ERROR(get_logger(),"Invalid costmap: %s",error.what());
        }
      });
    server_=rclcpp_action::create_server<Action>(this,"compute_path_3d",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>) {
        return busy_.exchange(true) ? rclcpp_action::GoalResponse::REJECT :
          rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<Handle>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<Handle> handle) {
        if (worker_.joinable()) {worker_.join();}
        worker_=std::thread([this,handle]() {execute(handle);});
      });
    RCLCPP_INFO(get_logger(),"Ready for ComputePath3D requests; waiting for /costmap");
  }
  ~Planner() override {stop_=true; if (worker_.joinable()) {worker_.join();}}
private:
  void execute(const std::shared_ptr<Handle> & handle)
  {
    auto output=std::make_shared<Action::Result>();
    try {
      std::shared_ptr<const astar_planner::Grid> grid;
      {std::lock_guard<std::mutex> lock(mutex_); grid=grid_;}
      auto goal=handle->get_goal();
      if (!grid) {output->status=Action::Result::NO_MAP; output->message="No valid costmap received";}
      else if (goal->start.header.frame_id != grid->frame) {
        output->status=Action::Result::INVALID_START; output->message="Start frame must match costmap frame";
      } else if (goal->goal.header.frame_id != grid->frame) {
        output->status=Action::Result::INVALID_GOAL; output->message="Goal frame must match costmap frame";
      } else {
        const auto & s=goal->start.pose.position;
        const auto & g=goal->goal.pose.position;
        auto result=astar_planner::search(*grid,{s.x,s.y,s.z},{g.x,g.y,g.z},limits_,
          [this,handle]() {return stop_ || !rclcpp::ok() || handle->is_canceling();},
          [handle](size_t expanded) {
            auto feedback=std::make_shared<Action::Feedback>();
            feedback->expanded_nodes=expanded; handle->publish_feedback(feedback);
          });
        output->status=result.status; output->message=result.message;
        output->planning_time=result.seconds; output->expanded_nodes=result.expanded;
        output->raw_point_count=result.raw_count; output->refined_point_count=result.path.size();
        output->path.header.frame_id=grid->frame; output->path.header.stamp=now();
        for (auto point : result.path) {
          geometry_msgs::msg::PoseStamped pose;
          pose.header=output->path.header; pose.pose.position.x=point.x;
          pose.pose.position.y=point.y; pose.pose.position.z=point.z;
          pose.pose.orientation.w=1; output->path.poses.push_back(pose);
        }
        RCLCPP_INFO(get_logger(),"Planning status %u, %.3f s, %u expanded, points %u -> %u (map %lu, sequence %lu)",
          output->status,output->planning_time,output->expanded_nodes,output->raw_point_count,
          output->refined_point_count,grid->map_id,grid->sequence);
      }
      if (handle->is_canceling()) {
        output->status=Action::Result::CANCELLED; output->message="Planning cancelled";
        output->path.poses.clear(); output->refined_point_count=0; handle->canceled(output);
      } else if (output->status==Action::Result::SUCCESS) {
        if (publisher_) {publisher_->publish(output->path);}
        handle->succeed(output);
      } else {handle->abort(output);}
    } catch (const std::exception & error) {
      output->status=Action::Result::INTERNAL_ERROR; output->message=error.what();
      output->path.poses.clear(); output->refined_point_count=0;
      try {if (handle->is_canceling()) {handle->canceled(output);} else {handle->abort(output);}} catch (...) {}
      RCLCPP_ERROR(get_logger(),"Planning failed: %s",error.what());
    }
    busy_=false;
  }
  astar_planner::Limits limits_;
  std::mutex mutex_;
  std::shared_ptr<const astar_planner::Grid> grid_;
  std::atomic<bool> busy_{false}, stop_{false};
  std::thread worker_;
  rclcpp::Subscription<uav_navigation_msgs::msg::VoxelCostmap>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
  rclcpp_action::Server<Action>::SharedPtr server_;
};

int main(int argc,char ** argv)
{
  rclcpp::init(argc,argv);
  int result=0;
  try {rclcpp::spin(std::make_shared<Planner>());}
  catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("astar_planner"),"%s",error.what()); result=1;
  }
  rclcpp::shutdown(); return result;
}
