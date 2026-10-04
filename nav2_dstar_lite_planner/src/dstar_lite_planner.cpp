#include "nav2_dstar_lite_planner/dstar_lite_planner.hpp"

#include <chrono>
#include <cmath>
#include <vector>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace nav2_dstar_lite_planner
{

void DStarLitePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  DStarLiteCore::Params params;
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".heuristic_weight", rclcpp::ParameterValue(params.heuristic_weight));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".cost_factor", rclcpp::ParameterValue(params.cost_factor));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".neutral_cost", rclcpp::ParameterValue(params.neutral_cost));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".lethal_cost", rclcpp::ParameterValue(params.lethal_cost));
  node_->get_parameter(name_ + ".heuristic_weight", params.heuristic_weight);
  node_->get_parameter(name_ + ".cost_factor", params.cost_factor);
  node_->get_parameter(name_ + ".neutral_cost", params.neutral_cost);
  node_->get_parameter(name_ + ".lethal_cost", params.lethal_cost);
  core_.setParams(params);

  RCLCPP_INFO(
    node_->get_logger(),
    "Configured incremental D* Lite planner '%s': heuristic_weight=%.2f, cost_factor=%.2f, "
    "neutral_cost=%.1f, lethal_cost=%d",
    name_.c_str(), params.heuristic_weight, params.cost_factor, params.neutral_cost,
    params.lethal_cost);
}

void DStarLitePlanner::activate()
{
  RCLCPP_INFO(node_->get_logger(), "Activating D* Lite planner");
}

void DStarLitePlanner::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "Deactivating D* Lite planner");
}

void DStarLitePlanner::cleanup()
{
  RCLCPP_INFO(node_->get_logger(), "Cleaning up D* Lite planner");
  std::lock_guard<std::mutex> lock(plan_mutex_);
  core_.invalidate();
}

nav_msgs::msg::Path DStarLitePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  // Planner server actions can call this concurrently; the search state is shared.
  std::lock_guard<std::mutex> plan_lock(plan_mutex_);
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(*(costmap_->getMutex()));

  unsigned int start_mx, start_my, goal_mx, goal_my;
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_mx, start_my)) {
    RCLCPP_ERROR(node_->get_logger(), "Start pose is out of bounds");
    return nav_msgs::msg::Path();
  }
  if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_mx, goal_my)) {
    RCLCPP_ERROR(node_->get_logger(), "Goal pose is out of bounds");
    return nav_msgs::msg::Path();
  }

  // A moved or resized costmap (for example a rolling window) invalidates the search state.
  if (costmap_->getResolution() != resolution_ || costmap_->getOriginX() != origin_x_ ||
    costmap_->getOriginY() != origin_y_)
  {
    core_.invalidate();
    resolution_ = costmap_->getResolution();
    origin_x_ = costmap_->getOriginX();
    origin_y_ = costmap_->getOriginY();
  }

  const auto begin = std::chrono::steady_clock::now();
  if (!core_.update(
      costmap_->getCharMap(), costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY(),
      static_cast<int>(start_mx), static_cast<int>(start_my),
      static_cast<int>(goal_mx), static_cast<int>(goal_my)))
  {
    RCLCPP_ERROR(node_->get_logger(), "Invalid planning request");
    return nav_msgs::msg::Path();
  }
  if (core_.goalBlocked()) {
    RCLCPP_ERROR(node_->get_logger(), "Goal is in a lethal obstacle");
    return nav_msgs::msg::Path();
  }
  if (!core_.computeShortestPath()) {
    RCLCPP_WARN(node_->get_logger(), "D* Lite stopped at its expansion limit");
  }
  const std::chrono::duration<double, std::milli> elapsed =
    std::chrono::steady_clock::now() - begin;

  const auto & stats = core_.stats();
  RCLCPP_INFO(
    node_->get_logger(),
    "D* Lite %s: %zu expansions, %zu changed cells, %.2f ms",
    stats.full_reset ? "new search" : "repair", stats.expansions, stats.changed_cells,
    elapsed.count());

  std::vector<DStarLiteCore::Cell> cells;
  if (!core_.extractPath(cells)) {
    RCLCPP_WARN(node_->get_logger(), "No path found from the start to the goal");
    return nav_msgs::msg::Path();
  }
  return buildPath(cells, goal);
}

nav_msgs::msg::Path DStarLitePlanner::buildPath(
  const std::vector<DStarLiteCore::Cell> & cells, const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.stamp = node_->now();
  path.header.frame_id = global_frame_;
  path.poses.reserve(cells.size());

  for (const auto & cell : cells) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    costmap_->mapToWorld(cell.first, cell.second, pose.pose.position.x, pose.pose.position.y);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  // The last pose is the requested goal itself. Nav2's goal checker compares the robot against
  // this pose, so it must carry the requested position and heading.
  geometry_msgs::msg::PoseStamped final_pose = goal;
  final_pose.header = path.header;
  path.poses.back() = final_pose;

  // Orient every intermediate pose along the direction of travel.
  for (size_t i = 0; i + 1 < path.poses.size(); ++i) {
    const auto & a = path.poses[i].pose.position;
    const auto & b = path.poses[i + 1].pose.position;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, std::atan2(b.y - a.y, b.x - a.x));
    path.poses[i].pose.orientation.x = q.x();
    path.poses[i].pose.orientation.y = q.y();
    path.poses[i].pose.orientation.z = q.z();
    path.poses[i].pose.orientation.w = q.w();
  }
  return path;
}

}  // namespace nav2_dstar_lite_planner

PLUGINLIB_EXPORT_CLASS(nav2_dstar_lite_planner::DStarLitePlanner, nav2_core::GlobalPlanner)
