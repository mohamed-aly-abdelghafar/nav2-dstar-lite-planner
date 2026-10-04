#ifndef NAV2_DSTAR_LITE_PLANNER__DSTAR_LITE_PLANNER_HPP_
#define NAV2_DSTAR_LITE_PLANNER__DSTAR_LITE_PLANNER_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

#include "nav2_dstar_lite_planner/dstar_lite_core.hpp"

namespace nav2_dstar_lite_planner
{

/**
 * Nav2 global planner plugin running incremental D* Lite on the global costmap. The search
 * state is kept between calls, so replanning toward the same goal only repairs the parts of
 * the search affected by the robot's movement and by costmap changes.
 */
class DStarLitePlanner : public nav2_core::GlobalPlanner
{
public:
  DStarLitePlanner() = default;
  ~DStarLitePlanner() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  nav_msgs::msg::Path buildPath(
    const std::vector<DStarLiteCore::Cell> & cells, const geometry_msgs::msg::PoseStamped & goal);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string name_;
  nav2_costmap_2d::Costmap2D * costmap_ = nullptr;
  std::string global_frame_;

  DStarLiteCore core_;
  std::mutex plan_mutex_;

  // Costmap geometry the search state belongs to; the search restarts if it changes.
  double resolution_ = 0.0;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
};

}  // namespace nav2_dstar_lite_planner

#endif  // NAV2_DSTAR_LITE_PLANNER__DSTAR_LITE_PLANNER_HPP_
