#ifndef NAV2_DSTAR_LITE_PLANNER__DSTAR_LITE_CORE_HPP_
#define NAV2_DSTAR_LITE_PLANNER__DSTAR_LITE_CORE_HPP_

#include <cstddef>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace nav2_dstar_lite_planner
{

/**
 * Incremental D* Lite (Koenig & Likhachev, 2002) on an 8-connected grid.
 *
 * The search runs backwards from the goal. Between calls to update() the search state is kept:
 * when the robot moves, the key offset km is advanced, and when cell costs change only the
 * affected vertices are repaired, so a replan usually expands far fewer cells than a new search.
 * The class has no ROS dependency and works on a plain array of costmap values (0-255).
 */
class DStarLiteCore
{
public:
  struct Params
  {
    double heuristic_weight = 1.0;  // weight on the Euclidean heuristic (>1 is faster, not optimal)
    double cost_factor = 0.8;       // strength of the costmap-cost penalty
    double neutral_cost = 50.0;     // scale of the costmap-cost penalty
    int lethal_cost = 253;          // costmap values at or above this are obstacles
  };

  struct Stats
  {
    bool full_reset = false;     // the search was restarted from scratch
    size_t changed_cells = 0;    // costmap cells that changed since the previous update
    size_t expansions = 0;       // vertices expanded by the last computeShortestPath()
  };

  using Cell = std::pair<int, int>;

  void setParams(const Params & params);

  /// Discard all search state; the next update() starts a new search.
  void invalidate();

  /**
   * Feed the current costmap and the robot/goal cells. Restarts the search when the map size
   * or the goal changes, otherwise repairs the existing search. Returns false for invalid input.
   */
  bool update(
    const unsigned char * costmap, unsigned int width, unsigned int height,
    int start_x, int start_y, int goal_x, int goal_y);

  /// Expand vertices until the start is optimal. Returns false if the iteration limit was hit.
  bool computeShortestPath();

  bool goalBlocked() const;
  bool pathExists() const;

  /// Cost of the best path from the start to the goal (infinity if there is none).
  double startCost() const;

  /// Follow the computed values from the start to the goal. Returns false if no path exists.
  bool extractPath(std::vector<Cell> & cells) const;

  /// Total traversal cost of a cell path under the current costmap.
  double pathCost(const std::vector<Cell> & cells) const;

  const Stats & stats() const {return stats_;}

private:
  using Key = std::pair<double, double>;

  struct Node
  {
    double g = std::numeric_limits<double>::infinity();
    double rhs = std::numeric_limits<double>::infinity();
    double cost = 0.0;  // costmap cost, infinity for obstacles
    double key1 = std::numeric_limits<double>::infinity();
    double key2 = std::numeric_limits<double>::infinity();
    bool in_queue = false;
  };

  struct QueueEntry
  {
    Key key;
    int index;
  };

  struct CompareEntry
  {
    bool operator()(const QueueEntry & a, const QueueEntry & b) const {return a.key > b.key;}
  };

  size_t index(int x, int y) const {return static_cast<size_t>(y) * width_ + x;}
  bool inBounds(int x, int y) const
  {
    return x >= 0 && y >= 0 && x < static_cast<int>(width_) && y < static_cast<int>(height_);
  }
  double costFromRaw(unsigned char raw) const;
  double heuristic(int x1, int y1, int x2, int y2) const;
  double traversalCost(int from_x, int from_y, int to_x, int to_y) const;
  bool isBlocked(int x, int y) const;
  double computeRhs(int x, int y) const;
  Key calcKey(int x, int y) const;
  Key topKey();
  void updateQueue(int x, int y);
  void updateVertex(int x, int y);
  void reset(int start_x, int start_y, int goal_x, int goal_y);

  Params params_;
  Stats stats_;
  bool initialized_ = false;
  unsigned int width_ = 0;
  unsigned int height_ = 0;
  int start_x_ = 0, start_y_ = 0;
  int goal_x_ = 0, goal_y_ = 0;
  int last_x_ = 0, last_y_ = 0;  // start used for the previous search (s_last)
  double km_ = 0.0;
  std::vector<Node> nodes_;
  std::vector<unsigned char> raw_;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, CompareEntry> queue_;
};

}  // namespace nav2_dstar_lite_planner

#endif  // NAV2_DSTAR_LITE_PLANNER__DSTAR_LITE_CORE_HPP_
