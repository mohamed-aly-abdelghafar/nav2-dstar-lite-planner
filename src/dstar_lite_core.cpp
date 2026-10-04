#include "nav2_dstar_lite_planner/dstar_lite_core.hpp"

#include <algorithm>
#include <cmath>

namespace nav2_dstar_lite_planner
{

namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kMaxCostmapCost = 252.0;
constexpr double kKeyTolerance = 1e-9;

// Lexicographic key comparison that ignores floating-point noise. Two keys that are equal in
// exact arithmetic can differ in the last bits (for example 35.649 + 1 against 36.649), and a
// tie that is broken the wrong way would stop the search one vertex too early.
bool keyLess(const std::pair<double, double> & a, const std::pair<double, double> & b)
{
  if (a.first < b.first - kKeyTolerance) {
    return true;
  }
  if (a.first > b.first + kKeyTolerance) {
    return false;
  }
  return a.second < b.second - kKeyTolerance;
}
}  // namespace

void DStarLiteCore::setParams(const Params & params)
{
  params_ = params;
  invalidate();
}

void DStarLiteCore::invalidate()
{
  initialized_ = false;
  width_ = 0;
  height_ = 0;
  nodes_.clear();
  raw_.clear();
  queue_ = std::priority_queue<QueueEntry, std::vector<QueueEntry>, CompareEntry>();
}

double DStarLiteCore::costFromRaw(unsigned char raw) const
{
  if (raw >= params_.lethal_cost) {
    return kInf;
  }
  return static_cast<double>(raw);
}

double DStarLiteCore::heuristic(int x1, int y1, int x2, int y2) const
{
  return params_.heuristic_weight * std::hypot(x1 - x2, y1 - y2);
}

// Cost of moving from one cell into an adjacent one: the step length plus a penalty that grows
// with the costmap cost of the cell being entered.
double DStarLiteCore::traversalCost(int from_x, int from_y, int to_x, int to_y) const
{
  const double cell_cost = nodes_[index(to_x, to_y)].cost;
  if (std::isinf(cell_cost)) {
    return kInf;
  }
  const double base = (from_x == to_x || from_y == to_y) ? 1.0 : std::sqrt(2.0);
  const double penalty = cell_cost > 0.0 ?
    params_.cost_factor * (cell_cost / kMaxCostmapCost) * params_.neutral_cost : 0.0;
  return base + penalty;
}

// An obstacle cell cannot be left, except the robot's own cell: a robot that starts inside the
// inflated zone must still be able to plan a way out.
bool DStarLiteCore::isBlocked(int x, int y) const
{
  return std::isinf(nodes_[index(x, y)].cost) && !(x == start_x_ && y == start_y_);
}

double DStarLiteCore::computeRhs(int x, int y) const
{
  if (x == goal_x_ && y == goal_y_) {
    return 0.0;
  }
  if (isBlocked(x, y)) {
    return kInf;
  }
  double best = kInf;
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      if (dx == 0 && dy == 0) {
        continue;
      }
      const int nx = x + dx;
      const int ny = y + dy;
      if (!inBounds(nx, ny)) {
        continue;
      }
      const double cost = traversalCost(x, y, nx, ny);
      if (std::isinf(cost)) {
        continue;
      }
      best = std::min(best, cost + nodes_[index(nx, ny)].g);
    }
  }
  return best;
}

DStarLiteCore::Key DStarLiteCore::calcKey(int x, int y) const
{
  const Node & node = nodes_[index(x, y)];
  const double m = std::min(node.g, node.rhs);
  return {m + heuristic(start_x_, start_y_, x, y) + km_, m};
}

// Smallest key in the queue, discarding entries that are stale (the vertex was updated or
// removed after the entry was pushed).
DStarLiteCore::Key DStarLiteCore::topKey()
{
  while (!queue_.empty()) {
    const QueueEntry & top = queue_.top();
    const Node & node = nodes_[top.index];
    if (node.in_queue && node.key1 == top.key.first && node.key2 == top.key.second) {
      return top.key;
    }
    queue_.pop();
  }
  return {kInf, kInf};
}

// Put the vertex in the queue if it is inconsistent (g != rhs), take it out otherwise.
void DStarLiteCore::updateQueue(int x, int y)
{
  Node & node = nodes_[index(x, y)];
  if (node.g != node.rhs) {
    // Always push: an earlier entry with the same key may already have been discarded as stale,
    // and duplicates are harmless because only the first one popped is still marked queued.
    const Key key = calcKey(x, y);
    node.key1 = key.first;
    node.key2 = key.second;
    node.in_queue = true;
    queue_.push({key, static_cast<int>(index(x, y))});
  } else {
    node.in_queue = false;
  }
}

// UpdateVertex() of the D* Lite paper: recompute rhs from the successors, then fix the queue.
void DStarLiteCore::updateVertex(int x, int y)
{
  nodes_[index(x, y)].rhs = computeRhs(x, y);
  updateQueue(x, y);
}

void DStarLiteCore::reset(int start_x, int start_y, int goal_x, int goal_y)
{
  const size_t n = static_cast<size_t>(width_) * height_;
  nodes_.assign(n, Node());
  for (size_t i = 0; i < n; ++i) {
    nodes_[i].cost = costFromRaw(raw_[i]);
  }
  queue_ = std::priority_queue<QueueEntry, std::vector<QueueEntry>, CompareEntry>();
  start_x_ = last_x_ = start_x;
  start_y_ = last_y_ = start_y;
  goal_x_ = goal_x;
  goal_y_ = goal_y;
  km_ = 0.0;
  nodes_[index(goal_x_, goal_y_)].rhs = 0.0;
  updateQueue(goal_x_, goal_y_);
}

bool DStarLiteCore::update(
  const unsigned char * costmap, unsigned int width, unsigned int height,
  int start_x, int start_y, int goal_x, int goal_y)
{
  stats_ = Stats();
  if (costmap == nullptr || width == 0 || height == 0) {
    return false;
  }
  const int w = static_cast<int>(width);
  const int h = static_cast<int>(height);
  if (start_x < 0 || start_y < 0 || start_x >= w || start_y >= h ||
    goal_x < 0 || goal_y < 0 || goal_x >= w || goal_y >= h)
  {
    return false;
  }
  const size_t n = static_cast<size_t>(width) * height;

  if (!initialized_ || width != width_ || height != height_ ||
    goal_x != goal_x_ || goal_y != goal_y_)
  {
    width_ = width;
    height_ = height;
    raw_.assign(costmap, costmap + n);
    reset(start_x, start_y, goal_x, goal_y);
    initialized_ = true;
    stats_.full_reset = true;
    return true;
  }

  // The robot moved: advance the key offset so keys already in the queue stay comparable.
  if (start_x != start_x_ || start_y != start_y_) {
    const int old_x = start_x_;
    const int old_y = start_y_;
    km_ += heuristic(last_x_, last_y_, start_x, start_y);
    last_x_ = start_x;
    last_y_ = start_y;
    start_x_ = start_x;
    start_y_ = start_y;
    // Only these two cells change status (an obstacle start cell is traversable, others not).
    updateVertex(old_x, old_y);
    updateVertex(start_x_, start_y_);
  }

  // Costmap changes: a cell's cost affects the edges into it, i.e. the rhs of its neighbours.
  std::vector<int> changed;
  for (size_t i = 0; i < n; ++i) {
    if (costmap[i] != raw_[i]) {
      raw_[i] = costmap[i];
      nodes_[i].cost = costFromRaw(raw_[i]);
      changed.push_back(static_cast<int>(i));
    }
  }
  stats_.changed_cells = changed.size();

  std::vector<int> touched;
  touched.reserve(changed.size() * 9);
  for (const int i : changed) {
    const int cx = i % w;
    const int cy = i / w;
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (inBounds(cx + dx, cy + dy)) {
          touched.push_back(static_cast<int>(index(cx + dx, cy + dy)));
        }
      }
    }
  }
  std::sort(touched.begin(), touched.end());
  touched.erase(std::unique(touched.begin(), touched.end()), touched.end());
  for (const int i : touched) {
    updateVertex(i % w, i / w);
  }
  return true;
}

bool DStarLiteCore::computeShortestPath()
{
  if (!initialized_) {
    return false;
  }
  stats_.expansions = 0;
  const size_t max_expansions = static_cast<size_t>(width_) * height_ * 4;

  while (true) {
    const Key top = topKey();
    const Node & start = nodes_[index(start_x_, start_y_)];
    if (!keyLess(top, calcKey(start_x_, start_y_)) && start.rhs == start.g) {
      break;
    }
    if (queue_.empty()) {
      break;
    }
    if (stats_.expansions >= max_expansions) {
      return false;
    }

    const QueueEntry entry = queue_.top();
    queue_.pop();
    const int ux = entry.index % static_cast<int>(width_);
    const int uy = entry.index / static_cast<int>(width_);
    Node & u = nodes_[entry.index];
    u.in_queue = false;

    const Key new_key = calcKey(ux, uy);
    if (entry.key < new_key) {
      // The key grew since it was queued (the robot moved): requeue with the current key.
      u.key1 = new_key.first;
      u.key2 = new_key.second;
      u.in_queue = true;
      queue_.push({new_key, entry.index});
      continue;
    }
    ++stats_.expansions;

    if (u.g > u.rhs) {
      // Overconsistent: settle it and offer the better value to its predecessors.
      u.g = u.rhs;
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) {
            continue;
          }
          const int px = ux + dx;
          const int py = uy + dy;
          if (!inBounds(px, py) || (px == goal_x_ && py == goal_y_) || isBlocked(px, py)) {
            continue;
          }
          const double cost = traversalCost(px, py, ux, uy);
          if (std::isinf(cost)) {
            continue;
          }
          Node & p = nodes_[index(px, py)];
          if (u.g + cost < p.rhs) {
            p.rhs = u.g + cost;
            updateQueue(px, py);
          }
        }
      }
    } else {
      // Underconsistent: its value can no longer be trusted, so recompute it and every
      // vertex that may have been relying on it.
      u.g = kInf;
      updateVertex(ux, uy);
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if ((dx != 0 || dy != 0) && inBounds(ux + dx, uy + dy)) {
            updateVertex(ux + dx, uy + dy);
          }
        }
      }
    }
  }
  return true;
}

bool DStarLiteCore::goalBlocked() const
{
  return initialized_ && std::isinf(nodes_[index(goal_x_, goal_y_)].cost);
}

bool DStarLiteCore::pathExists() const
{
  return initialized_ && std::isfinite(nodes_[index(start_x_, start_y_)].g);
}

double DStarLiteCore::startCost() const
{
  return initialized_ ? nodes_[index(start_x_, start_y_)].g : kInf;
}

bool DStarLiteCore::extractPath(std::vector<Cell> & cells) const
{
  cells.clear();
  if (!pathExists()) {
    return false;
  }
  int x = start_x_;
  int y = start_y_;
  cells.emplace_back(x, y);
  const size_t max_length = static_cast<size_t>(width_) * height_;

  while ((x != goal_x_ || y != goal_y_) && cells.size() <= max_length) {
    double best = kInf;
    int best_x = -1;
    int best_y = -1;
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        const int nx = x + dx;
        const int ny = y + dy;
        if (!inBounds(nx, ny)) {
          continue;
        }
        const double g = nodes_[index(nx, ny)].g;
        const double cost = traversalCost(x, y, nx, ny);
        if (std::isinf(g) || std::isinf(cost)) {
          continue;
        }
        if (g + cost < best) {
          best = g + cost;
          best_x = nx;
          best_y = ny;
        }
      }
    }
    if (best_x < 0) {
      cells.clear();
      return false;
    }
    x = best_x;
    y = best_y;
    cells.emplace_back(x, y);
  }
  if (x != goal_x_ || y != goal_y_) {
    cells.clear();
    return false;
  }
  return true;
}

double DStarLiteCore::pathCost(const std::vector<Cell> & cells) const
{
  double total = 0.0;
  for (size_t i = 0; i + 1 < cells.size(); ++i) {
    total +=
      traversalCost(cells[i].first, cells[i].second, cells[i + 1].first, cells[i + 1].second);
  }
  return total;
}

}  // namespace nav2_dstar_lite_planner
