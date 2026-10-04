#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <random>
#include <vector>

#include "nav2_dstar_lite_planner/dstar_lite_core.hpp"

using nav2_dstar_lite_planner::DStarLiteCore;

namespace
{

constexpr unsigned char kObstacle = 254;

struct Grid
{
  unsigned int w, h;
  std::vector<unsigned char> data;
  Grid(unsigned int width, unsigned int height)
  : w(width), h(height), data(static_cast<size_t>(width) * height, 0) {}
  unsigned char & at(int x, int y) {return data[static_cast<size_t>(y) * w + x];}
};

unsigned char randomCost(std::mt19937 & rng)
{
  const double r = std::uniform_real_distribution<double>(0.0, 1.0)(rng);
  if (r < 0.15) {
    return kObstacle;
  }
  if (r < 0.35) {
    return static_cast<unsigned char>(std::uniform_int_distribution<int>(1, 252)(rng));
  }
  return 0;
}

// Runs a from-scratch search on the same inputs, for comparison.
double freshCost(const Grid & g, int sx, int sy, int gx, int gy)
{
  DStarLiteCore fresh;
  fresh.update(g.data.data(), g.w, g.h, sx, sy, gx, gy);
  fresh.computeShortestPath();
  return fresh.startCost();
}

// Independent reference: plain Dijkstra with the same edge costs (obstacles cannot be entered,
// but the start cell may be left even if it is an obstacle).
double dijkstraCost(const Grid & g, int sx, int sy, int gx, int gy)
{
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> dist(g.data.size(), inf);
  using Item = std::pair<double, int>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> pq;
  dist[static_cast<size_t>(sy) * g.w + sx] = 0.0;
  pq.push({0.0, sy * static_cast<int>(g.w) + sx});
  while (!pq.empty()) {
    const auto [d, idx] = pq.top();
    pq.pop();
    if (d > dist[idx]) {
      continue;
    }
    const int x = idx % static_cast<int>(g.w);
    const int y = idx / static_cast<int>(g.w);
    if (x == gx && y == gy) {
      return d;
    }
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        const int nx = x + dx;
        const int ny = y + dy;
        if ((dx == 0 && dy == 0) || nx < 0 || ny < 0 || nx >= static_cast<int>(g.w) ||
          ny >= static_cast<int>(g.h))
        {
          continue;
        }
        const unsigned char raw = g.data[static_cast<size_t>(ny) * g.w + nx];
        if (raw >= 253) {
          continue;
        }
        const double base = (dx == 0 || dy == 0) ? 1.0 : std::sqrt(2.0);
        const double penalty = raw > 0 ? 0.8 * (raw / 252.0) * 50.0 : 0.0;
        const size_t nidx = static_cast<size_t>(ny) * g.w + nx;
        if (d + base + penalty < dist[nidx]) {
          dist[nidx] = d + base + penalty;
          pq.push({dist[nidx], static_cast<int>(nidx)});
        }
      }
    }
  }
  return inf;
}

}  // namespace

TEST(DStarLiteCore, StraightLineInFreeSpace)
{
  Grid grid(20, 20);
  DStarLiteCore core;
  ASSERT_TRUE(core.update(grid.data.data(), grid.w, grid.h, 2, 2, 12, 2));
  ASSERT_TRUE(core.computeShortestPath());
  ASSERT_TRUE(core.pathExists());
  EXPECT_NEAR(core.startCost(), 10.0, 1e-9);

  std::vector<DStarLiteCore::Cell> path;
  ASSERT_TRUE(core.extractPath(path));
  EXPECT_EQ(path.size(), 11u);
  EXPECT_EQ(path.front(), DStarLiteCore::Cell(2, 2));
  EXPECT_EQ(path.back(), DStarLiteCore::Cell(12, 2));
}

TEST(DStarLiteCore, NoPathWhenGoalIsWalledIn)
{
  Grid grid(15, 15);
  for (int d = -1; d <= 1; ++d) {
    for (int e = -1; e <= 1; ++e) {
      if (d != 0 || e != 0) {
        grid.at(10 + d, 10 + e) = kObstacle;
      }
    }
  }
  DStarLiteCore core;
  ASSERT_TRUE(core.update(grid.data.data(), grid.w, grid.h, 1, 1, 10, 10));
  core.computeShortestPath();
  EXPECT_FALSE(core.pathExists());
  std::vector<DStarLiteCore::Cell> path;
  EXPECT_FALSE(core.extractPath(path));
}

TEST(DStarLiteCore, GoalOnObstacleIsReported)
{
  Grid grid(10, 10);
  grid.at(5, 5) = kObstacle;
  DStarLiteCore core;
  ASSERT_TRUE(core.update(grid.data.data(), grid.w, grid.h, 1, 1, 5, 5));
  EXPECT_TRUE(core.goalBlocked());
}

TEST(DStarLiteCore, StartInsideObstacleCanPlanOut)
{
  Grid grid(10, 10);
  grid.at(2, 2) = kObstacle;
  DStarLiteCore core;
  ASSERT_TRUE(core.update(grid.data.data(), grid.w, grid.h, 2, 2, 8, 8));
  ASSERT_TRUE(core.computeShortestPath());
  EXPECT_TRUE(core.pathExists());
}

TEST(DStarLiteCore, RejectsOutOfRangeInput)
{
  Grid grid(10, 10);
  DStarLiteCore core;
  EXPECT_FALSE(core.update(grid.data.data(), grid.w, grid.h, -1, 0, 5, 5));
  EXPECT_FALSE(core.update(grid.data.data(), grid.w, grid.h, 0, 0, 10, 5));
  EXPECT_FALSE(core.update(nullptr, 10, 10, 0, 0, 5, 5));
}

TEST(DStarLiteCore, UnchangedInputsDoNoWork)
{
  Grid grid(60, 60);
  DStarLiteCore core;
  core.update(grid.data.data(), grid.w, grid.h, 3, 3, 55, 55);
  core.computeShortestPath();
  const double cost = core.startCost();

  core.update(grid.data.data(), grid.w, grid.h, 3, 3, 55, 55);
  EXPECT_FALSE(core.stats().full_reset);
  EXPECT_EQ(core.stats().changed_cells, 0u);
  core.computeShortestPath();
  EXPECT_EQ(core.stats().expansions, 0u);
  EXPECT_DOUBLE_EQ(core.startCost(), cost);
}

TEST(DStarLiteCore, NewGoalRestartsTheSearch)
{
  Grid grid(30, 30);
  DStarLiteCore core;
  core.update(grid.data.data(), grid.w, grid.h, 1, 1, 20, 20);
  core.computeShortestPath();
  core.update(grid.data.data(), grid.w, grid.h, 1, 1, 25, 5);
  EXPECT_TRUE(core.stats().full_reset);
  core.computeShortestPath();
  EXPECT_NEAR(core.startCost(), freshCost(grid, 1, 1, 25, 5), 1e-9);
}

TEST(DStarLiteCore, RepairingAfterMovingIsCheaperThanSearchingAgain)
{
  Grid grid(200, 200);
  DStarLiteCore core;
  core.update(grid.data.data(), grid.w, grid.h, 5, 100, 190, 100);
  core.computeShortestPath();
  const size_t first = core.stats().expansions;

  // The robot follows the path a few cells; nothing else changes.
  std::vector<DStarLiteCore::Cell> path;
  ASSERT_TRUE(core.extractPath(path));
  core.update(grid.data.data(), grid.w, grid.h, path[4].first, path[4].second, 190, 100);
  EXPECT_FALSE(core.stats().full_reset);
  core.computeShortestPath();
  EXPECT_LT(core.stats().expansions, first / 10);
  EXPECT_NEAR(core.startCost(), freshCost(grid, path[4].first, path[4].second, 190, 100), 1e-9);
}

TEST(DStarLiteCore, NewObstacleOnThePathForcesADetour)
{
  Grid grid(40, 40);
  DStarLiteCore core;
  core.update(grid.data.data(), grid.w, grid.h, 2, 20, 37, 20);
  core.computeShortestPath();
  EXPECT_NEAR(core.startCost(), 35.0, 1e-9);

  for (int y = 10; y <= 30; ++y) {
    grid.at(20, y) = kObstacle;
  }
  core.update(grid.data.data(), grid.w, grid.h, 2, 20, 37, 20);
  EXPECT_EQ(core.stats().changed_cells, 21u);
  core.computeShortestPath();
  EXPECT_GT(core.startCost(), 35.0);
  EXPECT_NEAR(core.startCost(), freshCost(grid, 2, 20, 37, 20), 1e-9);

  std::vector<DStarLiteCore::Cell> path;
  ASSERT_TRUE(core.extractPath(path));
  for (const auto & cell : path) {
    EXPECT_NE(grid.at(cell.first, cell.second), kObstacle);
  }
}

TEST(DStarLiteCore, RemovingAnObstacleShortensThePath)
{
  Grid grid(40, 40);
  for (int y = 0; y < 40; ++y) {
    grid.at(20, y) = kObstacle;
  }
  grid.at(20, 35) = 0;  // a single gap far from the straight line
  DStarLiteCore core;
  core.update(grid.data.data(), grid.w, grid.h, 2, 5, 37, 5);
  core.computeShortestPath();
  const double detour = core.startCost();

  grid.at(20, 5) = 0;
  core.update(grid.data.data(), grid.w, grid.h, 2, 5, 37, 5);
  core.computeShortestPath();
  EXPECT_LT(core.startCost(), detour);
  EXPECT_NEAR(core.startCost(), freshCost(grid, 2, 5, 37, 5), 1e-9);
}

// The central correctness property: however the costmap changes and the robot moves, the
// incrementally repaired search gives exactly the same optimal cost as a new search.
TEST(DStarLiteCore, IncrementalAlwaysMatchesFromScratch)
{
  for (unsigned int seed = 1; seed <= 100; ++seed) {
    std::mt19937 rng(seed);
    Grid grid(45, 45);
    for (auto & cell : grid.data) {
      cell = randomCost(rng);
    }
    auto random_free = [&]() {
        while (true) {
          const int x = std::uniform_int_distribution<int>(0, 44)(rng);
          const int y = std::uniform_int_distribution<int>(0, 44)(rng);
          if (grid.at(x, y) < kObstacle) {
            return DStarLiteCore::Cell(x, y);
          }
        }
      };

    const auto goal = random_free();
    auto start = random_free();
    DStarLiteCore core;

    for (int step = 0; step < 40; ++step) {
      const int changes = std::uniform_int_distribution<int>(0, 25)(rng);
      for (int i = 0; i < changes; ++i) {
        const int x = std::uniform_int_distribution<int>(0, 44)(rng);
        const int y = std::uniform_int_distribution<int>(0, 44)(rng);
        if (x == goal.first && y == goal.second) {
          continue;
        }
        grid.at(x, y) = randomCost(rng);
      }
      if (step > 0 && core.pathExists()) {
        // move along the current path when there is one, otherwise jump nearby
        std::vector<DStarLiteCore::Cell> path;
        core.extractPath(path);
        const size_t advance = std::min<size_t>(
          path.size() - 1, std::uniform_int_distribution<size_t>(1, 3)(rng));
        start = path[advance];
      } else if (step > 0) {
        start = random_free();
      }

      ASSERT_TRUE(
        core.update(
          grid.data.data(), grid.w, grid.h, start.first, start.second, goal.first, goal.second));
      ASSERT_TRUE(core.computeShortestPath());
      const double expected =
        dijkstraCost(grid, start.first, start.second, goal.first, goal.second);
      const double scratch = freshCost(grid, start.first, start.second, goal.first, goal.second);
      ASSERT_EQ(
        std::isfinite(scratch),
        std::isfinite(expected)) << "scratch, seed " << seed << " step " << step;
      if (std::isfinite(expected)) {
        ASSERT_NEAR(scratch, expected, 1e-9) << "scratch search, seed " << seed << " step " << step;
      }

      ASSERT_EQ(core.pathExists(), std::isfinite(expected)) << "seed " << seed << " step " << step;
      if (!std::isfinite(expected)) {
        continue;
      }
      ASSERT_NEAR(core.startCost(), expected, 1e-9) << "seed " << seed << " step " << step;

      std::vector<DStarLiteCore::Cell> path;
      ASSERT_TRUE(core.extractPath(path)) << "seed " << seed << " step " << step;
      ASSERT_EQ(path.front(), start);
      ASSERT_EQ(path.back(), goal);
      ASSERT_NEAR(core.pathCost(path), expected, 1e-6) << "seed " << seed << " step " << step;
      for (size_t i = 1; i < path.size(); ++i) {
        ASSERT_LE(std::abs(path[i].first - path[i - 1].first), 1);
        ASSERT_LE(std::abs(path[i].second - path[i - 1].second), 1);
        ASSERT_LT(grid.at(path[i].first, path[i].second), kObstacle);
      }
    }
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
