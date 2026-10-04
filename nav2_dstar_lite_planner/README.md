# nav2-dstar-lite-planner

An **incremental D\* Lite** global planner plugin for [ROS 2 Nav2](https://docs.nav2.org/) (Humble), implementing `nav2_core::GlobalPlanner`.

D\* Lite (Koenig and Likhachev, 2002) plans backwards from the goal and keeps its search state between calls. When the robot moves or the costmap changes, it repairs only the part of the search that is affected instead of planning again from scratch, which suits robots that replan constantly in changing environments. Costmap cost is part of the traversal cost, so paths keep away from obstacles instead of hugging them.

Part of a hybrid navigation project: see [hybrid-dstar-lite-td3-navigation](https://github.com/mohamed-aly-abdelghafar/hybrid-dstar-lite-td3-navigation) for this planner combined with a TD3 reinforcement-learning local controller.

## How it works

The search runs on the 8-connected global costmap and lives in `DStarLiteCore`, a class with no ROS dependency (`include/nav2_dstar_lite_planner/dstar_lite_core.hpp`). For every `ComputePathToPose` request the plugin:

1. Reads the global costmap (holding the costmap lock) and compares it with the previous one.
2. Repairs the search. If the goal and map size are unchanged, the robot's movement is absorbed by advancing D\* Lite's key offset `km`, and every changed costmap cell triggers a local update of the vertices around it. A new goal or a resized map starts a new search.
3. Expands vertices until the start is optimal. Cells with cost at or above `lethal_cost` are obstacles (unknown space, cost 255, counts as an obstacle). Moving into a cell costs `1` (straight) or `sqrt(2)` (diagonal) plus `cost_factor * (cell_cost / 252) * neutral_cost`, and the heuristic is Euclidean distance.
4. Extracts the path by following the computed values from the start, orients each pose along the direction of travel, and ends the path with the requested goal pose (position and heading).

Each replan logs one line, for example `D* Lite repair: 0 expansions, 0 changed cells, 0.3 ms` after an initial `D* Lite new search: 393 expansions ...`.

If the robot starts inside the inflated zone next to an obstacle, its own cell stays traversable so the planner can plan a way out.

**Limitations**

- The path follows grid cells and is not smoothed. Use Nav2's smoother server if you need a smooth path.
- Each call still compares the whole costmap with the previous one, which costs a few hundred microseconds on a typical map, independent of how much changed.
- Requests for different goals in a row (for example `NavigateThroughPoses`) start a new search each time, because the search is rooted at the goal.
- With `heuristic_weight` above 1 the search is faster but the path is no longer guaranteed to be optimal.

## Build

Requires ROS 2 Humble with Nav2.

```bash
cd ~/ros2_ws/src
git clone https://github.com/mohamed-aly-abdelghafar/nav2-dstar-lite-planner.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select nav2_dstar_lite_planner
source install/setup.bash
```

## Usage

In your Nav2 parameters file:

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_dstar_lite_planner::DStarLitePlanner"
      heuristic_weight: 1.0
      cost_factor: 0.8
      neutral_cost: 50.0
      lethal_cost: 253
```

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `heuristic_weight` | `1.0` | Weight on the Euclidean heuristic. Values above 1 search faster but the path may no longer be optimal. |
| `cost_factor` | `0.8` | How strongly costmap cost raises the traversal cost. Higher values keep the path further from obstacles; `0` ignores the costmap cost. |
| `neutral_cost` | `50.0` | Scale of the cost penalty (the penalty at the highest non-lethal cost is `cost_factor * neutral_cost`). |
| `lethal_cost` | `253` | Costmap values at or above this are obstacles. |

## Testing

```bash
colcon test --packages-select nav2_dstar_lite_planner
colcon test-result --verbose
```

The unit tests (`test/test_dstar_lite_core.cpp`) cover the basics (free space, walled-in goal, start inside an obstacle, new goal) and check that repairing is cheaper than searching again. The main test runs 100 random scenarios of 40 steps each: random costs, random costmap changes, and a start that moves along the path. After every step the incrementally repaired cost must equal the cost from an independent Dijkstra search, and the extracted path must be valid and have exactly that cost. During development the same test was run with 3,000 scenarios (120,000 repairs).

The plugin was also run in Gazebo with a TurtleBot3 and Nav2, on a free-space goal and on a goal behind an obstacle. The plan reached the requested goal pose, went around the obstacle (4.45 m against a 4.22 m straight line) and kept at least 0.18 m from obstacles. In a 25-replan run the log showed one new search (393 expansions) followed by repairs that needed no expansions, since the map was static. The simulation smoke test is in [hybrid-dstar-lite-td3-navigation](https://github.com/mohamed-aly-abdelghafar/hybrid-dstar-lite-td3-navigation).

## License

Apache-2.0. See [LICENSE](LICENSE).

## Author

Mohamed Aly · [LinkedIn](https://www.linkedin.com/in/mohamed-aly-889b0320a/) · mohamedalyabdelghafar@gmail.com
