#include <chrono>
#include <iostream>
#include <memory>
#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "pathfinding/algorithms.hpp"
#include "pathfinding/utils.hpp"

namespace pathfinding_algorithms
{

using namespace pathfinding_utils;

std::vector<Node> AStar::FindPath(
  std::shared_ptr<const OccupancyGrid> map, Node start, Node goal,
  const std::string & summary_filename)
{
  if (map->IsOccupied(start.x, start.y)) {
    throw std::runtime_error("Start node is occupied");
  }
  if (map->IsOccupied(goal.x, goal.y)) {
    throw std::runtime_error("Goal node is occupied");
  }

  std::cout << "\nPathfinding A* algorithm has started. It may take some time..." << std::endl;
  const auto start_time = std::chrono::high_resolution_clock::now();

  auto scratch_map = std::make_unique<OccupancyGrid>(*map);
  int nodes_checked = 0;

  std::unordered_map<Node, Node, NodeHash> came_from;
  std::priority_queue<Node> frontier;

  start.g = 0;
  start.h = heuristic_->Calculate(start, goal);
  start.f = start.g + start.h;

  frontier.push(start);
  came_from[start] = start;

  while (!frontier.empty()) {
    Node current = frontier.top();
    frontier.pop();

    if (current == goal) {
      const auto end_time = std::chrono::high_resolution_clock::now();
      const auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

      std::cout << "\nPathfinding successful! Goal node has been reached." << std::endl;
      std::cout << "\t - Total execution time:\t\t" << duration.count() << "ms" << std::endl;
      std::cout << "\t - Total number of nodes inspected:\t" << nodes_checked << std::endl;

      const auto path = ReconstructPath(came_from, start, goal);
      scratch_map->DrawPath(path);
      scratch_map->SaveAsPGM(summary_filename);

      return path;
    }

    for (auto next : GetNeighbors(current, map)) {
      if (allow_near_optimal_solution_ && scratch_map->IsOccupied(next.x, next.y)) {
        // Skip previously checked nodes. This significantly reduces computation time, but it does
        // not guarantee an optimal solution. The solution will be near-optimal at worst.
        continue;
      }

      next.g = current.g + movement_cost_;
      next.h = heuristic_->Calculate(next, goal);
      next.f = next.g + next.h;

      if (came_from.find(next) == came_from.end() || next.g < came_from[next].g) {
        came_from[next] = current;
        frontier.push(next);
      }

      // Visualize the process and mark the 'next' node as checked.
      scratch_map->SetPixel(next.x, next.y, checked_node_gray_val_);
      nodes_checked++;
    }
  }

  if (frontier.empty()) {
    throw std::runtime_error("No path found from start to goal");
  }

  return std::vector<Node>{};
}

std::vector<Node> AStar::GetNeighbors(const Node & node, std::shared_ptr<const OccupancyGrid> map)
{
  std::vector<Node> neighbors;
  const std::vector<std::pair<int, int>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

  neighbors.reserve(4);
  for (const auto & dir : directions) {
    const int new_x = node.x + dir.first;
    const int new_y = node.y + dir.second;

    if (new_x >= 0 && new_x < map->GetWidth() && new_y >= 0 && new_y < map->GetHeight()) {
      if (!map->IsOccupied(new_x, new_y)) {
        neighbors.push_back(Node{new_x, new_y});
      }
    }
  }

  return neighbors;
}

std::vector<Node> AStar::ReconstructPath(
  const std::unordered_map<Node, Node, NodeHash> & came_from, const Node & start,
  const Node & goal) const
{
  std::vector<Node> total_path;
  Node current = goal;
  while (current != start) {
    total_path.push_back(current);
    current = came_from.at(current);
  }
  total_path.push_back(start);
  std::reverse(total_path.begin(), total_path.end());
  return total_path;
}

}  // namespace pathfinding_algorithms
