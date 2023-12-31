#include <chrono>
#include <iostream>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include "pathfinding/algorithms.hpp"
#include "pathfinding/utils.hpp"

namespace pathfinding_algorithms
{

using namespace pathfinding_utils;

std::vector<Node> AStar::FindPath(Node & start, Node & goal)
{
  if (map_->IsOccupied(start.x, start.y)) {
    throw std::runtime_error("Start node is occupied");
  }
  if (map_->IsOccupied(goal.x, goal.y)) {
    throw std::runtime_error("Goal node is occupied");
  }

  std::cout << "\nPathfinding A* algorithm has started. It may take some time..." << std::endl;
  auto start_time = std::chrono::high_resolution_clock::now();

  std::unordered_map<Node, Node, NodeHash> cameFrom;
  std::priority_queue<Node> frontier;
  scratch_map_ = std::make_unique<OccupancyGrid>(*map_);
  int nodes_checked = 0;

  start.g = 0;
  start.h = heuristic_->operator()(start, goal);
  start.f = start.g + start.h;

  frontier.push(start);
  cameFrom[start] = start;

  while (!frontier.empty()) {
    Node current = frontier.top();
    frontier.pop();

    if (current == goal) {
      auto end_time = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

      std::cout << "\nPathfinding successful! Goal node has been reached." << std::endl;
      std::cout << "\t - Total execution time:\t\t" << duration.count() << "ms" << std::endl;
      std::cout << "\t - Total number of nodes inspected:\t" << nodes_checked << std::endl;

      auto path = ReconstructPath(cameFrom, start, goal);
      scratch_map_->DrawPath(path);

      return path;
    }

    for (auto next : GetNeighbors(current)) {
      if (allow_near_optimal_solution_ && scratch_map_->IsOccupied(next.x, next.y)) {
        // Skip previously checked nodes. This significantly reduces computation time, but it does
        // not guarantee an optimal solution. The solution will be near-optimal at worst.
        continue;
      }

      next.g = current.g + movement_cost_;
      next.h = heuristic_->operator()(next, goal);
      next.f = next.g + next.h;

      if (cameFrom.find(next) == cameFrom.end() || next.g < cameFrom[next].g) {
        cameFrom[next] = current;
        frontier.push(next);
      }

      // Visualize the process and mark the 'next' node as checked.
      scratch_map_->operator()(next.x, next.y) = checked_node_gray_val_;
      nodes_checked++;
    }
  }

  if (frontier.empty()) {
    throw std::runtime_error("No path found from start to goal");
  }

  return std::vector<Node>{};
}

std::vector<Node> AStar::GetNeighbors(const Node & node) const
{
  std::vector<Node> neighbors;
  std::vector<std::pair<int, int>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

  for (const auto & dir : directions) {
    int newX = node.x + dir.first;
    int newY = node.y + dir.second;

    if (newX >= 0 && newX < map_->GetWidth() && newY >= 0 && newY < map_->GetHeight()) {
      if (!map_->IsOccupied(newX, newY)) {
        neighbors.push_back(Node{newX, newY});
      }
    }
  }

  return neighbors;
}

std::vector<Node> AStar::ReconstructPath(
  const std::unordered_map<Node, Node, NodeHash> & cameFrom, const Node & start,
  const Node & goal) const
{
  std::vector<Node> total_path;
  Node current = goal;
  while (current != start) {
    total_path.push_back(current);
    current = cameFrom.at(current);
  }
  total_path.push_back(start);
  std::reverse(total_path.begin(), total_path.end());
  return total_path;
}

}  // namespace pathfinding_algorithms
