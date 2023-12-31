#ifndef PATHFINDING_ALGORITHMS_HPP_
#define PATHFINDING_ALGORITHMS_HPP_

#include <cstddef>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "pathfinding/utils.hpp"

namespace pathfinding_algorithms
{

using namespace pathfinding_utils;

class PathFindingStrategy
{
public:
  virtual ~PathFindingStrategy() = default;
  virtual std::vector<Node> FindPath(Node & start, Node & goal) = 0;
};

class AStar : public PathFindingStrategy
{
public:
  AStar(
    std::shared_ptr<OccupancyGrid> map, std::unique_ptr<Heuristic> heuristic,
    float movement_cost = 0.7)
  : map_(map), heuristic_(std::move(heuristic)), movement_cost_(movement_cost){};
  virtual ~AStar(){};

  std::vector<Node> FindPath(Node & start, Node & goal) override;

  void SaveSummaryAsPGM(const std::string filename) const { scratch_map_->SaveAsPGM(filename); }

  void SetMovementCost(const float new_cost) { movement_cost_ = new_cost; }
  void SetHeuristic(std::unique_ptr<Heuristic> new_h) { heuristic_ = std::move(new_h); }
  void AllowNearOptimalSolution(const bool allow) { allow_near_optimal_solution_ = allow; }

private:
  std::vector<Node> GetNeighbors(const Node & node) const;
  std::vector<Node> ReconstructPath(
    const std::unordered_map<Node, Node, NodeHash> & cameFrom, const Node & start,
    const Node & goal) const;

  const std::shared_ptr<OccupancyGrid> map_;
  std::unique_ptr<OccupancyGrid> scratch_map_;
  std::unique_ptr<Heuristic> heuristic_;
  float movement_cost_;
  bool allow_near_optimal_solution_ = false;
  static constexpr uint8_t checked_node_gray_val_ = 30;
};

}  // namespace pathfinding_algorithms

#endif  // PATHFINDING_ALGORITHMS_HPP_