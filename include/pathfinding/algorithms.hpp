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
  PathFindingStrategy(std::shared_ptr<OccupancyGrid> map) : map_(map){};
  virtual ~PathFindingStrategy() = default;

  /**
   * @brief Finds a path from start to goal node.
   *
   * @param start The starting node.
   * @param goal The goal node.
   *
   * @return std::vector<Node> The path from start to goal.
   */
  virtual std::vector<Node> FindPath(Node & start, Node & goal) = 0;

protected:
  /**
   * @brief Occupancy grid representing the environment. This is a shared pointer because the map
   * could be shared with other algorithms.
   */
  const std::shared_ptr<OccupancyGrid> map_;
};

class AStar : public PathFindingStrategy
{
public:
  /**
   * @brief Constructor for A* algorithm.
   *
   * @param map The occupancy grid representing the environment.
   * @param heuristic The heuristic used in pathfinding.
   *
   * @param movement_cost The cost for movement. Default value is 0.7.
   */
  AStar(
    std::shared_ptr<OccupancyGrid> map, std::unique_ptr<Heuristic> heuristic,
    float movement_cost = 0.7)
  : PathFindingStrategy(map), heuristic_(std::move(heuristic)), movement_cost_(movement_cost){};

  virtual ~AStar(){};

  /**
   * @brief Finds the path from start to goal using A* algorithm.
   *
   * @param start The starting node.
   * @param goal The goal node.
   *
   * @return std::vector<Node> The path from start to goal.
   */
  std::vector<Node> FindPath(Node & start, Node & goal) override;

  /**
   * @brief Saves a summary as a PGM file that shows checked pixels and path. The checked pixels are
   * shown in light gray color, and the path is shown in dark gray color.
   *
   * @param filename The filename for the saved PGM file. The extension will be automatically added.
   */
  void SaveSummaryAsPGM(const std::string filename) const { scratch_map_->SaveAsPGM(filename); }

  /**
   * @brief Sets the movement cost for the algorithm. The default value is 0.7.
   *
   * @param new_cost The new movement cost.
   *
   * @note This value is used to calculate the g value of a node. Setting a new value for this
   * variable could be useful for manipulating the influence of the heuristic value.
   */
  void SetMovementCost(const float new_cost) { movement_cost_ = new_cost; }

  /**
   * @brief Sets the heuristic used in the algorithm.
   *
   * @param new_h The new heuristic.
   */
  void SetHeuristic(std::unique_ptr<Heuristic> new_h) { heuristic_ = std::move(new_h); }

  /* @brief Sets whether to allow near-optimal solution.
   *
   * @param allow Whether to allow near-optimal solution.
   *
   * @note If this is set to true, the algorithm will skip previously checked nodes. This
   * significantly reduces computation time, but it does not guarantee an optimal solution. The
   * solution will be near-optimal at worst.
   */
  void AllowNearOptimalSolution(const bool allow) { allow_near_optimal_solution_ = allow; }

private:
  /**
   * @brief Retrieves neighboring nodes for a given node. The neighboring nodes are the nodes that
   * are adjacent to the given node (without diagonal movement) and are not occupied. If the given
   * node is on the edge of the map, the neighboring nodes will be limited to the nodes that are
   * within the map.
   *
   * @param node The node for which neighbors are obtained.
   *
   * @return std::vector<Node> The neighboring nodes.
   */
  std::vector<Node> GetNeighbors(const Node & node) const;

  /**
   * @brief Reconstructs the path from start to goal using the unordered map that stores the parent
   * node for each node. The path is reconstructed by starting from the goal node and following the
   * parent node until the start node is reached.
   *
   * @param cameFrom The unordered_map that stores the parent node for each node.
   * @param start The starting node.
   * @param goal The goal node.
   *
   * @return std::vector<Node> The path from start to goal.
   */
  std::vector<Node> ReconstructPath(
    const std::unordered_map<Node, Node, NodeHash> & cameFrom, const Node & start,
    const Node & goal) const;

  /**
   * @brief Scratch map used for visualization and marking checked nodes. The map is only used for
   * these purposes and is not shared with other algorithms. It is possible to save it as a .pgm
   * file. The checked pixels are shown in light gray color.
   */
  std::unique_ptr<OccupancyGrid> scratch_map_;

  /**
   * @brief Heuristic used in the algorithm.
   */
  std::unique_ptr<Heuristic> heuristic_;

  /**
   * @brief Represents the movement cost between two adjacent nodes (in this case, only neighbors in
   * the grid). It is connected with manipulating the influence of the heuristic value.
   */
  float movement_cost_;

  /**
   * @brief Specifies whether to allow a near-optimal solution. If set to true, the algorithm will
   * skip previously checked nodes. This significantly reduces computation time, but it does not
   * guarantee an optimal solution because the algorithm will stop checking other paths once it
   * finds a path to the goal. The solution will be near-optimal at worst, but still satisfying.
   */
  bool allow_near_optimal_solution_ = false;

  /**
   * @brief Value used to mark checked nodes in the scratch map.
   */
  static constexpr uint8_t checked_node_gray_val_ = 30;
};

}  // namespace pathfinding_algorithms

#endif  // PATHFINDING_ALGORITHMS_HPP_