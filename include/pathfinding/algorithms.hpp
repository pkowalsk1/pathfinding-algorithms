#ifndef PATHFINDING_ALGORITHMS_HPP_
#define PATHFINDING_ALGORITHMS_HPP_

#include <cstddef>
#include <functional>
#include <memory>
#include <string>
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

  /**
   * @brief Finds a path from the start to the goal node.
   *
   * @param map A shared pointer to the occupancy grid map representing the environment.
   * @param start The starting node for the path.
   * @param goal The goal node for the path.
   * @param summary_filename The name of the file where the pathfinding process will be logged.
   *
   * @return std::vector<Node> A path from start to goal, represented as a vector of
   * nodes.
   */
  virtual std::vector<Node> FindPath(
    std::shared_ptr<const OccupancyGrid> map, Node start, Node goal,
    const std::string & summary_filename) = 0;
};

class AStar : public PathFindingStrategy
{
public:
  /**
   * @brief Constructor for A* algorithm.
   *
   * @param heuristic A unique pointer to the heuristic used in pathfinding.
   * @param movement_cost The cost for movement. Default value is 1.0.
   * @param allow_near_optimal_solution A boolean flag indicating whether near-optimal solutions are
   * allowed. Default value is false.
   */
  AStar(
    std::unique_ptr<Heuristic> heuristic, float movement_cost = 1.0,
    bool allow_near_optimal_solution = false)
  : heuristic_(std::move(heuristic)),
    movement_cost_(movement_cost),
    allow_near_optimal_solution_(allow_near_optimal_solution){};

  virtual ~AStar(){};

  /**
   * @brief Finds the shortest path from start to goal using the A* algorithm on a given occupancy
   * grid map.
   *
   * The pathfinding process is visualized in a summary .pgm file specified by the summary_filename
   * parameter.
   *
   * @param map A shared pointer to the occupancy grid map representing the environment.
   * @param start The starting node for the path.
   * @param goal The goal node for the path.
   * @param summary_filename The name of the .pgm file where the pathfinding process will be
   * visualized. File extension is automatically added if not specified.
   *
   * @throws std::runtime_error If the start or goal node is occupied or if no path is found.
   *
   * @return std::vector<Node> The shortest path from start to goal, represented as a vector of
   * nodes.
   */
  std::vector<Node> FindPath(
    std::shared_ptr<const OccupancyGrid> map, Node start, Node goal,
    const std::string & summary_filename) override;

  /**
   * @brief Sets the movement cost for the algorithm. The default value is 1.0.
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

  /**
   * @brief Sets whether to allow near-optimal solution.
   *
   * @param allow Whether to allow near-optimal solution.
   *
   * @note If this is set to true, the algorithm will skip previously checked nodes. This
   * significantly reduces computation time, but it does not guarantee an optimal solution. The
   * solution will be near-optimal at worst.
   */
  void SetNearOptimalSolution(const bool allow) { allow_near_optimal_solution_ = allow; }

private:
  /**
   * @brief Retrieves unoccupied neighboring nodes for a given node from a specified map.
   *
   * The neighboring nodes are the nodes that are adjacent to the given node (without diagonal
   * movement) and are not occupied. If the given node is on the edge of the map, the neighboring
   * nodes will be limited to the nodes that are within the map.
   *
   * @param node The node for which unoccupied neighbors are obtained.
   * @param map A shared pointer to the occupancy grid map.
   *
   * @return std::vector<Node> The unoccupied neighboring nodes.
   */
  static std::vector<Node> GetNeighbors(
    const Node & node, std::shared_ptr<const OccupancyGrid> map);

  /**
   * @brief Reconstructs the path from start to goal using the unordered map that stores the parent
   * node for each node. The path is reconstructed by starting from the goal node and following the
   * parent node until the start node is reached.
   *
   * @param came_from The unordered_map that stores the parent node for each node.
   * @param start The starting node.
   * @param goal The goal node.
   *
   * @return std::vector<Node> The path from start to goal.
   */
  std::vector<Node> ReconstructPath(
    const std::unordered_map<Node, Node, NodeHash> & came_from, const Node & start,
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
  bool allow_near_optimal_solution_;

  /**
   * @brief Value used to mark checked nodes in the scratch map.
   */
  static constexpr uint8_t checked_node_gray_val_ = 30;
};

}  // namespace pathfinding_algorithms

#endif  // PATHFINDING_ALGORITHMS_HPP_