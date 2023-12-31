#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "pathfinding/algorithms.hpp"
#include "pathfinding/utils.hpp"

using namespace pathfinding_algorithms;
using namespace pathfinding_utils;

int main(int argc, char * argv[])
{
  if (argc != 6) {
    std::cerr << "Usage: " << argv[0] << " <filename.pgm> <start_x> <start_y> <goal_x> <goal_y>"
              << std::endl;
    return 1;
  }

  std::string map_pgm_filename = argv[1];

  int start_x = std::stoi(argv[2]);
  int start_y = std::stoi(argv[3]);
  int goal_x = std::stoi(argv[4]);
  int goal_y = std::stoi(argv[5]);

  std::shared_ptr<OccupancyGrid> map = std::make_shared<OccupancyGrid>(map_pgm_filename);
  Node start = {start_x, start_y};
  Node goal = {goal_x, goal_y};

  AStar a_star = AStar(map, std::make_unique<EuclideanHeuristic>());
  std::vector<Node> path = a_star.FindPath(start, goal);
  a_star.SaveSummaryAsPGM("results/euclidean_optimal_solution.pgm");

  a_star.SetHeuristic(std::make_unique<ManhattanHeuristic>());
  a_star.SetMovementCost(1.0);
  path = a_star.FindPath(start, goal);
  a_star.SaveSummaryAsPGM("results/manhattan_optimal_solution.pgm");

  a_star.AllowNearOptimalSolution(true);
  path = a_star.FindPath(start, goal);
  a_star.SaveSummaryAsPGM("results/manhattan_near_optimal_solution.pgm");

  a_star.SetHeuristic(std::make_unique<EuclideanHeuristic>());
  a_star.SetMovementCost(0.7);
  path = a_star.FindPath(start, goal);
  a_star.SaveSummaryAsPGM("results/euclidean_near_optimal_solution.pgm");

  return 0;
}

// Map has been successfully loaded. Size of the map: 4633x2403

// Pathfinding A* algorithm has started. It may take some time...

// Pathfinding successful! Goal node has been reached.
//          - Total execution time:                38557ms
//          - Total number of nodes inspected:     4634297
// Map has been successfully saved to: results/euclidean_optimal_solution.pgm

// Pathfinding A* algorithm has started. It may take some time...

// Pathfinding successful! Goal node has been reached.
//          - Total execution time:                39977ms
//          - Total number of nodes inspected:     4229139
// Map has been successfully saved to: results/manhattan_optimal_solution.pgm

// Pathfinding A* algorithm has started. It may take some time...

// Pathfinding successful! Goal node has been reached.
//          - Total execution time:                17858ms
//          - Total number of nodes inspected:     801426
// Map has been successfully saved to: results/manhattan_near_optimal_solution.pgm

// Pathfinding A* algorithm has started. It may take some time...

// Pathfinding successful! Goal node has been reached.
//          - Total execution time:                27391ms
//          - Total number of nodes inspected:     891952
// Map has been successfully saved to: results/euclidean_near_optimal_solution.pgm