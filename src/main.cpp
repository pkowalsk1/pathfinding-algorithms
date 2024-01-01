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

  AStar a_star = AStar(std::make_unique<EuclideanHeuristic>());

  std::vector<Node> path = a_star.FindPath(
    map, start, goal, "../results/euclidean_best_solution.pgm");

  a_star.SetHeuristic(std::make_unique<ManhattanHeuristic>());
  path = a_star.FindPath(map, start, goal, "../results/manhattan_best_solution.pgm");

  a_star.SetNearOptimalSolution(true);
  path = a_star.FindPath(map, start, goal, "../results/manhattan_high_quality_solution.pgm");

  a_star.SetHeuristic(std::make_unique<EuclideanHeuristic>());
  path = a_star.FindPath(map, start, goal, "../results/euclidean_high_quality_solution.pgm");

  return 0;
}
