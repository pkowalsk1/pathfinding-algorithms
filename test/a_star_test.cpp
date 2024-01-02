#include <gtest/gtest.h>

#include <pathfinding/algorithms.hpp>
#include <pathfinding/utils.hpp>

using namespace pathfinding_algorithms;
using namespace pathfinding_utils;

TEST(AStarTest, ThrowsWhenStartOrGoalIsOccupied)
{
  std::string summary_filename = "./Testing/Temporary/test_map";

  std::vector<uint8_t> pixels(100, 0);
  auto start = Node{0, 0};
  auto goal = Node{9, 9};

  auto map = std::make_shared<OccupancyGrid>(pixels, 10, 10, 255);
  map->SetPixel(start.x, start.y, CellOccupancyLevel::OCCUPIED);

  AStar astar(std::make_unique<ManhattanHeuristic>());

  EXPECT_THROW(
    {
      try {
        astar.FindPath(map, start, goal, summary_filename);
      } catch (const std::runtime_error & e) {
        EXPECT_STREQ("Start node is occupied", e.what());
        throw;
      }
    },
    std::runtime_error);

  map->SetPixel(start.x, start.y, CellOccupancyLevel::FREE);
  map->SetPixel(goal.x, goal.y, CellOccupancyLevel::OCCUPIED);

  EXPECT_THROW(
    {
      try {
        astar.FindPath(map, start, goal, summary_filename);
      } catch (const std::runtime_error & e) {
        EXPECT_STREQ("Goal node is occupied", e.what());
        throw;
      }
    },
    std::runtime_error);
}

TEST(AStarTest, ThrowsWhenNoPath)
{
  std::string summary_filename = "./Testing/Temporary/test_map";

  std::vector<uint8_t> pixels(100, 0);
  auto map = std::make_shared<OccupancyGrid>(pixels, 10, 10, 255);
  auto start = Node{0, 0};
  auto goal = Node{9, 9};

  // Make a wall in the middle of the map
  for (int i = 0; i < 10; ++i) {
    map->SetPixel(i, 5, CellOccupancyLevel::OCCUPIED);
  }

  AStar astar(std::make_unique<ManhattanHeuristic>());

  EXPECT_THROW(
    {
      try {
        astar.FindPath(map, start, goal, summary_filename);
      } catch (const std::runtime_error & e) {
        EXPECT_STREQ("No path found from start to goal", e.what());
        throw;
      }
    },
    std::runtime_error);
}

TEST(AStarTest, ReturnsCorrectPath)
{
  std::string summary_filename = "./Testing/Temporary/test_map";

  std::vector<uint8_t> pixels(100, 0);
  auto map = std::make_shared<OccupancyGrid>(pixels, 10, 10, 255);
  auto start = Node{0, 0};
  auto goal = Node{9, 9};

  AStar astar(std::make_unique<ManhattanHeuristic>());

  std::vector<Node> path = astar.FindPath(map, start, goal, summary_filename);

  EXPECT_EQ(path.front().x, start.x);
  EXPECT_EQ(path.front().y, start.y);
  EXPECT_EQ(path.back().x, goal.x);
  EXPECT_EQ(path.back().y, goal.y);

  // Check that the path is valid (each node is adjacent to the next one and is not occupied)
  for (size_t i = 0; i < path.size() - 1; ++i) {
    const auto & node1 = path[i];
    const auto & node2 = path[i + 1];

    // Check that node1 and node2 are adjacent
    EXPECT_TRUE(std::abs(node1.x - node2.x) <= 1 && std::abs(node1.y - node2.y) <= 1);

    EXPECT_FALSE(map->IsOccupied(path[i].x, path[i].y));
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}