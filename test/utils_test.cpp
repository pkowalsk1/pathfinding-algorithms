#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <pathfinding/utils.hpp>

using namespace pathfinding_utils;

TEST(OccupancyGridTest, MaxValueExceeds255)
{
  std::vector<uint8_t> pixels = {0, 0, 0, 255, 255, 255};
  unsigned width = 3;
  unsigned height = 2;
  unsigned max_value = 300;

  EXPECT_THROW(
    {
      try {
        OccupancyGrid grid(pixels, width, height, max_value);
      } catch (const std::runtime_error & e) {
        EXPECT_STREQ("Max value exceeds 255 (unsupported)", e.what());
        throw;
      }
    },
    std::runtime_error);
}

TEST(OccupancyGridTest, MapSizeDoesNotMatchPixelData)
{
  std::vector<uint8_t> pixels = {0, 0, 0, 255, 255, 255};
  unsigned width = 4;
  unsigned height = 4;
  unsigned max_value = 255;

  EXPECT_THROW(
    {
      try {
        OccupancyGrid grid(pixels, width, height, max_value);
      } catch (const std::runtime_error & e) {
        EXPECT_STREQ(
          "Map import from file failed. Map size does not match the number of pixel data.",
          e.what());
        throw;
      }
    },
    std::runtime_error);
}

TEST(OccupancyGridTest, IsOccupied)
{
  std::vector<uint8_t> pixels = {0, 0, 0, 255, 255, 255};
  unsigned width = 3;
  unsigned height = 2;
  unsigned max_value = 255;
  OccupancyGrid grid(pixels, width, height, max_value);

  EXPECT_FALSE(grid.IsOccupied(0, 0));
  EXPECT_FALSE(grid.IsOccupied(1, 0));
  EXPECT_FALSE(grid.IsOccupied(2, 0));
  EXPECT_TRUE(grid.IsOccupied(0, 1));
  EXPECT_TRUE(grid.IsOccupied(1, 1));
  EXPECT_TRUE(grid.IsOccupied(2, 1));
}

TEST(OccupancyGridTest, DrawPath)
{
  std::vector<uint8_t> pixels = {0, 0, 0, 0, 0, 0};
  unsigned width = 3;
  unsigned height = 2;
  unsigned max_value = 255;
  OccupancyGrid grid(pixels, width, height, max_value);

  std::vector<Node> path = {{0, 0, 0, 0, 0}, {1, 0, 0, 0, 0}, {2, 0, 0, 0, 0}};
  uint8_t path_color = 100;
  grid.DrawPath(path, 100);

  EXPECT_EQ(grid.GetPixel(0, 1), 0);
  EXPECT_EQ(grid.GetPixel(1, 1), 0);
  EXPECT_EQ(grid.GetPixel(2, 1), 0);
  EXPECT_EQ(grid.GetPixel(0, 0), path_color);
  EXPECT_EQ(grid.GetPixel(1, 0), path_color);
  EXPECT_EQ(grid.GetPixel(2, 0), path_color);
}

TEST(OccupancyGridTest, SaveAndLoadPGM)
{
  std::vector<uint8_t> pixels = {0, 0, 0, 255, 255, 255};
  unsigned width = 3;
  unsigned height = 2;
  unsigned max_value = 255;
  OccupancyGrid grid(pixels, width, height, max_value);

  std::string filename = testing::TempDir() + "/test_map";
  grid.SaveAsPGM(filename);

  OccupancyGrid loaded_grid(filename + ".pgm", 127);

  for (unsigned y = 0; y < height; ++y) {
    for (unsigned x = 0; x < width; ++x) {
      uint8_t expected_pixel = pixels[y * width + x];
      EXPECT_EQ(expected_pixel, loaded_grid.GetPixel(x, y));
    }
  }
}

TEST(OccupancyGridTest, NodeOperators)
{
  Node node1 = {1, 1, 1, 1, 1};
  Node node2 = {1, 1, 1, 1, 1};
  Node node3 = {2, 2, 2, 2, 2};

  EXPECT_TRUE(node1 == node2);
  EXPECT_FALSE(node1 != node2);
  EXPECT_TRUE(node3 < node1);
}

TEST(HeuristicTest, ManhattanHeuristic)
{
  ManhattanHeuristic heuristic;
  Node start = {0, 0, 0, 0, 0};
  Node end = {3, 4, 0, 0, 0};

  float expected = 7.0f;
  float actual = heuristic.Calculate(start, end);

  EXPECT_FLOAT_EQ(expected, actual);
}

TEST(HeuristicTest, EuclideanHeuristic)
{
  EuclideanHeuristic heuristic;
  Node start = {0, 0, 0, 0, 0};
  Node end = {3, 4, 0, 0, 0};

  float expected = 5.0f;
  float actual = heuristic.Calculate(start, end);

  EXPECT_FLOAT_EQ(expected, actual);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}