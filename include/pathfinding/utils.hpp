#ifndef PATHFINDING_UTILS_HPP_
#define PATHFINDING_UTILS_HPP_

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <vector>

namespace pathfinding_utils
{

struct Node
{
  int x, y;
  float g, h, f;
};

struct NodeHash
{
  std::size_t operator()(const Node & node) const
  {
    return std::hash<int>()(node.x) ^ std::hash<int>()(node.y);
  }
};

bool operator==(const Node & lhs, const Node & rhs);

bool operator!=(const Node & lhs, const Node & rhs);

bool operator<(const Node & lhs, const Node & rhs);

enum CellOccupancyLevel : uint8_t {
  FREE = 0,
  UNKNOWN = 127,
  OCCUPIED = 255,
};

class OccupancyGrid
{
public:
  OccupancyGrid() : width_(0), height_(0), max_value_(0) {}
  OccupancyGrid(const std::string filename, const uint8_t free_threshold = 40);
  OccupancyGrid(
    std::vector<uint8_t> pixels, const unsigned width, const unsigned height,
    const unsigned max_value);

  void SaveAsPGM(const std::string filename) const;
  bool IsOccupied(int x, int y) const;
  void DrawPath(const std::vector<Node> & path, const uint8_t color = 120);

  unsigned GetWidth() const { return width_; }
  unsigned GetHeight() const { return height_; }
  unsigned GetMaxValue() const { return max_value_; }

  uint8_t & operator()(int x, int y)
  {
    if (x < 0 || x >= width_ || y < 0 || y >= height_) {
      throw std::out_of_range("Pixel coordinates out of range");
    }
    uint8_t & pixel = pixels_[y * width_ + x];
    if (pixel > max_value_) {
      throw std::runtime_error("Pixel value is greater than max value");
    }
    return pixel;
  }

  const uint8_t & operator()(int x, int y) const
  {
    if (x < 0 || x >= width_ || y < 0 || y >= height_) {
      throw std::out_of_range("Pixel coordinates out of range");
    }
    const uint8_t & pixel = pixels_[y * width_ + x];
    if (pixel > max_value_) {
      throw std::runtime_error("Pixel value is greater than max value");
    }
    return pixel;
  }

private:
  void GetGridFromPGM(const std::string & filename);
  void ThresholdValues(const uint8_t free_threshold);
  void ValidateHeader() const;
  void ValidateSize() const;
  std::string EnsurePGMExtension(const std::string & filename) const;

  unsigned width_;
  unsigned height_;
  unsigned max_value_;
  std::vector<uint8_t> pixels_;
};

class Heuristic
{
public:
  virtual ~Heuristic() = default;
  virtual float operator()(const Node & start, const Node & end) = 0;
};

class ManhattanHeuristic : public Heuristic
{
public:
  float operator()(const Node & start, const Node & end) override
  {
    return std::abs(end.x - start.x) + std::abs(end.y - start.y);
  }
};

class EuclideanHeuristic : public Heuristic
{
public:
  float operator()(const Node & start, const Node & end) override
  {
    return std::hypot(end.x - start.x, end.y - start.y);
  }
};

}  // namespace pathfinding_utils

#endif  // PATHFINDING_UTILS_HPP_
