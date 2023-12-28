#ifndef PATHFINDING_BINARY_MAP_HPP_
#define PATHFINDING_BINARY_MAP_HPP_

#include <cstdint>
#include <iostream>
#include <memory>
#include <vector>

namespace pathfinding_utils
{

enum CellOccupancyLevel : uint8_t {
  FREE = 0,
  UNKNOWN = 127,
  OCCUPIED = 255,
};

class BinaryMap
{
public:
  BinaryMap(const std::string filename, const uint8_t free_threshold);
  BinaryMap(
    std::vector<uint8_t> pixels, const unsigned width, const unsigned height,
    const unsigned max_value);

  void SaveAsPGM(const std::string filename) const;
  bool IsOccupied(int x, int y) const;

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
  void GetMapFromPGM(const std::string & filename);
  void ThresholdMapValues(const uint8_t free_threshold);
  void ValidateMapHeader() const;
  void ValidateMapSize() const;
  std::string EnsurePGMExtension(const std::string & filename) const;

  unsigned width_;
  unsigned height_;
  unsigned max_value_;
  std::vector<uint8_t> pixels_;
};

}  // namespace pathfinding_utils

#endif  // PATHFINDING_BINARY_MAP_HPP_
