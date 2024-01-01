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
  /**
   * @brief Coordinate of the node
   */
  int x, y;

  /**
   * @brief Cost to reach the node from the start node
   */
  float g;

  /**
   * @brief Estimated cost to reach the goal node from the current node
   */
  float h;

  /**
   * @brief Sum of g and h
   */
  float f;
};

struct NodeHash
{
  /**
   * @brief Generates a hash value for a Node. This is used for the unordered_map container.
   * @param node The Node object to hash.
   * @return std::size_t The hash value generated.
   */
  std::size_t operator()(const Node & node) const
  {
    return std::hash<int>()(node.x) ^ std::hash<int>()(node.y);
  }
};

bool operator==(const Node & lhs, const Node & rhs);

bool operator!=(const Node & lhs, const Node & rhs);

/**
 * @brief Compares two nodes based on their f values.
 *
 * @param lhs The left-hand side node.
 * @param rhs The right-hand side node.
 *
 * @return true if the f value of lhs is greater than that of rhs.
 *
 * @note The logic in the operator< function is negated because it is used for a priority queue in
 * the A* algorithm. In the A* algorithm, the node with the lowest f value (the sum of the cost to
 * reach the node and the estimated cost to the goal) is considered the highest priority. Therefore,
 * the nodes are sorted in descending order of f values, which is achieved by using the greater-than
 * operator (>) instead of the less-than operator (<).
 */
bool operator<(const Node & lhs, const Node & rhs);

/**
 * @brief Enum representing cell occupancy levels.
 *
 * @note The values of the enum are negated compared to the values of the pixels in the .pgm file
 * due to the definitions of free and occupied thresholds.
 */
enum CellOccupancyLevel : uint8_t {
  FREE = 0,
  UNKNOWN = 127,
  OCCUPIED = 255,
};

class OccupancyGrid
{
public:
  OccupancyGrid() : width_(0), height_(0), max_value_(0) {}

  /**
   * @brief Constructor that loads an occupancy grid from a file.
   *
   * @param filename The filename of the PGM file.
   * @param free_threshold The threshold for free cells.
   */
  OccupancyGrid(const std::string filename, const uint8_t free_threshold = 40);

  /**
   * @brief Constructor to initialize an occupancy grid with provided parameters.
   *
   * @param pixels The pixel data representing the grid.
   * @param width The width of the grid.
   * @param height The height of the grid.
   * @param max_value The maximum pixel value.
   */
  OccupancyGrid(
    std::vector<uint8_t> pixels, const unsigned width, const unsigned height,
    const unsigned max_value);

  /**
   * @brief Saves the occupancy grid as a PGM file. The file extension is automatically appended if
   * it is not provided.
   *
   * @throws std::runtime_error if unable to open the file or if map size does not match the number
   * of pixel data
   *
   * @param filename The filename for the saved PGM file.
   *
   * @note The pixel values are inverted before saving. In .pgm format, 0 represents black and 255
   * represents white, while in the OccupancyGrid class, 0 (associated with white color) represents
   * free due to the definitions of the free and occupied thresholds.
   */
  void SaveAsPGM(const std::string filename) const;

  /**
   * @brief Checks if a specific cell is occupied.
   *
   * @param x The x-coordinate of the cell.
   * @param y The y-coordinate of the cell.
   *
   * @throws std::out_of_range if the coordinates are out of range.
   *
   * @return bool True if the cell is occupied, false otherwise.
   */
  bool IsOccupied(int x, int y) const;

  /**
   * @brief Draws a path on the occupancy grid.
   *
   * @param path The path to draw.
   * @param color The color of the path (0-255). Default value is 120.
   */
  void DrawPath(const std::vector<Node> & path, const uint8_t color = 120);

  unsigned GetWidth() const { return width_; }
  unsigned GetHeight() const { return height_; }
  unsigned GetMaxValue() const { return max_value_; }

  /**
   * @brief Accesses the pixel value at the specified coordinates.
   *
   * @param x The x-coordinate of the pixel.
   * @param y The y-coordinate of the pixel.
   *
   * @return uint8_t& Reference to the pixel value.
   */
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

  /**
   * @brief Accesses the pixel value at the specified coordinates (const version).
   *
   * @param x The x-coordinate of the pixel.
   * @param y The y-coordinate of the pixel.
   *
   * @return const uint8_t& Const reference to the pixel value.
   */
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
  /**
   * @brief Get the vector of pixels from .pgm file.
   *
   * @param filename The filename of the .pgm file.
   *
   * @throws std::runtime_error if unable to open the file or if the file is not in .pgm format
   *
   * @note The pixel values are inverted before saving. In .pgm format, 0 represents black and 255
   * represents white, while in the OccupancyGrid class, 0 (associated with white color) represents
   * free due to the definitions of the free and occupied thresholds.
   */
  void GetGridFromPGM(const std::string & filename);

  /**
   * @brief Thresholds the pixel values based on the free threshold. The pixels with values less
   * than or equal to the free threshold are considered free. Other pixels are considered occupied.
   *
   * @param free_threshold The threshold for free cells.
   */
  void ThresholdValues(const uint8_t free_threshold);

  /**
   * @brief Validates the header of the .pgm file (max_value, width, height)
   *
   * @throws std::runtime_error if the header is invalid
   */
  void ValidateHeader() const;

  /**
   * @brief Validates the size of the .pgm file (width * height == number of pixel data)
   *
   * @throws std::runtime_error if the size is invalid
   */
  void ValidateSize() const;

  /**
   * @brief Ensures that the filename has the .pgm extension.
   *
   * @param filename The filename to check.
   *
   * @return std::string The filename with the .pgm extension.
   */
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

  /**
   * @brief Calculates the heuristic value between two nodes.
   *
   * @param start The starting node.
   * @param end The goal node.
   *
   * @return float The calculated heuristic value.
   */
  virtual float operator()(const Node & start, const Node & end) = 0;
};

class ManhattanHeuristic : public Heuristic
{
public:
  /**
   * @brief Calculates the Manhattan distance heuristic value.
   *
   * @param start The starting node.
   * @param end The goal node.
   *
   * @return float The calculated heuristic value.
   */
  float operator()(const Node & start, const Node & end) override
  {
    return std::abs(end.x - start.x) + std::abs(end.y - start.y);
  }
};

class EuclideanHeuristic : public Heuristic
{
public:
  /**
   * @brief Calculates the Euclidean distance heuristic value.
   *
   * @param start The starting node.
   * @param end The goal node.
   *
   * @return float The calculated heuristic value.
   */
  float operator()(const Node & start, const Node & end) override
  {
    return std::hypot(end.x - start.x, end.y - start.y);
  }
};

}  // namespace pathfinding_utils

#endif  // PATHFINDING_UTILS_HPP_
