# Software Architecture & Capabilities

![code_structure](./docs/code_structure.png)

## `PathFindingStrategy` Class

*Defined in header [`pathfinding/algorithms.hpp`](./include/pathfinding/algorithms.hpp)*

#### Member Functions

- `PathFindingStrategy(std::shared_ptr<OccupancyGrid> map)`: Constructs the `PathFindingStrategy` object. Takes a shared pointer to an `OccupancyGrid` object as an argument.

- `virtual ~PathFindingStrategy() = default`: Virtual destructor.

- `virtual std::vector<Node> FindPath(Node & start, Node & goal) = 0`: Pure virtual function that finds a path from the start node to the goal node. Must be implemented by any class that inherits from `PathFindingStrategy`.

#### Protected Members

- ```const std::shared_ptr<OccupancyGrid> map_```: Shared pointer to an `OccupancyGrid` object representing the environment.

## `AStar` Class

*Defined in header [`pathfinding/algorithms.hpp`](./include/pathfinding/algorithms.hpp)*

#### Member Functions

- `AStar(std::shared_ptr<OccupancyGrid> map, std::unique_ptr<Heuristic> heuristic, float movement_cost = 0.7)`: Constructs the `AStar` object. Takes a shared pointer to an `OccupancyGrid` object, a unique pointer to a `Heuristic` object, and a float representing the movement cost as arguments.

- `virtual ~AStar()`: Destructor.

- `std::vector<Node> FindPath(Node & start, Node & goal) override`: Finds the path from the start node to the goal node using the A* algorithm.

- `void SaveSummaryAsPGM(const std::string filename) const`: Saves a summary as a PGM file that shows checked pixels and path. The checked pixels are shown in light gray color, and the path is shown in dark gray color.

- `void SetMovementCost(const float new_cost)`: Sets the movement cost for the algorithm.

- `void SetHeuristic(std::unique_ptr<Heuristic> new_h)`: Sets the heuristic used in the algorithm.

- `void AllowNearOptimalSolution(const bool allow)`: Sets whether to allow near-optimal solution.

#### Private Members

- `std::vector<Node> GetNeighbors(const Node & node) const`: Retrieves neighboring nodes for a given node. The neighboring nodes are the nodes that are adjacent to the given node (without diagonal movement) and are not occupied. If the given node is on the edge of the map, the neighboring nodes will be limited to the nodes that are within the map.

- `std::vector<Node> ReconstructPath(const std::unordered_map<Node, Node, NodeHash> & cameFrom, const Node & start, const Node & goal) const`: Reconstructs the path from start to goal.

- `std::unique_ptr<OccupancyGrid> scratch_map_`: Scratch map used for visualization and marking checked nodes. It is possible to save it as a .pgm file. The checked pixels are shown in light gray color.

- `std::unique_ptr<Heuristic> heuristic_`: Heuristic used in the algorithm.

- `float movement_cost_`: Represents the movement cost between two adjacent nodes. This value is used to calculate the g value of a node. Setting a new value for this variable could be useful for manipulating the influence of the heuristic value.

- `bool allow_near_optimal_solution_ = false`: Specifies whether to allow a near-optimal solution. If this is set to true, the algorithm will skip previously checked nodes. This significantly reduces computation solution will be near-optimal at worst.

- `static constexpr uint8_t checked_node_gray_val_ = 30`: Value used to mark checked nodes in the scratch map.

## Utils Structures

*Defined in header [`pathfinding/utils.hpp`](./include/pathfinding/utils.hpp)*

### Node

- `int x, y`: Coordinates of the node.
- `float g`: Cost to reach the node from the start node.
- `float h`: Estimated cost to reach the goal node from the current node.
- `float f`: Sum of `g` and `h`.

### NodeHash

- `std::size_t operator()(const Node & node) const`: Generates a hash value for a Node. This is used for the unordered_map container.

### CellOccupancyLevel

- `FREE = 0`: Represents free cells.
- `UNKNOWN = 127`: Represents unknown cells.
- `OCCUPIED = 255`: Represents occupied cells.

## Utils Classes

*Defined in header [`pathfinding/utils.hpp`](./include/pathfinding/utils.hpp)*

### OccupancyGrid

- `OccupancyGrid()`: Default constructor.
- `OccupancyGrid(const std::string filename, const uint8_t free_threshold = 40)`: Constructor that loads an occupancy grid from a file.
- `OccupancyGrid(std::vector<uint8_t> pixels, const unsigned width, const unsigned height, const unsigned max_value)`: Constructor to initialize an occupancy grid with provided parameters.

- `void SaveAsPGM(const std::string filename) const`: Saves the occupancy grid as a PGM file.
- `bool IsOccupied(int x, int y) const`: Checks if a specific cell is occupied.
- `void DrawPath(const std::vector<Node> & path, const uint8_t color = 120)`: Draws a path on the occupancy grid.

- `uint8_t & operator()(int x, int y)`: Accesses the pixel value at the specified coordinates.
- `const uint8_t & operator()(int x, int y) const`: Accesses the pixel value at the specified coordinates (const version).

#### Heuristic

- `virtual ~Heuristic() = default`: Virtual destructor.
- `virtual float operator()(const Node & start, const Node & end) = 0`: Pure virtual function to calculate the heuristic value between two nodes.

#### ManhattanHeuristic

- `float operator()(const Node & start, const Node & end) override`: Calculates the Manhattan distance heuristic value.

#### EuclideanHeuristic

- `float operator()(const Node & start, const Node & end) override`: Calculates the Euclidean distance heuristic value.
