# Pathfinding Algorithms in C++

This repository contains an implementation of the A* search algorithm in C++. The goal of this code is to find an optimal (or near-optimal) path in terms of distance between two points on a map imported from a `.pgm` file.

## Getting Started

To use this project, follow these steps to build it:

```bash
mkdir build && \
cmake -S . -B ./build && \
make -C ./build
```

After building the project, you can run the binary by navigating to the build directory and executing the binary file. Here's how you can do it:

```bash
cd ./build
./task_1 "../maps/map_2.pgm" 2 2 20 2 
```

Alternatively, you can use a different map and start/end points:

```bash
./task_1 "../maps/its.pgm" 100 300 3400 1500
```

## Results

| Method                          | Execution Time (ms) | Nodes Inspected | Result File                                                                               |
| ------------------------------- | ------------------- | --------------- | ----------------------------------------------------------------------------------------- |
| Euclidean Optimal Solution      | 38557               | 4634297         | [its_euclidean_optimal_solution.pgm](./docs/its_euclidean_optimal_solution.pgm)           |
| Manhattan Optimal Solution      | 39977               | 4229139         | [its_manhattan_optimal_solution.pgm](./docs/its_manhattan_optimal_solution.pgm)           |
| Manhattan Near Optimal Solution | 17858               | 801426          | [its_manhattan_near_optimal_solution.pgm](./its_docs/manhattan_near_optimal_solution.pgm) |
| Euclidean Near Optimal Solution | 27391               | 891952          | [its_euclidean_near_optimal_solution.pgm](./its_docs/euclidean_near_optimal_solution.pgm) |

There are several strategies available to optimize the A* algorithm's execution time and computational demand. One approach is to redefine a neighbour node as a cell that is i.e. 3 pixels away (define chunks of pixels), rather than a directly adjacent cell. This reduces the number of nodes the algorithm needs to inspect, potentially significantly improving execution time. Another strategy is to use the squared Euclidean distance (`dx^2 + dy^2`) instead of the standard Euclidean distance formula (`sqrt(dx^2 + dy^2)`). This eliminates the need for a computationally expensive square root operation, thereby reducing the algorithm's computational demand.

Please note that while these optimizations may not always yield the optimal path, they can provide near-optimal solutions much more quickly.

## Code Structure & Features

The architecture outlined above supports the use of pre-existing pathfinding A* algorithm, as well as the development of custom solutions using the same interface. The code structure is detailed in the [API.md](./API.md) file.
