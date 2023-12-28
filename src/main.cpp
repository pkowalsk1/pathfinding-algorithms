#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "pathfinding/binary_map.hpp"

#define FREE_THRESHOLD 220

using namespace pathfinding_utils;

int main(int argc, char * argv[])
{
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <filename.pgm>" << std::endl;
    return 1;
  }

  std::string map_pgm_filename = argv[1];

  std::shared_ptr<BinaryMap> map = std::make_shared<BinaryMap>(map_pgm_filename, FREE_THRESHOLD);
  std::cout << "Map size: " << map->GetWidth() << "x" << map->GetHeight() << std::endl;

  map->SaveAsPGM("out.pgm");

  return 0;
}