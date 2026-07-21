#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>

#include "track_robot_semantic_memory/deterministic_replay.hpp"

int main(int argc, char ** argv)
{
  if (argc != 3) {
    std::cerr << "usage: semantic_memory_replay INPUT.json OUTPUT.json\n";
    return 2;
  }
  try {
    std::ifstream input(argv[1], std::ios::binary);
    if (!input) {
      throw std::runtime_error("could not open normalized replay input");
    }
    const std::string serialized{
      std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
    const auto output =
      track_robot_semantic_memory::run_normalized_replay(serialized);
    std::ofstream stream(argv[2], std::ios::binary | std::ios::trunc);
    if (!stream) {
      throw std::runtime_error("could not open normalized replay output");
    }
    stream << output << '\n';
    if (!stream) {
      throw std::runtime_error("could not write normalized replay output");
    }
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
