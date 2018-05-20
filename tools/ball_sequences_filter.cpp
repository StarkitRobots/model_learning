#include "rhoban_model_learning/ball_models/position_sequence_reader.h"

#include <iostream>

using namespace rhoban_model_learning;

int main(int argc, char ** argv) {

  if (argc < 3) {
    std::cerr << "Usage: " << argv[0]
              << "<position_sequence_reader.json> <trajectory_file>" << std::endl;
    exit(EXIT_FAILURE);
  }

  std::string reader_path(argv[1]);
  std::string traj_path(argv[2]);

  PositionSequenceReader r;
  r.loadFile(reader_path);

  std::vector<PositionSequence> sequences = r.readPositionSequences(traj_path);

  std::cout << "log,time,ball_x,ball_y" << std::endl;

  for (size_t idx = 0; idx < sequences.size(); idx++) {
    const PositionSequence & seq = sequences[idx];
    for (const Eigen::Vector3d & entry : seq.timed_positions) {
      std::cout << idx << "," << entry[0] << "," << entry[1] << "," << entry[2] << std::endl;
    }
  }
}
