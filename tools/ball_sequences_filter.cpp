#include "starkit_model_learning/ball_models/position_sequence_reader.h"
#include "starkit_model_learning/ball_models/speed_estimator_factory.h"

#include <iostream>

using namespace starkit_model_learning;

int main(int argc, char** argv)
{
  if (argc < 4)
  {
    std::cerr << "Usage: " << argv[0] << "<position_sequence_reader.json> <speed_estimator.json> <trajectory_files.csv>"
              << std::endl;
    exit(EXIT_FAILURE);
  }

  std::string reader_path(argv[1]);
  std::string speed_estimator_path(argv[2]);
  std::vector<std::string> traj_paths;
  for (int i = 3; i < argc; i++)
  {
    traj_paths.push_back(argv[i]);
  }

  PositionSequenceReader r;
  r.loadFile(reader_path);

  std::unique_ptr<SpeedEstimator> se = SpeedEstimatorFactory().buildFromJsonFile(speed_estimator_path);
  std::cout << "log,traj,time,ball_x,ball_y,vx,vy" << std::endl;

  int log_idx = 0;
  for (const std::string& traj_path : traj_paths)
  {
    std::vector<PositionSequence> sequences = r.readPositionSequences(traj_path);
    for (size_t traj_idx = 0; traj_idx < sequences.size(); traj_idx++)
    {
      const PositionSequence& seq = sequences[traj_idx];
      for (const Eigen::Vector3d& entry : seq.timed_positions)
      {
        SpeedEstimator::Input input;
        input.seq = seq;
        input.prediction_time = entry[0];
        Eigen::VectorXd speed = se->predictObservation(input, nullptr);
        std::cout << log_idx << "," << traj_idx << "," << entry[0] << "," << entry[1] << "," << entry[2] << ","
                  << speed[0] << "," << speed[1] << std::endl;
      }
    }
    log_idx++;
  }
}
