#include "rhoban_model_learning/model_factory.h"
#include "rhoban_model_learning/ball_models/position_sequence_reader.h"
#include "rhoban_model_learning/ball_models/trajectory_predictor.h"

#include "rhoban_random/tools.h"

// Debug program:
// Read and simulate trajectories

using namespace rhoban_model_learning;

class Config : public rhoban_utils::JsonSerializable {
public:
  Config() : min_time_to_start(0), max_time_to_start(0.5), memory_duration(0.5){
  }

  std::string getClassName() const override {
    return "BallTrajectoryDebug";
  }

  Json::Value toJson() const override {
    throw std::logic_error("Not implemented");
  }

  void fromJson(const Json::Value & v, const std::string & dir_name) override {
    reader.read(v, "reader", dir_name);
    model = ModelFactory().read(v, "model", dir_name);
    rhoban_utils::tryRead(v, "min_time_to_start"    , &min_time_to_start    );
    rhoban_utils::tryRead(v, "max_time_to_start"    , &max_time_to_start    );
    rhoban_utils::tryRead(v, "memory_duration"      , &memory_duration      );
  }

  PositionSequenceReader reader;

  /// All available models
  std::unique_ptr<Model> model;

  double min_time_to_start;
  double max_time_to_start;
  double memory_duration;
};


int main(int argc, char ** argv) {
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <config.json> <data_file>"
              << std::endl;
    exit(EXIT_FAILURE);
  }

  Config config;
  config.loadFile(argv[1]);

  /// Getting sequences
  std::vector<PositionSequence> sequences = config.reader.readPositionSequences(argv[2]);

  // Analyze data
  std::default_random_engine engine = rhoban_random::getRandomEngine();

  std::ofstream csv_file("debug.csv");
  csv_file << "seq,data_type,time,ball_x,ball_y" << std::endl;

  for (size_t seq_id = 0; seq_id < sequences.size(); seq_id++) {
    const PositionSequence & seq = sequences[seq_id];
    std::map<std::string, PositionSequence> seq_by_type;
    // 1. Get sequence start
    std::uniform_real_distribution<double> start_distribution(config.min_time_to_start,
                                                              config.max_time_to_start);
    double start = seq.getStartTime() + start_distribution(engine);
    double end = start + config.memory_duration;
    // 2. Separating data
    seq_by_type["ignored"  ] = seq.extractSequence(seq.getStartTime(), start);
    seq_by_type["input"    ] = seq.extractSequence(start, end);
    seq_by_type["measured" ] = seq.extractSequence(end, seq.getEndTime());
    seq_by_type["predicted"] = PositionSequence();
    // 3. Predicting based on model
    for (const Eigen::Vector3d pos : seq_by_type["measured"].timed_positions) {
      double time = pos(0);
      TrajectoryPredictor::Input input(seq_by_type["input"], time);
      Eigen::Vector2d predicted = config.model->predictObservation(input, &engine);
      Eigen::Vector3d entry;
      entry(0) = time;
      entry.segment(1,2) = predicted;
      seq_by_type["predicted"].addEntry(entry);
    }
    // 4. Writing Data
    for (const auto & pair : seq_by_type) {
      std::string seq_name = pair.first;
      for (const Eigen::Vector3d & pos : pair.second.timed_positions) {
        csv_file << seq_id << "," << seq_name << ","
                 << pos(0) << "," << pos(1) << "," << pos(2) << std::endl;
      }
    }
  }
}
