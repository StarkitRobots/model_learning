#include "rhoban_model_learning/ball_models/trajectory_predictor.h"

#include "rhoban_model_learning/ball_models/position_predictor_factory.h"
#include "rhoban_model_learning/ball_models/speed_estimator_factory.h"

#include "rhoban_random/tools.h"

#include "rhoban_utils/util.h"

namespace rhoban_model_learning
{

TrajectoryPredictor::Input::Input() {}
TrajectoryPredictor::Input::Input(const Input & other)
  : ball_positions(other.ball_positions),
    prediction_time(other.prediction_time)
{
}

TrajectoryPredictor::Input::Input(const PositionSequence & seq, double t)
  : ball_positions(seq), prediction_time(t)
{
}

TrajectoryPredictor::Input::~Input() {}

std::unique_ptr<rhoban_model_learning::Input> TrajectoryPredictor::Input::clone() const {
  return std::unique_ptr<rhoban_model_learning::Input>(new Input(*this));
}

DataSet TrajectoryPredictor::Reader::extractSamples(const std::string & file_path,
                                                    std::default_random_engine * engine) const
{
  DataSet data;
  std::vector<PositionSequence> sequences = sequence_reader.readPositionSequences(file_path);

  // Selecting sequences used for training
  size_t training_size = nb_training_sequences;
  if (sequences.size() < training_size) {
    throw std::runtime_error(DEBUG_INFO + " Not enough sequences: " + std::to_string(sequences.size())
                             + " while expecting at least " + std::to_string(training_size));
  }
  size_t validation_size = sequences.size() - training_size;
  std::vector<size_t> set_sizes = {training_size, validation_size};
  std::vector<std::vector<size_t>> separated_indices;
  separated_indices = rhoban_random::splitIndices(sequences.size() - 1, set_sizes, engine);
  // Generate TrajectoryPredictor inputs
  std::uniform_real_distribution<double>
    start_distribution(min_time_to_start, max_time_to_start),
    dt_distribution(min_dt, max_dt);
  for (size_t set_idx : {0,1}) {
    bool training_set = set_idx == 0;
    for (size_t seq_idx : separated_indices[set_idx]) {
      const PositionSequence & seq = sequences[seq_idx];
      if (seq.timed_positions.size() == 0) {
        throw std::logic_error(DEBUG_INFO + " Empty sequence");
      }
      // 1. Get sequence start
      double start = seq.getStartTime() + start_distribution(*engine);
      double end = start + memory_duration;
      // 2. Getting wished prediction time and finding closest sample (time)
      double prediction_time = end + dt_distribution(*engine);
      double best_diff = std::numeric_limits<double>::max();
      Eigen::Vector3d best_entry;
      for (const Eigen::Vector3d & entry : seq.timed_positions) {
        double time_error = fabs(entry(0) - prediction_time);
        if (time_error < best_diff) {
          best_diff = time_error;
          best_entry = entry;
        }
      }
      prediction_time = best_entry(0);
      Eigen::Vector2d ball_measured_pos = best_entry.segment(1,2);
      // 3. Building sample and adding it to the appropriate set
      std::unique_ptr<rhoban_model_learning::Input> input(
        new TrajectoryPredictor::Input(seq.extractSequence(start, end), prediction_time));
      std::unique_ptr<Sample> sample(new Sample(std::move(input), ball_measured_pos));
      if (training_set) {
        data.training_set.push_back(std::move(sample));
      } else {
        data.validation_set.push_back(std::move(sample));
      }
    }
  }
  return data;
}


Json::Value TrajectoryPredictor::Reader::toJson() const {
  Json::Value v;
  v["sequence_reader"] = sequence_reader.toJson();
  v["nb_training_sequences"] = nb_training_sequences;
  v["min_time_to_start"    ] = min_time_to_start    ;
  v["max_time_to_start"    ] = max_time_to_start    ;
  v["memory_duration"      ] = memory_duration      ;
  v["min_dt"               ] = min_dt               ;
  v["max_dt"               ] = max_dt               ;
  return v;
}

void TrajectoryPredictor::Reader::fromJson(const Json::Value & v, const std::string & dir_name) {
  sequence_reader.read(v, "sequence_reader", dir_name);
  rhoban_utils::tryRead(v, "nb_training_sequences", &nb_training_sequences);
  rhoban_utils::tryRead(v, "min_time_to_start"    , &min_time_to_start    );
  rhoban_utils::tryRead(v, "max_time_to_start"    , &max_time_to_start    );
  rhoban_utils::tryRead(v, "memory_duration"      , &memory_duration      );
  rhoban_utils::tryRead(v, "min_dt"               , &min_dt               );
  rhoban_utils::tryRead(v, "max_dt"               , &max_dt               );
}

std::string TrajectoryPredictor::Reader::getClassName() const {
  return "TrajectoryPredictor";
}


TrajectoryPredictor::TrajectoryPredictor()
  : Model(0), speed_estimator(), position_predictor()
{
}

TrajectoryPredictor::TrajectoryPredictor(const TrajectoryPredictor & other)
  : Model(other)
{
  speed_estimator = std::unique_ptr<SpeedEstimator>((SpeedEstimator*)other.speed_estimator->clone().release());
  position_predictor = std::unique_ptr<PositionPredictor>((PositionPredictor*)other.position_predictor->clone().release());
}

TrajectoryPredictor::~TrajectoryPredictor() {}

Eigen::VectorXd
TrajectoryPredictor::predictObservation(const rhoban_model_learning::Input & raw_input,
                                        std::default_random_engine * engine) const {
  try {
    const Input & input = dynamic_cast<const Input &>(raw_input);

    // 1. Ball status estimation
    // - Estimating speed at last position seen
    //   - An offset could be used (estimation of speed in the past is more accurate)
    // - Pos is estimated using last entry (quite bad, should be changed)
    //   - Very sensitive to noise + no smoothing at all
    SpeedEstimator::Input se_input;
    se_input.seq = input.ball_positions;
    se_input.prediction_time = input.ball_positions.getEndTime();
    Eigen::Vector2d ball_pos = input.ball_positions.timed_positions.back().segment(1,2);
    Eigen::Vector2d ball_speed = speed_estimator->predictObservation(se_input, engine);
    // 2. Ball future estimation
    PositionPredictor::Input pp_input;
    pp_input.ball_pos = ball_pos;
    pp_input.ball_speed = ball_speed;
    pp_input.prediction_duration = input.prediction_time - se_input.prediction_time;
    return position_predictor->predictObservation(pp_input, engine).segment(0,2);
  } catch (const std::bad_cast & exc) {
    throw std::logic_error(DEBUG_INFO + " invalid type for input");
  }
}


Eigen::VectorXi TrajectoryPredictor::getObservationsCircularity() const {
  return Eigen::VectorXi::Zero(2);
}

Eigen::VectorXd TrajectoryPredictor::getGlobalParameters() const {
  Eigen::VectorXd se_params = speed_estimator->getGlobalParameters();
  Eigen::VectorXd pp_params = position_predictor->getGlobalParameters();
  Eigen::VectorXd params(se_params.rows() + pp_params.rows());
  params << se_params, pp_params;
  return params;
}

Eigen::MatrixXd TrajectoryPredictor::getGlobalParametersSpace() const {
  Eigen::MatrixXd se_limits = speed_estimator->getGlobalParametersSpace();
  Eigen::MatrixXd pp_limits = position_predictor->getGlobalParametersSpace();
  Eigen::MatrixXd limits(se_limits.rows() + pp_limits.rows(),2);
  limits << se_limits, pp_limits;
  return limits;
}

void TrajectoryPredictor::setGlobalParameters(const Eigen::VectorXd & new_params) {
  int se_rows = speed_estimator->getGlobalParametersCount();
  int pp_rows = position_predictor->getGlobalParametersCount();
  if (new_params.rows() != se_rows + pp_rows) {
    throw std::logic_error(DEBUG_INFO + " Invalid number of parameters: "
                           + std::to_string(new_params.rows()) + " received while expecting "
                           + std::to_string(se_rows + pp_rows));
  }
  speed_estimator->setGlobalParameters(new_params.segment(0,se_rows));
  position_predictor->setGlobalParameters(new_params.segment(se_rows, pp_rows));
}

std::vector<std::string> TrajectoryPredictor::getGlobalParametersNames() const {
  std::vector<std::string> names, se_names, pp_names;
  se_names = speed_estimator->getGlobalParametersNames();
  pp_names = position_predictor->getGlobalParametersNames();
  names.insert(names.end(), se_names.begin(), se_names.end());
  names.insert(names.end(), pp_names.begin(), pp_names.end());
  return names;
}

Json::Value TrajectoryPredictor::toJson() const {
  Json::Value v = Model::toJson();
  v["speed_estimator"] = speed_estimator->toFactoryJson();
  v["position_predictor"] = position_predictor->toFactoryJson();
  return v;
}

void TrajectoryPredictor::fromJson(const Json::Value & v, const std::string & dir_name) {
  Model::fromJson(v, dir_name);
  speed_estimator = SpeedEstimatorFactory().read(v, "speed_estimator", dir_name);
  position_predictor = PositionPredictorFactory().read(v, "position_predictor", dir_name);
}

std::string TrajectoryPredictor::getClassName() const {
  return "TrajectoryPredictor";
}

std::unique_ptr<Model> TrajectoryPredictor::clone() const {
  return std::unique_ptr<Model>(new TrajectoryPredictor(*this));
}

}
