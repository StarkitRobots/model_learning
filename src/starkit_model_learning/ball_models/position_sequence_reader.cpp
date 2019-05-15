#include "starkit_model_learning/ball_models/position_sequence_reader.h"

#include "starkit_model_learning/ball_models/speed_estimator_factory.h"

#include "starkit_utils/io_tools.h"
#include "starkit_utils/util.h"

namespace starkit_model_learning
{
PositionSequenceReader::PositionSequenceReader()
  : high_threshold(0.6)
  , low_threshold(0.1)
  , anticipation(0.2)
  , allowed_time_gap(0.5)
  , min_amplitude(0.8)
  , min_point_count(20)
{
}

std::vector<PositionSequence> PositionSequenceReader::readPositionSequences(const std::string& file_path) const
{
  if (!speed_estimator)
  {
    throw std::logic_error(DEBUG_INFO + " speed estimator is null");
  }
  std::vector<std::string> lines = starkit_utils::file2lines(file_path);
  // Checking consistency of file
  if (lines.size() == 0)
  {
    throw std::runtime_error(DEBUG_INFO + " empty file " + file_path);
  }
  // Choosing format
  int log_name_column, traj_name_column, time_column, x_column, y_column;
  size_t expected_columns;
  if (lines[0] == "log,time,xWorld,yWorld,vxWorld,vyWorld,xSelf,ySelf,vxSelf,vySelf")
  {
    log_name_column = 0;
    traj_name_column = -1;
    time_column = 1;
    x_column = 2;
    y_column = 3;
    expected_columns = 10;
    // TODO: export should rather contain xField yField
  }
  else if (lines[0] == "log,traj,time,ball_x,ball_y")
  {
    log_name_column = 0;
    traj_name_column = 1;
    time_column = 2;
    x_column = 3;
    y_column = 4;
    expected_columns = 5;
  }
  else
  {
    throw std::runtime_error(DEBUG_INFO + " unexpected header '" + lines[0] + "'");
  }
  // Some properties for reading
  // From lines to vector of PositionSequences
  std::vector<PositionSequence> unfiltered_sequences;
  std::string current_log_name;
  std::string current_traj_name;
  for (size_t line_idx = 1; line_idx < lines.size(); line_idx++)
  {
    // Separating columns
    std::vector<std::string> elements;
    starkit_utils::split(lines[line_idx], ',', elements);
    if (elements.size() != expected_columns)
    {
      throw std::runtime_error(DEBUG_INFO + " invalid line: '" + lines[line_idx] + "'");
    }
    // Parsing content
    std::string log_name = log_name_column >= 0 ? elements[log_name_column] : "default";
    std::string traj_name = traj_name_column >= 0 ? elements[traj_name_column] : "default";
    double time = std::stod(elements[time_column]);
    double ball_x = std::stod(elements[x_column]);
    double ball_y = std::stod(elements[y_column]);
    // Add a new entry if log has changed
    if (log_name != current_log_name || traj_name != current_traj_name)
    {
      current_log_name = log_name;
      current_traj_name = traj_name;
      unfiltered_sequences.push_back(PositionSequence());
    }
    // Insert entry into last sequence
    unfiltered_sequences.back().addEntry(Eigen::Vector3d(time, ball_x, ball_y));
  }
  // Filtering sequences
  std::vector<PositionSequence> filtered_sequences;
  for (const PositionSequence& raw_seq : unfiltered_sequences)
  {
    std::vector<PositionSequence> sequences = splitSequence(raw_seq);
    for (const PositionSequence& sequence : sequences)
    {
      if (sequence.size() < min_point_count)
        continue;

      double dist = (sequence.getStartPos() - sequence.getEndPos()).norm();
      if (dist < min_amplitude)
        continue;

      filtered_sequences.push_back(sequence);
    }
  }
  return filtered_sequences;
}

std::vector<PositionSequence> PositionSequenceReader::splitSequence(const PositionSequence& seq) const
{
  if (!speed_estimator)
  {
    throw std::logic_error(DEBUG_INFO + " Speed estimator not initialized");
  }
  // Estimating speed at each entry
  std::vector<double> estimated_speeds;
  for (const Eigen::Vector3d& entry : seq.timed_positions)
  {
    SpeedEstimator::Input input;
    input.seq = seq;  // Note: using a copy constructor here is was suboptimal
    input.prediction_time = entry(0);
    Eigen::VectorXd speed = speed_estimator->predictObservation(input, nullptr);
    estimated_speeds.push_back(speed.norm());
  }
  // Populate vector
  bool new_seq_allowed = true;
  std::vector<PositionSequence> sequences;
  for (size_t idx = 0; idx < estimated_speeds.size(); idx++)
  {
    // Checking timing
    double now = seq.timed_positions[idx](0);
    double time_since_last = -1;
    if (idx > 0)
    {
      time_since_last = now - seq.timed_positions[idx - 1](0);
    }
    // Looking speed for futur entries
    double futur_speed = 0;
    double time_limit = now + anticipation;
    for (size_t futur_idx = idx; futur_idx < estimated_speeds.size(); futur_idx++)
    {
      if (seq.timed_positions[futur_idx](0) > time_limit)
        break;
      futur_speed = std::max(futur_speed, estimated_speeds[futur_idx]);
    }
    // Opening a new sequence
    if (time_since_last > allowed_time_gap || (new_seq_allowed && (futur_speed > high_threshold)))
    {
      sequences.push_back(PositionSequence());
      new_seq_allowed = false;
    }
    // Allowing start of a new sequence
    if (futur_speed < low_threshold)
    {
      new_seq_allowed = true;
    }
    // Fill the current sequence
    if (sequences.size() > 0)
    {
      sequences.back().addEntry(seq.timed_positions[idx]);
    }
  }
  return sequences;
}

std::string PositionSequenceReader::getClassName() const
{
  return "PositionSequenceReader";
}

Json::Value PositionSequenceReader::toJson() const
{
  Json::Value v;
  v["speed_estimator"] = speed_estimator->toFactoryJson();
  v["low_threshold"] = low_threshold;
  v["high_threshold"] = high_threshold;
  v["anticipation"] = anticipation;
  v["allowed_time_gap"] = allowed_time_gap;
  return v;
}

void PositionSequenceReader::fromJson(const Json::Value& v, const std::string& dir_name)
{
  speed_estimator = SpeedEstimatorFactory().read(v, "speed_estimator", dir_name);
  starkit_utils::tryRead(v, "high_threshold", &high_threshold);
  starkit_utils::tryRead(v, "low_threshold", &low_threshold);
  starkit_utils::tryRead(v, "anticipation", &anticipation);
  starkit_utils::tryRead(v, "allowed_time_gap", &allowed_time_gap);
}

}  // namespace starkit_model_learning
