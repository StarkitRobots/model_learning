#include "rhoban_model_learning/humanoid_models/calibration_data_set_reader.h"

#include "rhoban_model_learning/humanoid_models/calibration_input.h"

#include <rhoban_random/tools.h>
#include <rhoban_utils/util.h>

#include "Types/MatrixLabel.hpp"

namespace rhoban_model_learning
{
CalibrationDataSetReader::CalibrationDataSetReader()
  : x_coord_range(0, 10000)
  , y_coord_range(0, 10000)
  , nb_training_tags(-1)
  , nb_validation_tags(-1)
  , max_samples_per_tag(1)
  , min_samples_per_tag(1)
  , rescale_width(0)
  , rescale_height(0)
  , verbose(true)
{
}

DataSet CalibrationDataSetReader::extractSamples(const std::string& file_path, std::default_random_engine* engine) const
{
  Leph::MatrixLabel logs;
  logs.load(file_path);
  // First: Read all entries and conserve only valid ones
  std::map<int, SampleVector> samples_by_id;
  for (size_t i = 0; i < logs.size(); i++)
  {
    Leph::VectorLabel& entry = logs[i];
    int tag_id = entry("tag_id");
    if (rescale_width != 0)
    {
      entry("pixel_x") = (entry("pixel_x") + 1 / 2.0) * rescale_width;
      entry("pixel_x_uncorrected") = (entry("pixel_x_uncorrected") + 1) / 2.0 * rescale_width;
    }
    if (rescale_height != 0)
    {
      entry("pixel_y") = (entry("pixel_y") + 1 / 2.0) * rescale_height;
      entry("pixel_y_uncorrected") = (entry("pixel_y_uncorrected") + 1) / 2.0 * rescale_height;
    }
    Eigen::Vector2d observation(entry("pixel_x_uncorrected"), entry("pixel_y_uncorrected"));
    std::unique_ptr<Input> input(new CalibrationInput(entry));
    std::unique_ptr<Sample> sample(new Sample(std::move(input), observation));

    double x = observation(0);
    double y = observation(1);

    if (x >= x_coord_range(0) && x <= x_coord_range(1) && y >= y_coord_range(0) && y <= y_coord_range(1))
    {
      samples_by_id[tag_id].push_back(std::move(sample));
    }
  }
  // Get valid indices
  if (verbose)
  {
    std::cout << "Filtering tag indices" << std::endl;
  }
  std::vector<int> tags_indices;
  for (const auto& pair : samples_by_id)
  {
    int tag_id = pair.first;
    int nb_samples = pair.second.size();
    if (nb_samples >= min_samples_per_tag)
    {
      tags_indices.push_back(pair.first);
    }
    else if (verbose)
    {
      std::cout << "\tIgnoring tag " << tag_id << " because it has only " << nb_samples << " valid samples"
                << std::endl;
    }
  }
  // Choose which tags will be used for training and validation
  int training_size = nb_training_tags;
  int validation_size = nb_validation_tags;
  if (training_size == -1 && validation_size == -1)
  {
    throw std::runtime_error(DEBUG_INFO + "No size specified for either training or validation");
  }
  else if (training_size + validation_size > (int)tags_indices.size())
  {
    throw std::runtime_error(DEBUG_INFO + "Not enough tags available: (" + std::to_string(validation_size) + "+" +
                             std::to_string(training_size) + ">" + std::to_string(tags_indices.size()) + ")");
  }
  if (validation_size == -1)
  {
    validation_size = tags_indices.size() - training_size;
  }
  if (training_size == -1)
  {
    training_size = tags_indices.size() - validation_size;
  }

  int nb_unused_tags = tags_indices.size() - (training_size + validation_size);

  std::vector<size_t> set_sizes = { (size_t)training_size, (size_t)validation_size, (size_t)nb_unused_tags };
  std::vector<std::vector<size_t>> separated_indices;
  separated_indices = rhoban_random::splitIndices(tags_indices.size() - 1, set_sizes, engine);
  // Fill data set
  DataSet data;
  if (verbose)
    std::cout << "Training indices" << std::endl;
  for (size_t idx : separated_indices[0])
  {
    // Limited number of samples for training_set
    int tag_id = tags_indices[idx];
    const SampleVector& tag_samples = samples_by_id[tag_id];
    int nb_samples = tag_samples.size();
    std::vector<size_t> samples_indices = rhoban_random::getUpToKDistinctFromN(max_samples_per_tag, nb_samples, engine);
    for (size_t sample_idx : samples_indices)
    {
      data.training_set.push_back(tag_samples[sample_idx]->clone());
    }
    if (verbose)
    {
      std::cout << "\t" << tag_id << ": " << samples_indices.size() << " samples" << std::endl;
    }
  }
  if (verbose)
    std::cout << "Validation indices" << std::endl;
  for (size_t idx : separated_indices[1])
  {
    // Limited number of samples for training_set
    int tag_id = tags_indices[idx];
    const SampleVector& tag_samples = samples_by_id[tag_id];
    for (const auto& sample : tag_samples)
    {
      data.validation_set.push_back(sample->clone());
    }
    if (verbose)
    {
      std::cout << "\t" << tag_id << ": " << tag_samples.size() << " samples" << std::endl;
    }
  }

  if (verbose)
  {
    std::cout << "Training Set size: " << data.training_set.size() << std::endl;
    std::cout << "Validation Set size: " << data.validation_set.size() << std::endl;
  }

  return data;
}

std::string CalibrationDataSetReader::getClassName() const
{
  return "CalibrationDataSetReader";
}

Json::Value CalibrationDataSetReader::toJson() const
{
  Json::Value v;
  v["x_coord_range"] = rhoban_utils::vector2Json(x_coord_range);
  v["y_coord_range"] = rhoban_utils::vector2Json(y_coord_range);
  v["nb_training_tags"] = nb_training_tags;
  v["nb_validation_tags"] = nb_validation_tags;
  v["max_samples_per_tag"] = max_samples_per_tag;
  v["min_samples_per_tag"] = min_samples_per_tag;
  v["rescale_width"] = rescale_width;
  v["rescale_height"] = rescale_height;
  v["verbose"] = verbose;
  return v;
}
void CalibrationDataSetReader::fromJson(const Json::Value& v, const std::string& dir_name)
{
  (void)dir_name;
  rhoban_utils::tryReadEigen(v, "x_coord_range", &x_coord_range);
  rhoban_utils::tryReadEigen(v, "y_coord_range", &y_coord_range);
  rhoban_utils::tryRead(v, "nb_training_tags", &nb_training_tags);
  rhoban_utils::tryRead(v, "nb_validation_tags", &nb_validation_tags);
  rhoban_utils::tryRead(v, "max_samples_per_tag", &max_samples_per_tag);
  rhoban_utils::tryRead(v, "min_samples_per_tag", &min_samples_per_tag);
  rhoban_utils::tryRead(v, "rescale_width", &rescale_width);
  rhoban_utils::tryRead(v, "rescale_height", &rescale_height);
  rhoban_utils::tryRead(v, "verbose", &verbose);
}

}  // namespace rhoban_model_learning
