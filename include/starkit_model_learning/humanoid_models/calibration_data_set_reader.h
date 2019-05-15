#pragma once

#include "starkit_model_learning/data_set_reader.h"

namespace starkit_model_learning
{
class CalibrationDataSetReader : public DataSetReader
{
public:
  CalibrationDataSetReader();

  /// Throw an error if:
  /// - none of `nb_training_tags` and `nb_validation_tags` is provided
  /// - The sum of `nb_training_tags` and `nb_validation_tags` is higher
  ///   than the number of valid tags
  virtual DataSet extractSamples(const std::string& file_path, std::default_random_engine* engine) const override;

  virtual std::string getClassName() const override;
  Json::Value toJson() const override;
  void fromJson(const Json::Value& v, const std::string& dir_name) override;

private:
  /// The acceptable range for x coordinates [px]
  Eigen::Vector2d x_coord_range;
  /// The acceptable range for y coordinates [px]
  Eigen::Vector2d y_coord_range;
  /// Number of tags used for the training set
  /// If value is < 0, the number of tags is deduced from nb_validation_tags
  int nb_training_tags;
  /// Number of tags used for validation. By choosing to use separate training
  /// set and validation set based on tagId, we ensure that the validation set
  /// is really different from the training set
  /// If value is < 0, the number of tags is deduced from nb_training_tags
  int nb_validation_tags;
  /// The maximal number of samples per tag allowed in training_set
  int max_samples_per_tag;
  /// If a tag is represented less than min_samples_per_tag, it is ignored
  int min_samples_per_tag;
  /// Unnormalize the information from data file to be able to read old format
  double rescale_width;
  /// Unnormalize the information from data file to be able to read old format
  double rescale_height;
  /// Is the reading process printing summary of reading?
  bool verbose;
};

}  // namespace starkit_model_learning
