#pragma once

#include "rhoban_model_learning/data_set_reader.h"

namespace rhoban_model_learning
{

class PosesOptimizationDataSetReader : public DataSetReader {
public:
  PosesOptimizationDataSetReader();

  /// Throw an error if:
  /// - none of `nb_training_images` and `nb_validation_images` is provided
  /// - The sum of `nb_training_images` and `nb_validation_tags` is higher
  ///   than the number of images with at least 'min_tags'
  virtual DataSet extractSamples(const std::string & file_path,
                                 std::default_random_engine * engine) const override;

  virtual std::string getClassName() const override;
  Json::Value toJson() const override;
  void fromJson(const Json::Value & v, const std::string & dir_name) override;
private:
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
  
  /// Printing debug information
  bool verbose;
};

}
