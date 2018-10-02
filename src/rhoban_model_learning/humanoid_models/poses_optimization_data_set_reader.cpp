#include "rhoban_model_learning/humanoid_models/calibration_data_set_reader.h"

#include "rhoban_model_learning/humanoid_models/poses_optimization_input.h"

#include <rhoban_random/tools.h>
#include <rhoban_utils/tables/string_table.h>
#include <rhoban_utils/util.h>

namespace rhoban_model_learning
{

typedef PosesOptimizationDataSetReader PODSR;
typedef PosesOptimizationInput POI;

PODSR::PosesOptimizationDataSetReader()
  : nb_training_tags(-1), nb_validation_tags(-1),
    max_samples_per_tag(1), min_samples_per_tag(1),
    verbose(false)
{
}

DataSet PODSR::extractSamples(const std::string & file_path,
                              std::default_random_engine * engine) const
{
  StringTable data = StringTable::buildFromFile(file_path);

  std::map<int, >> inputs_by_image;
  for (size_t row = 0; row < data.nbRows(); row++) {
    std::map<std::string, std::string> row_content = data.getRow(row);
    int image_id = std::stoi(row_content["image_id"]);
    int marker_id = std::stoi(row_content["marker_id"]);
    double pixel_x = std::stod(row_content["pixel_x"]);
    double pixel_y = std::stod(row_content["pixel_y"]);
    inputs_by_image[image_id]
  }
  
  // Get valid indices
  if (verbose) {
    std::cout << "Filtering tag indices" << std::endl;
  }
  std::vector<int> tags_indices;
  for (const auto & pair : samples_by_id) {
    int tag_id = pair.first;
    int nb_samples = pair.second.size();
    if (nb_samples >= min_samples_per_tag) {
      tags_indices.push_back(pair.first);
    } else if (verbose) {
      std::cout << "\tIgnoring tag " << tag_id << " because it has only "
                << nb_samples << " valid samples" << std::endl;
    }
  }
  // Choose which tags will be used for training and validation
  int training_size = nb_training_tags;
  int validation_size = nb_validation_tags;
  if (training_size == -1 && validation_size == -1) { 
    throw std::runtime_error(DEBUG_INFO + "No size specified for either training or validation");
  } else if (training_size + validation_size > (int) tags_indices.size()) {
    throw std::runtime_error(DEBUG_INFO + "Not enough tags available: ("
                             + std::to_string(validation_size) + "+"
                             + std::to_string(training_size) + ">"
                             + std::to_string(tags_indices.size()) + ")");
  }
  if (validation_size == -1) {
    validation_size = tags_indices.size() - training_size;
  }
  if (training_size == -1) {
    training_size = tags_indices.size() - validation_size;
  }

  int nb_unused_tags = tags_indices.size() - (training_size+validation_size);

  std::vector<size_t> set_sizes =
    {(size_t)training_size, (size_t)validation_size, (size_t)nb_unused_tags};
  std::vector<std::vector<size_t>> separated_indices;
  separated_indices = rhoban_random::splitIndices(tags_indices.size() - 1, set_sizes, engine);
  // Fill data set
  DataSet data;
  if (verbose) std::cout << "Training indices" << std::endl;
  for (size_t idx : separated_indices[0]) {
    // Limited number of samples for training_set
    int tag_id = tags_indices[idx];
    const SampleVector & tag_samples = samples_by_id[tag_id];
    int nb_samples = tag_samples.size();
    std::vector<size_t> samples_indices =
      rhoban_random::getUpToKDistinctFromN(max_samples_per_tag, nb_samples, engine);
    for (size_t sample_idx : samples_indices) {
      data.training_set.push_back(tag_samples[sample_idx]->clone());
    }
    if (verbose) {
      std::cout << "\t" << tag_id << ": " << samples_indices.size() << " samples" << std::endl;
    }
  }
  if (verbose) std::cout << "Validation indices" << std::endl;
  for (size_t idx : separated_indices[1]) {
    // Limited number of samples for training_set
    int tag_id = tags_indices[idx];
    const SampleVector & tag_samples = samples_by_id[tag_id];
    for (const auto & sample : tag_samples) {
      data.validation_set.push_back(sample->clone());
    }
    if (verbose) {
      std::cout << "\t" << tag_id << ": " << tag_samples.size() << " samples" << std::endl;
    }
  }

  if (verbose) {
    std::cout << "Training Set size: " << data.training_set.size() << std::endl;
    std::cout << "Validation Set size: " << data.validation_set.size() << std::endl;
  }
  
  return data;
}

std::string PODSR::getClassName() const {
  return "CalibrationDataSetReader";
}

Json::Value PODSR::toJson() const {
  Json::Value  v;
  v["x_coord_range"] = rhoban_utils::vector2Json(x_coord_range);
  v["y_coord_range"] = rhoban_utils::vector2Json(y_coord_range);
  v["nb_training_tags"   ] = nb_training_tags   ;
  v["nb_validation_tags" ] = nb_validation_tags ;
  v["max_samples_per_tag"] = max_samples_per_tag;
  v["min_samples_per_tag"] = min_samples_per_tag;
  v["rescale_width"      ] = rescale_width      ;
  v["rescale_height"     ] = rescale_height     ;
  v["verbose"            ] = verbose            ;
  return v;
}
void PODSR::fromJson(const Json::Value & v, 
                                        const std::string & dir_name) {
  (void) dir_name;
  rhoban_utils::tryReadEigen(v, "x_coord_range", &x_coord_range);
  rhoban_utils::tryReadEigen(v, "y_coord_range", &y_coord_range);
  rhoban_utils::tryRead(v, "nb_training_tags"   , &nb_training_tags   );
  rhoban_utils::tryRead(v, "nb_validation_tags" , &nb_validation_tags );
  rhoban_utils::tryRead(v, "max_samples_per_tag", &max_samples_per_tag);
  rhoban_utils::tryRead(v, "min_samples_per_tag", &min_samples_per_tag);
  rhoban_utils::tryRead(v, "rescale_width"      , &rescale_width      );
  rhoban_utils::tryRead(v, "rescale_height"     , &rescale_height     );
  rhoban_utils::tryRead(v, "verbose"            , &verbose            );
}

}
