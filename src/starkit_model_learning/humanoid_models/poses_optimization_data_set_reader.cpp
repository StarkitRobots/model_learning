#include "starkit_model_learning/humanoid_models/poses_optimization_data_set_reader.h"

#include "starkit_model_learning/humanoid_models/poses_optimization_input.h"

#include <starkit_random/tools.h>
#include <starkit_utils/tables/string_table.h>
#include <starkit_utils/util.h>

#include <iostream>

namespace starkit_model_learning
{
using starkit_utils::StringTable;

typedef PosesOptimizationDataSetReader PODSR;
typedef PosesOptimizationInput POI;

PODSR::PosesOptimizationDataSetReader()
  : nb_images(-1), training_tags_per_image(-1), validation_tags_per_image(-1), verbose(false)
{
}

DataSet PODSR::extractSamples(const std::string& file_path, std::default_random_engine* engine) const
{
  StringTable data = StringTable::buildFromFile(file_path);

  std::map<int, std::vector<Sample>> samples_by_image;
  for (size_t row = 0; row < data.nbRows(); row++)
  {
    std::map<std::string, std::string> row_content = data.getRow(row);
    int image_id = std::stoi(row_content.at("image_id"));
    int marker_id = std::stoi(row_content.at("marker_id"));
    double pixel_x = std::stod(row_content.at("pixel_x"));
    double pixel_y = std::stod(row_content.at("pixel_y"));
    samples_by_image[image_id].push_back(
        Sample(std::unique_ptr<Input>(new POI(image_id, marker_id)), Eigen::Vector2d(pixel_x, pixel_y)));
  }

  // Get valid images indices
  if (verbose)
  {
    std::cout << "Filtering images" << std::endl;
  }
  std::vector<int> images_indices;
  for (const auto& pair : samples_by_image)
  {
    int image_id = pair.first;
    int nb_samples = pair.second.size();
    if (nb_samples >= training_tags_per_image + validation_tags_per_image)
    {
      images_indices.push_back(pair.first);
    }
    else if (verbose)
    {
      std::cout << "\tIgnoring image " << image_id << " because it has only " << nb_samples << " valid samples"
                << std::endl;
    }
  }
  if (images_indices.size() < (size_t)nb_images)
  {
    throw std::runtime_error(DEBUG_INFO + " not enough images with enough tags (" +
                             std::to_string(images_indices.size()) + " images available, " + std::to_string(nb_images) +
                             " required)");
  }
  // Separating samples
  DataSet data_set;
  std::vector<size_t> chosen_indices = starkit_random::getKDistinctFromN(nb_images, images_indices.size(), engine);
  for (size_t idx : chosen_indices)
  {
    int image_idx = images_indices[idx];
    const std::vector<Sample>& image_samples = samples_by_image[image_idx];
    std::vector<size_t> set_sizes = { (size_t)training_tags_per_image, (size_t)validation_tags_per_image };
    std::vector<std::vector<size_t>> samples_indices =
        starkit_random::splitIndices(image_samples.size() - 1, set_sizes, engine);
    for (size_t training_idx : samples_indices[0])
    {
      data_set.training_set.push_back(image_samples[training_idx].clone());
    }
    for (size_t validation_idx : samples_indices[1])
    {
      data_set.validation_set.push_back(image_samples[validation_idx].clone());
    }
  }

  return data_set;
}

std::string PODSR::getClassName() const
{
  return "CalibrationDataSetReader";
}

Json::Value PODSR::toJson() const
{
  Json::Value v;
  v["nb_images"] = nb_images;
  v["training_tags_per_image"] = training_tags_per_image;
  v["validation_tags_per_image"] = validation_tags_per_image;
  v["verbose"] = verbose;
  return v;
}
void PODSR::fromJson(const Json::Value& v, const std::string& dir_name)
{
  (void)dir_name;
  starkit_utils::tryRead(v, "nb_images", &nb_images);
  starkit_utils::tryRead(v, "training_tags_per_image", &training_tags_per_image);
  starkit_utils::tryRead(v, "validation_tags_per_image", &validation_tags_per_image);
  starkit_utils::tryRead(v, "verbose", &verbose);
}

}  // namespace starkit_model_learning
