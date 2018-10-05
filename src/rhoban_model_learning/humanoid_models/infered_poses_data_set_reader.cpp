#include "rhoban_model_learning/humanoid_models/infered_poses_data_set_reader.h"
#include "rhoban_model_learning/humanoid_models/infered_poses_input.h"

#include <rhoban_random/tools.h>
#include <rhoban_utils/tables/string_table.h>
#include <rhoban_utils/util.h>

#include <Eigen/Core>

#include <iostream>

namespace rhoban_model_learning
{

using rhoban_utils::StringTable;

typedef InferedPosesDataSetReader IPDSR;
typedef InferedPosesInput IPI;

IPDSR::InferedPosesDataSetReader()
  : nb_training_images(-1), nb_validation_images(-1), nb_tags_to_infer_pose(-1), nb_tags_per_image(-1), verbose(false)
{
}

DataSet IPDSR::extractSamples(const std::string & file_path,
                              std::default_random_engine * engine) const
{
  StringTable data = StringTable::buildFromFile(file_path);

  std::map<int, std::vector<Eigen::Vector3d>> raw_datas_by_image;
  for (size_t row = 0; row < data.nbRows(); row++) {
    std::map<std::string, std::string> row_content = data.getRow(row);
    int image_id = std::stoi(row_content.at("image_id"));
    int marker_id = std::stoi(row_content.at("marker_id"));
    double pixel_x = std::stod(row_content.at("pixel_x"));
    double pixel_y = std::stod(row_content.at("pixel_y"));

    raw_datas_by_image[image_id].push_back(Eigen::Vector3d(marker_id, pixel_x, pixel_y));
  }
  
  // Get valid images indices
  int nb_images = nb_training_images + nb_validation_images;
  if (verbose) {
    std::cout << "Filtering images" << std::endl;
  }
  std::vector<int> images_indices;
  for (const auto & pair : raw_datas_by_image) {
    int image_id = pair.first;
    int nb_datas = pair.second.size();
    if (nb_datas >= nb_tags_to_infer_pose + nb_tags_per_image) {
      images_indices.push_back(pair.first);
    } else if (verbose) {
      std::cout << "\tIgnoring image " << image_id << " because it has only "
                << nb_datas << " valid samples" << std::endl;
    }
  }
  if (images_indices.size() < (size_t) (nb_images)) {
    throw std::runtime_error(DEBUG_INFO + " not enough images with enough tags ("
                             + std::to_string(images_indices.size()) +" images available, "
                             + std::to_string(nb_images) + " required)");
  }

  // Choosing the images used for training and validation
  std::vector<size_t> chosen_image_indices = rhoban_random::getKDistinctFromN(
      nb_images, images_indices.size(), engine
  );

  // Choosing the tags used to infer the pose
  std::map<int, std::vector<Sample>> samples_by_image;

  for (size_t idx : chosen_image_indices) {
    const std::vector<Eigen::Vector3d> & raw_datas_of_image = raw_datas_by_image[idx];
    std::vector<size_t> set_sizes = { (size_t)nb_tags_to_infer_pose,
                                      (size_t)nb_tags_per_image };
    std::vector<std::vector<size_t>> tags_indices =
      rhoban_random::splitIndices(raw_datas_of_image.size(), set_sizes, engine);
    std::vector<Eigen::Vector3d> tags_to_infer;
    for (size_t tag_to_infer_id : tags_indices[0]) {
      tags_to_infer.push_back(raw_datas_of_image[tag_to_infer_id]);
    }
    for (size_t validation_idx : tags_indices[1]) {
      Eigen::Vector3d tag = raw_datas_of_image[validation_idx];
      samples_by_image[idx].push_back(
        Sample(
          std::unique_ptr<Input>( new IPI(tags_to_infer, tag[0])),
          Eigen::Vector2d(tag[1],tag[2])
        )
      );
    }
  }

  // Choosing the data set by separating the images for training and validation
  DataSet data_set;
  std::vector<size_t> set_sizes = {(size_t)nb_training_images, (size_t)nb_validation_images};
  std::vector<std::vector<size_t>> training_and_validation_image_indices_separation =
    rhoban_random::splitIndices(nb_images, set_sizes, engine);
  for (size_t idx : training_and_validation_image_indices_separation[0]){
    for (const Sample & sample : samples_by_image[idx]){
      data_set.training_set.push_back(sample.clone());
    }
  }
  for (size_t idx : training_and_validation_image_indices_separation[1]){
    for (const Sample & sample : samples_by_image[idx]){
      data_set.validation_set.push_back(sample.clone());
    }
  }

  return data_set;
}

std::string IPDSR::getClassName() const {
  return "CalibrationDataSetReader";
}

Json::Value IPDSR::toJson() const {
  Json::Value  v;
  v["nb_training_images"   ] = nb_training_images;
  v["nb_validation_images" ] = nb_validation_images;
  v["nb_tags_per_image"       ] = nb_tags_per_image;
  v["nb_tags_to_infer_pose"] = nb_tags_to_infer_pose;
  v["verbose"              ] = verbose;
  return v;
}
void IPDSR::fromJson(const Json::Value & v, 
                     const std::string & dir_name) {
  (void) dir_name;
  rhoban_utils::tryRead(v,"nb_training_images"   , &nb_training_images);
  rhoban_utils::tryRead(v,"nb_validation_images" , &nb_validation_images);
  rhoban_utils::tryRead(v,"nb_tags_per_image"       , &nb_tags_per_image);
  rhoban_utils::tryRead(v,"nb_tags_to_infer_pose", &nb_tags_to_infer_pose);
  rhoban_utils::tryRead(v,"verbose"              , &verbose);

  if (2 < nb_tags_to_infer_pose) {
    throw std::runtime_error(DEBUG_INFO + " the number of tags ("
                             + std::to_string(nb_tags_to_infer_pose) +
                             " should be bigger than 3.");
  }
}

}
