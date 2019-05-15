#include "starkit_model_learning/tags/aruco_collection.h"

#include <starkit_utils/util.h>

namespace starkit_model_learning
{
ArucoCollection::ArucoCollection() : CompositeModel()
{
}

ArucoCollection::ArucoCollection(const ArucoCollection& other) : CompositeModel(other)
{
}

std::map<int, ArucoTag> ArucoCollection::getMarkers() const
{
  std::map<int, ArucoTag> all_tags;
  for (const auto& model_entry : models)
  {
    const TagsCollection& tag_collection = dynamic_cast<const TagsCollection&>(*model_entry.second);
    for (const auto& tag_entry : tag_collection.getMarkers())
    {
      int tag_id = tag_entry.first;
      const ArucoTag& tag = tag_entry.second;
      if (all_tags.count(tag_id) > 0)
      {
        throw std::logic_error(DEBUG_INFO + " marker " + std::to_string(tag_id) + " found twice");
      }
      all_tags[tag_id] = tag;
    }
  }
  return all_tags;
}

void ArucoCollection::fromJson(const Json::Value& v, const std::string& dir_path)
{
  CompositeModel::fromJson(v, dir_path);
  // Checking types
  for (const auto& entry : models)
  {
    try
    {
      (void)dynamic_cast<const TagsCollection&>(*entry.second);
      (void)dynamic_cast<const Model&>(*entry.second);
    }
    catch (const std::bad_cast& exc)
    {
      throw std::runtime_error(DEBUG_INFO + "Invalid type for model '" + entry.first +
                               "'. Expecting an object of type TagsCollection and Model");
    }
  }
}

std::string ArucoCollection::getClassName() const
{
  return "ArucoCollection";
}

std::unique_ptr<Model> ArucoCollection::clone() const
{
  return std::unique_ptr<Model>(new ArucoCollection(*this));
}

}  // namespace starkit_model_learning
