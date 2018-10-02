#pragma once

#include "rhoban_model_learning/composite_model.h"
#include "rhoban_model_learning/tags/tags_collection.h"

namespace rhoban_model_learning
{

/// Represent multiple cubes, thus allowing to get all 
class ArucoCollection : public CompositeModel, public TagsCollection
{
public:
  ArucoCollection();
  ArucoCollection(const ArucoCollection & other);

  std::map<int, ArucoTag> getMarkers() const override;
  virtual void fromJson(const Json::Value & v, const std::string & dir_path) override;

  virtual std::string getClassName() const override;
  virtual std::unique_ptr<Model> clone() const override;
};

}
