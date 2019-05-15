#pragma once

#include "starkit_model_learning/tags/aruco_tag.h"

#include <starkit_utils/serialization/json_serializable.h>

namespace starkit_model_learning
{
class TagsCollection
{
public:
  /// Return a dictionary containing all the aruco tags of the collection with
  /// their position in world referential
  virtual std::map<int, ArucoTag> getMarkers() const = 0;
};

}  // namespace starkit_model_learning
