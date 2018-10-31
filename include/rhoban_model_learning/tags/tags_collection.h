#pragma once

#include "rhoban_model_learning/tags/aruco_tag.h"

#include <rhoban_utils/serialization/json_serializable.h>

namespace rhoban_model_learning
{

class TagsCollection {
public:
  /// Return a dictionary containing all the aruco tags of the collection with
  /// their position in world referential
  virtual std::map<int, ArucoTag> getMarkers() const = 0;
};

}
