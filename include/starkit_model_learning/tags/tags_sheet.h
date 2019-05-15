#pragma once

#include "starkit_model_learning/model.h"

#include "starkit_model_learning/humanoid_models/pose_model.h"
#include "starkit_model_learning/tags/aruco_tag.h"
#include "starkit_model_learning/tags/tags_collection.h"

namespace starkit_model_learning
{
/// Represent a rows*cols sheet of aruco markers with a rectangular pattern
///
/// e.g. a 3*2 sheet of aruco Tags
///           ----dx--->
///    |    id[0]     id[1]
///    |
///   dy
///    |
///   \ /   id[2]     id[3]
///
///
///
///         id[4]     id[5]
class TagsSheet : public Model, public TagsCollection
{
public:
  TagsSheet();
  TagsSheet(double marker_size, double dx, double dy, int cols, int rows, const PoseModel& sheet_pose,
            const std::vector<int>& markers_ids);
  TagsSheet(const TagsSheet& other);

  /// Update the pose of the tags sheet
  void setPose(const Eigen::Vector3d& pos, const Eigen::Quaterniond& orientation);

  std::map<int, ArucoTag> getMarkers() const override;

  int getParametersSize() const override;

  Eigen::VectorXd getParameters() const override;
  void setParameters(const Eigen::VectorXd& new_params) override;
  std::vector<std::string> getParametersNames() const override;

  virtual Json::Value toJson() const override;
  virtual void fromJson(const Json::Value& v, const std::string& dir_path) override;
  virtual std::string getClassName() const override;

private:
  /// center and orientation of the sheet
  /// - center: sheet_center
  /// - orientation: x-axis -> left of the sheet, y-axis -> bottom of the sheet
  /// This is the only customizable parameters, other are supposed to be 'known'
  PoseModel sheet_pose;

  /// Size of an aruco marker (all markers of a sheet have the same size) [m]
  double marker_size;

  /// Spacing between two columns of tags (center to center) [m]
  double dx;

  /// Delta between two lines of tags (center to center) [m]
  double dy;

  /// Number of columns in the sheet
  int cols;

  /// Number of rows in the sheet
  int rows;

  /// The list of the marker ids
  std::vector<int> markers_ids;
};

}  // namespace starkit_model_learning
