#include "rhoban_model_learning/tags/tags_sheet.h"

#include <iostream>

namespace rhoban_model_learning
{

TagsSheet::TagsSheet()
  : sheet_pose(), marker_size(0.09), dx(0.12), dy(0.10),
    markers_ids({0,1,2,3,4,5})
{
}

TagsSheet::TagsSheet(double marker_size,
                     double dx,
                     double dy,
                     const PoseModel & sheet_pose,
                     const std::vector<int> & markers_ids)
  : sheet_pose(sheet_pose), marker_size(marker_size), dx(dx), dy(dy),
    markers_ids(markers_ids)
{
}

std::map<int, ArucoTag> TagsSheet::getMarkers() const
{
  std::map<int, ArucoTag> markers;
  for (int idx = 0; idx < 6; idx ++) {
    int marker_id = markers_ids[idx];
    double coeff_x = (idx % 2 == 0) ? -0.5 : 0.5;
    double coeff_y = std::floor(idx / 2) - 1;
    Eigen::Vector3d marker_center_self(dx * coeff_x, dy * coeff_y, 0);
    Eigen::Vector3d marker_center = sheet_pose.getPosFromSelf(marker_center_self);
    markers[marker_id] = ArucoTag(marker_id, marker_size, marker_center,
                                  sheet_pose.orientation);
  }
  return markers;
}


int TagsSheet::getParametersSize() const {
  return sheet_pose.getParametersSize();
}

Eigen::VectorXd TagsSheet::getParameters() const {
  return sheet_pose.getParameters();
}

void TagsSheet::setParameters(const Eigen::VectorXd & new_params) {
  sheet_pose.setParameters(new_params);
}

std::vector<std::string> TagsSheet::getParametersNames() const {
  return sheet_pose.getParametersNames();
}

Json::Value TagsSheet::toJson() const {
  Json::Value v;
  v["marker_size" ] = marker_size;
  v["markers_ids" ] = rhoban_utils::vector2Json(markers_ids);
  v["dx"          ] = dx;
  v["dy"          ] = dy;
  v["sheet_pose"] = sheet_pose.toJson();
  return v;
}

void TagsSheet::fromJson(const Json::Value & v, const std::string & dir_path) {
  (void) dir_path;
  rhoban_utils::tryRead(v,"marker_size" , &marker_size );
  rhoban_utils::tryRead(v,"dx"          , &dx          );
  rhoban_utils::tryRead(v,"dy"          , &dy          );
  rhoban_utils::tryReadVector(v,"markers_ids" , &markers_ids );
  if (v.isObject() && v.isMember("sheet_pose")) {
    sheet_pose.fromJson(v["sheet_pose"], dir_path);
  }
}

std::string TagsSheet::getClassName() const {
  return "TagsSheet";
}

}
