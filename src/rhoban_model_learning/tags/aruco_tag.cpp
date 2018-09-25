#include "rhoban_model_learning/tags/aruco_tag.h"

namespace rhoban_model_learning
{

ArucoTag::ArucoTag()
  : ArucoTag(-1, 0.1, Eigen::Vector3d(0,0,0),
             Eigen::Quaterniond(Eigen::Vector4d(1,0,0,0)))
{
}

ArucoTag::ArucoTag(int marker_id, double marker_size,
                   const Eigen::Vector3d & marker_center,
                   const Eigen::Quaterniond & orientation)
  : marker_id(marker_id),
    marker_size(marker_size),
    marker_center(marker_center),
    orientation(orientation)
{
}

}
