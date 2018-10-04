#include "rhoban_model_learning/humanoid_models/infered_poses_input.h"

namespace rhoban_model_learning
{

InferedPosesInput::InferedPosesInput() : nb_to_infer(-1), aruco_id(-1)
{
}

InferedPosesInput::InferedPosesInput(
    int nb_to_infer_,
    std::vector<Eigen::Vector3d> tags_to_infer_,
    int aruco_id_
) : nb_to_infer(nb_to_infer_), tags_to_infer(tags_to_infer_), aruco_id(aruco_id_)
{
}

InferedPosesInput::InferedPosesInput(const InferedPosesInput& other)
  : nb_to_infer(other.nb_to_infer), tags_to_infer(other.tags_to_infer),
     aruco_id(other.aruco_id)
{
}

std::unique_ptr<Input> InferedPosesInput::clone() const {
  return std::unique_ptr<Input>(new InferedPosesInput(*this));
}

}
