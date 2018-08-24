#include "rhoban_model_learning/tools.h"

namespace rhoban_model_learning {

Eigen::VectorXd extractSubset(const Eigen::VectorXd & vector,
                              const std::vector<int> & used_indices)
{
  Eigen::VectorXd result(used_indices.size());
  int i = 0;
  for (int idx : used_indices) {
    result(i++) = vector(idx);
  }
  return result;
}

}
