#include "sample.h"

namespace rosban_model_learning
{

const Eigen::VectorXd & Sample::getObservation() const
{
  return observation;
}

}
