#include "starkit_model_learning/model_prior_factory.h"

#include "starkit_model_learning/composite_prior.h"
#include "starkit_model_learning/default_prior.h"

namespace starkit_model_learning
{
ModelPriorFactory::ModelPriorFactory()
{
  registerBuilder("CompositePrior", []() { return std::unique_ptr<ModelPrior>(new CompositePrior); });
  registerBuilder("DefaultPrior", []() { return std::unique_ptr<ModelPrior>(new DefaultPrior); });
}

}  // namespace starkit_model_learning
