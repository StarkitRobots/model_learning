#include "rhoban_model_learning/model_prior_factory.h"

#include "rhoban_model_learning/default_prior.h"

namespace rhoban_model_learning
{

ModelPriorFactory::ModelPriorFactory() {
  registerBuilder("DefaultPrior",[]() { return std::unique_ptr<ModelPrior>(new DefaultPrior); });
}

}
