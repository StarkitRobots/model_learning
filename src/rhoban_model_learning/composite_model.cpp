#include "rhoban_model_learning/composite_model.h"

namespace rhoban_model_learning
{

CompositeModel::CompositeModel() {}

CompositeModel::CompositeModel(const CompositeModel & other) {
  for (const auto & entry : other.models) {
    models[entry.first] = entry.second->clone();
  }
}

int CompositeModel::getParametersSize() const {
  int count = 0;
  for (const auto & entry : other.models) {
    count += entry.second->getParametersSize();
  }
  return count;
}

Eigen::VectorXd CompositeModel::getParameters() const {
  Eigen::VectorXd parameters(getParametersSize());
  int idx = 0;
  for (const auto & entry : other.models) {
    Eigen::VectorXd entry_parameters = entry.second->getParameters();
    int nb_entry_params = entry_parameters.size();
    parameters.segment(idx, nb_entry_params) = entry_parameters;
    idx += nb_entry_params;
  }
  return parameters;
}

void CompositeModel::setParameters(const Eigen::VectorXd & new_params) {}
std::vector<std::string> CompositeModel::getParametersNames() const {}
  
Json::Value CompositeModel::toJson() const {}
void CompositeModel::fromJson(const Json::Value & v, const std::string & dir_name) {}
virtual std::unique_ptr<Model> CompositeModel::clone() const {}


}
