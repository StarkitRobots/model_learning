#include "rhoban_model_learning/composite_model.h"

#include "rhoban_model_learning/model_factory.h"

namespace rhoban_model_learning
{

CompositeModel::CompositeModel() : Model()
{}

CompositeModel::CompositeModel(const CompositeModel & other)
  : Model()
{
  for (const auto & entry : other.models) {
    models[entry.first] = entry.second->clone();
  }
}

int CompositeModel::getParametersSize() const {
  int count = 0;
  for (const auto & entry : models) {
    count += entry.second->getParametersSize();
  }
  return count;
}

Eigen::VectorXd CompositeModel::getParameters() const {
  Eigen::VectorXd parameters(getParametersSize());
  int idx = 0;
  for (const auto & entry : models) {
    Eigen::VectorXd entry_parameters = entry.second->getParameters();
    int nb_entry_params = entry_parameters.size();
    parameters.segment(idx, nb_entry_params) = entry_parameters;
    idx += nb_entry_params;
  }
  return parameters;
}

void CompositeModel::setParameters(const Eigen::VectorXd & new_params) {
  int idx = 0;
  for (const auto & entry : models) {
    int model_count = entry.second->getParametersSize();
    entry.second->setParameters(new_params.segment(idx, model_count));
    idx += model_count;
  }
}

std::vector<std::string> CompositeModel::getParametersNames() const {
  std::vector<std::string> names;
  for (const auto & entry : models) {
    std::string model_name = entry.first;
    std::vector<std::string> entry_names = entry.second->getParametersNames();
    for (const std::string & param_name : entry_names) {
      names.push_back(model_name + ":" + param_name);
    }
  }
  return names;
}
  
Json::Value CompositeModel::toJson() const {
  Json::Value v;
  for (const auto & entry : models) {
    v["models"][entry.first] = entry.second->toJson();
  }
  return v;
}


void CompositeModel::fromJson(const Json::Value & v, const std::string & dir_name) {
  models = ModelFactory().readMap(v, "models", dir_name);
}


}
