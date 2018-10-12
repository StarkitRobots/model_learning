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

const Model & CompositeModel::getModel(const std::string & name) const {
  return *models.at(name);
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
  int nb_parameters_received = new_params.rows();
  int nb_parameters_expected = getParametersSize();
  if (nb_parameters_received != nb_parameters_expected) {
    throw std::runtime_error(DEBUG_INFO + " invalid number of parameters: "
                             + std::to_string(nb_parameters_received) + " received,"
                             + std::to_string(nb_parameters_expected) + " expected");
  }
  int idx = 0;
  for (const auto & entry : models) {
    int model_count = entry.second->getParametersSize();
    const Eigen::VectorXd & sub_model_params = new_params.segment(idx, model_count);
    std::cout << "params for model '" << entry.first << "' "
              << sub_model_params.transpose() << std::endl;
    entry.second->setParameters(sub_model_params);
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

std::set<int> CompositeModel::getIndicesFromName(const std::string & name) const {
  size_t separator_index = name.find(':');
  if (separator_index == std::string::npos) {
    // No separator found -> delegate to default implementation
    return Model::getIndicesFromName(name);
  }
  std::string prefix = name.substr(0, separator_index);
  std::string suffix = name.substr(separator_index + 1, name.size() - separator_index - 1);
  if (models.count(prefix) == 0) {
    throw std::out_of_range(DEBUG_INFO + " no model named '" + prefix + "'");
  }
  return models.at(prefix)->getIndicesFromName(suffix);
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
