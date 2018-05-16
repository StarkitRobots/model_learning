#include "rhoban_model_learning/modular_model.h"

#include "rhoban_model_learning/model_factory.h"

namespace rhoban_model_learning
{

ModularModel::ModularModel() {}
ModularModel::ModularModel(const ModularModel & other)
  : Model(other), used_indices(other.used_indices)
{}

ModularModel::ModularModel(int nb_dims)
{
  for (int i=0; i < nb_dims; i++) {
    used_indices.push_back(i);
  }
}

int ModularModel::getGlobalParametersCount() const {
  return getGlobalParameters().rows();
}

Eigen::VectorXd ModularModel::getParameters() const {
  Eigen::VectorXd global_parameters = getGlobalParameters();
  Eigen::VectorXd used_parameters(used_indices.size());
  int used_idx = 0;
  for (int idx : used_indices) {
    used_parameters(used_idx) = global_parameters[idx];
    used_idx++;
  }
  return used_parameters;
}

Eigen::MatrixXd ModularModel::getParametersSpace() const {
  Eigen::MatrixXd global_space = getGlobalParametersSpace();
  Eigen::MatrixXd used_space(used_indices.size(), 2);
  int used_idx = 0;
  for (int idx : used_indices) {
    used_space.row(used_idx) = global_space.row(idx);
    used_idx++;
  }
  return used_space;
}

void ModularModel::setParameters(const Eigen::VectorXd & new_params) {
  Eigen::VectorXd global_parameters = getGlobalParameters();
  int used_idx = 0;
  for (int idx : used_indices) {
    global_parameters(idx) = new_params(used_idx);
    used_idx++;
  }
  setGlobalParameters(global_parameters);
}

std::vector<std::string> ModularModel::getParametersNames() const {
  std::vector<std::string> global_names = getGlobalParametersNames();
  std::vector<std::string> used_names(used_indices.size());
  int used_idx = 0;
  for (int idx : used_indices) {
    used_names[used_idx] = global_names[idx];
    used_idx++;
  }
  return used_names;
}

Json::Value ModularModel::toJson() const {
  Json::Value v = Model::toJson();
  v["used_indices"] = rhoban_utils::vector2Json(used_indices);
  return v;
}

void ModularModel::fromJson(const Json::Value & v, const std::string & dir_name) {
  Model::fromJson(v, dir_name);
  rhoban_utils::tryReadVector<int>(v, "used_indices", &used_indices);
}

std::unique_ptr<Model> ModularModel::clone() const {
  ModularModel * copy = (ModularModel*) ModelFactory().build(getClassName()).release();
  copy->used_indices = used_indices;
  copy->setGlobalParameters(getGlobalParameters());
}

}
