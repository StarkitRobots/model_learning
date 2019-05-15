#include "starkit_model_learning/model.h"
#include "starkit_model_learning/model_factory.h"

#include "starkit_utils/util.h"

using namespace starkit_utils;

namespace starkit_model_learning
{
Model::Model()
{
}

int Model::getParametersSize() const
{
  return getParameters().rows();
}

Eigen::VectorXd Model::getParameters(const std::set<int>& used_indices) const
{
  Eigen::VectorXd all_parameters = getParameters();
  Eigen::VectorXd used_parameters(used_indices.size());
  int used_idx = 0;
  for (int idx : used_indices)
  {
    used_parameters(used_idx) = all_parameters[idx];
    used_idx++;
  }
  return used_parameters;
}

void Model::setParameters(const Eigen::VectorXd& new_params, const std::set<int>& used_indices)
{
  Eigen::VectorXd all_parameters = getParameters();
  int used_idx = 0;
  for (int idx : used_indices)
  {
    all_parameters(idx) = new_params(used_idx);
    used_idx++;
  }
  setParameters(all_parameters);
}

std::vector<std::string> Model::getParametersNames() const
{
  int nb_parameters = getParametersSize();
  std::vector<std::string> result;
  for (int idx = 0; idx < nb_parameters; idx++)
  {
    result.push_back("param" + std::to_string(idx + 1));
  }
  return result;
}

std::vector<std::string> Model::getParametersNames(const std::set<int>& used_indices) const
{
  std::vector<std::string> all_names = getParametersNames();
  std::vector<std::string> used_names(used_indices.size());
  int used_idx = 0;
  for (int idx : used_indices)
  {
    used_names[used_idx] = all_names[idx];
    used_idx++;
  }
  return used_names;
}

std::set<int> Model::getIndicesFromName(const std::string& name) const
{
  std::vector<std::string> parameters_names = getParametersNames();
  for (size_t idx = 0; idx < parameters_names.size(); idx++)
  {
    if (name == parameters_names[idx])
    {
      return { (int)idx };
    }
  }
  if (name == "all")
  {
    std::set<int> parameters;
    for (int i = 0; i < getParametersSize(); i++)
    {
      parameters.insert(i);
    }
    return parameters;
  }
  throw std::out_of_range(DEBUG_INFO + " unknown name '" + name + "'");
}

std::set<int> Model::getIndicesFromNames(const std::vector<std::string>& names) const
{
  std::set<int> result;
  for (const std::string& name : names)
  {
    std::set<int> name_indices = getIndicesFromName(name);
    for (int index : name_indices)
    {
      if (result.count(index) == 0)
      {
        result.insert(index);
      }
      else
      {
        throw std::runtime_error(DEBUG_INFO + " duplicated entry for index " + std::to_string(index));
      }
    }
  }
  return result;
}

std::unique_ptr<Model> Model::clone() const
{
  Json::Value v = toJson();
  std::unique_ptr<Model> other = ModelFactory().build(getClassName());
  other->fromJson(v, "./");
  return other;
}

}  // namespace starkit_model_learning
