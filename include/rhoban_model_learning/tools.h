#pragma once

#include <Eigen/Core>

#include <set>
#include <vector>

namespace rhoban_model_learning
{
/// With V the vector and I the used indices, return (V[I[0]],V[I[1]], ...)
Eigen::VectorXd extractSubset(const Eigen::VectorXd& vector, const std::vector<int>& used_indices);

Eigen::VectorXd extractSubset(const Eigen::VectorXd& vector, const std::set<int>& used_indices);

}  // namespace rhoban_model_learning
