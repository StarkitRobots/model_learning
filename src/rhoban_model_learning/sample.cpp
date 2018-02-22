#include "rhoban_model_learning/sample.h"

#include "rhoban_random/tools.h"

#include <iostream>

namespace rhoban_model_learning
{


Sample::Sample(std::unique_ptr<Input> input_,
               const Eigen::VectorXd & observation_) :
  input(std::move(input_)), observation(observation_)
{
}

const Input & Sample::getInput() const
{
  return *input;
}


const Eigen::VectorXd & Sample::getObservation() const
{
  return observation;
}

std::unique_ptr<Sample> Sample::clone() const
{
  return std::unique_ptr<Sample>(new Sample(input->clone(), observation));
}

DataSet splitSamples(const SampleVector & samples, double validation_ratio,
                     std::default_random_engine * engine)
{
  return splitSamples(samples, (int)ceil(validation_ratio * samples.size()), engine);
}

DataSet splitSamples(const SampleVector & samples, int nb_validation_samples,
                     std::default_random_engine * engine)
{
  DataSet result;
  std::vector<size_t> validation_indices;
  validation_indices = rhoban_random::getKDistinctFromN(nb_validation_samples, samples.size(),
                                                        engine);
  std::sort(validation_indices.begin(), validation_indices.end());
  int validation_idx = 0;
  for (size_t i=0; i< samples.size(); i++) {
    if (i == validation_indices[validation_idx]) {
      result.validation_set.push_back(samples[i]->clone());
      validation_idx++;
    } else {
      result.training_set.push_back(samples[i]->clone());
    }
  }
  return result;
}


}
