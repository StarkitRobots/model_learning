#include "starkit_model_learning/default_data_set_reader.h"

#include "starkit_model_learning/default_input.h"

#include "starkit_utils/util.h"
#include "starkit_utils/tables/double_table.h"

namespace starkit_model_learning
{
using starkit_utils::DoubleTable;

DefaultDataSetReader::DefaultDataSetReader() : input_dim(-1), obs_dim(-1), validation_ratio(0.1)
{
}

DataSet DefaultDataSetReader::extractSamples(const std::string& file_path, std::default_random_engine* engine) const
{
  if (input_dim == -1 || obs_dim == -1)
  {
    throw std::logic_error(DEBUG_INFO + " either input_dim or obs_dim has not been initialized");
  }
  DoubleTable data = DoubleTable::buildFromFile(file_path);
  if (data.nbCols() < (size_t)(input_dim + obs_dim))
  {
    std::ostringstream err_msg;
    err_msg << "Not enough column in file '" << file_path << "': " << input_dim << "+" << obs_dim << "="
            << (input_dim + obs_dim) << " requested, but only " << data.nbCols() << " found";
    throw std::runtime_error(DEBUG_INFO + err_msg.str());
  }
  SampleVector samples;
  for (size_t row = 0; row < data.nbRows(); row++)
  {
    Eigen::VectorXd sample = data.getRow(row);
    std::unique_ptr<Input> input(new DefaultInput(sample.segment(0, input_dim)));
    Eigen::VectorXd obs = sample.segment(input_dim, obs_dim);
    samples.push_back(std::unique_ptr<Sample>(new Sample(std::move(input), obs)));
  }
  return splitSamples(samples, validation_ratio, engine);
}

std::string DefaultDataSetReader::getClassName() const
{
  return "DefaultDataSetReader";
}

Json::Value DefaultDataSetReader::toJson() const
{
  Json::Value v;
  v["input_dim"] = input_dim;
  v["obs_dim"] = obs_dim;
  v["validation_ratio"] = validation_ratio;
  return v;
}

void DefaultDataSetReader::fromJson(const Json::Value& v, const std::string& dir_name)
{
  (void)dir_name;
  starkit_utils::tryRead(v, "input_dim", &input_dim);
  starkit_utils::tryRead(v, "obs_dim", &obs_dim);
  starkit_utils::tryRead(v, "validation_ratio", &validation_ratio);
}

}  // namespace starkit_model_learning
