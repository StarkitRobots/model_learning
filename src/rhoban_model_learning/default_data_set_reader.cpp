#include "rhoban_model_learning/default_data_set_reader.h"

#include "rhoban_utils/tables/double_table.h"

namespace rhoban_model_learning {

DefaultDataSetReader::DefaultDataSetReader()
  : input_dim(-1), obs_dim(-1), validation_ratio(0.1)
{
}
  
DataSet DefaultDataSetReader::extractSamples(const std::string & file_path,
                                             std::default_random_engine * engine) const {
  if (input_dim == -1 || obs_dim == -1) {
    throw std::logic_error(DEBUG_INFO + " either input_dim or obs_dim has not been initialized");
  }
  DoubleTable data = DoubleTable::buildFromFile(file_path);
  if (data.nbCols() < input_dim + obs_dim) {
    std::ostringstream err_msg;
    err_msg << "Not enough column in file '" << file_path << "': " << input_dim << "+" << obs_dim
            << "=" << (input_dim + obs_dim) << " requested, but only " << data.nbCols() << " found";
    throw std::runtime_error(DEBUG_INFO + err_msg.str());
  }
  SampleVector samples;
  for (int row = 0; row < data.nbRows(); row++) {
    Eigen::VectorXd sample = data.getRow(row);
    std::unique_ptr<Input> input(new DefaultInput(data.segment(0, input_dim)));
    Eigen::VectorXd obs = data.segment(input_dim, obs_dim);
    samples.push_back(std::unique_ptr<Sample>(new Sample(std::move(input), obs)));
  }
  return splitSamples(samples, validation_ratio);
}

virtual std::string DefaultDataSetReader::getClassName() const {
}
Json::Value DefaultDataSetReader::toJson() const {
}
void DefaultDataSetReader::fromJson(const Json::Value & v, const std::string & dir_name) {
}

}
