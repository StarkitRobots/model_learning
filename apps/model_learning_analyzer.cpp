#include "rhoban_model_learning/model_learner.h"
#include "rhoban_model_learning/model_factory.h"
#include "rhoban_model_learning/input_reader_factory.h"

#include "rhoban_bbo/optimizer_factory.h"

#include "rhoban_utils/timing/time_stamp.h"

#include "rhoban_random/tools.h"

using namespace rhoban_bbo;
using namespace rhoban_model_learning;
using namespace rhoban_utils;

class Config : public rhoban_utils::JsonSerializable {
public:
  Config() : nb_runs(1) {}

  std::string getClassName() const override {
    return "ModelLearningAnalyzerConfig";
  }

  Json::Value toJson() const override {
    throw std::logic_error("Not implemented");
  }

  void fromJson(const Json::Value & v, const std::string & dir_name) override {
    rhoban_utils::tryRead(v, "nb_runs", &nb_runs);
    readers = InputReaderFactory().readMap(v, "readers", dir_name);
    models = ModelFactory().readMap(v, "models", dir_name);
    optimizers = OptimizerFactory().readMap(v, "optimizers", dir_name);
    space = rhoban_utils::read<Eigen::MatrixXd>(v, "space");
  }

  /// The number of runs for each configuration
  int nb_runs;

  /// All available readers
  std::map<std::string, std::unique_ptr<InputReader>> readers;

  /// All available models
  std::map<std::string, std::unique_ptr<Model>> models;

  /// All available optimizers
  std::map<std::string, std::unique_ptr<Optimizer>> optimizers;

  /// Parameters space (to be moved to models)
  Eigen::MatrixXd space;
};

int main(int argc, char ** argv) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <config.json> <data_file>" << std::endl;
    exit(EXIT_FAILURE);
  }

  Config conf;
  conf.loadFile(argv[1]);

  std::string data_path(argv[2]);

  std::default_random_engine engine = rhoban_random::getRandomEngine();

  // Start in the middle of the parameters
  Eigen::VectorXd initial_guess = (conf.space.col(0) + conf.space.col(1)) / 2;

  std::ofstream results_file("results.csv");

  // OPTIONAL: eventually, reduce number of columns if there is only 1 optimizer
  // or only 1 reader etc...
  results_file << "optimizer,model,reader,trainingScore,validationScore,learningTime" << std::endl;

  // For each model
  for (const auto & model_pair : conf.models) {
    std::string model_name = model_pair.first;
    const Model & model = *(model_pair.second);
    // Parameters file
    std::ostringstream name_oss;
    name_oss <<  model_name << "_parameters.csv";
    std::ofstream params_file(name_oss.str());
    std::vector<std::string> parameter_names = model.getParametersNames();
    params_file << "optimizer,reader";
    for (size_t idx = 0; idx < parameter_names.size(); idx++) {
      params_file << "," << parameter_names[idx];
    };
    params_file << std::endl;
    // For each optimizer
    for (const auto & optimizer_pair : conf.optimizers) {
      std::string optimizer_name = optimizer_pair.first;
      const Optimizer & optimizer = *(optimizer_pair.second);
      // Initialize the learning_model
      ModelLearner learner(model.clone(), optimizer.clone(), conf.space, initial_guess);
      // For each reader
      for (const auto & reader_pair : conf.readers) {
        std::string reader_name = reader_pair.first;
        const InputReader & reader = *(reader_pair.second);
        // Perform multiple runs
        // TODO: Average parameters among runs to save a .json config
        for (int run_id = 0; run_id < conf.nb_runs; run_id++) {
          // Extract data (splits between training and validation
          DataSet data = reader.extractSamples(data_path, &engine);
          // Learn model
          TimeStamp start = TimeStamp::now();
          ModelLearner::Result r = learner.learnParameters(data, &engine);
          TimeStamp end = TimeStamp::now();
          double learning_time = diffSec(start, end);
          // Writing scores
          results_file << optimizer_name << "," << model_name << ","
                       << reader_name << "," << r.training_log_likelihood << ","
                       << r.validation_log_likelihood << ","
                       << learning_time<< std::endl;
          // Writing params
          Eigen::VectorXd params = r.model->getParameters();
          params_file << optimizer_name << "," << reader_name;
          for (int i = 0; i < params.rows(); i++) {
            params_file << "," << params(i);
          }
          params_file << std::endl;
        }
      }
    }
  }
}
