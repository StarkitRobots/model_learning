#include "rhoban_model_learning/humanoid_models/vision_correction_model.h"
#include "rhoban_model_learning/input_reader_factory.h"

#include "rhoban_model_learning/model_factory.h"

#include "rhoban_random/tools.h"

using namespace rhoban_model_learning;
using namespace rhoban_utils;

typedef VisionCorrectionModel::VisionInput VisionInput;

class Config : public rhoban_utils::JsonSerializable {
public:
  Config(){}

  std::string getClassName() const override {
    return "VisionCorrectionDebugConfig";
  }

  Json::Value toJson() const override {
    throw std::logic_error("Not implemented");
  }

  void fromJson(const Json::Value & v, const std::string & dir_name) override {
    models = ModelFactory().readMap(v, "models", dir_name);
  }

  /// All available models
  std::map<std::string, std::unique_ptr<Model>> models;
};

void compare(const std::string & model_name,
             const VisionCorrectionModel & m,
             const SampleVector & samples,
             std::ostream & out) {
  for (const auto & s : samples) {
    const VisionInput & input = dynamic_cast<const VisionInput &>(s->getInput());
    int tag_id = input.data("tag_id");
    Eigen::Vector2d obs(input.data("pixel_x_uncorrected"), input.data("pixel_y_uncorrected"));
    Eigen::Vector2d pred = m.predictObservation(input, nullptr);
    Eigen::Vector2d error = obs - pred;
    out << model_name << "," << tag_id << "," << error(0) << "," << error(1) << ","
        << obs(0) << "," << obs(1) << "," << pred(0) << "," << pred(1)<< std::endl;
  }
}

int main(int argc, char ** argv) {
  if (argc != 4) {
    std::cerr << "Usage: " << argv[0] << " <vision_correction_debug.json> <input_reader.json> <data_file>"
              << std::endl;
    exit(EXIT_FAILURE);
  }

  Config conf;
  conf.loadFile(argv[1]);

  // Reading input reader from Json
  std::unique_ptr<InputReader> input_reader = InputReaderFactory().buildFromJsonFile(argv[2]);

  // Analyze data
  std::default_random_engine engine = rhoban_random::getRandomEngine();
  DataSet data = input_reader->extractSamples(argv[3], &engine);

  std::ofstream csv_file("debug.csv");

  csv_file << "model,tagId,errX,errY,obsX,obsY,predX,predY" << std::endl;

  //
  for (const auto & model_pair : conf.models) {
      std::string model_name = model_pair.first;
      const VisionCorrectionModel & model =
          dynamic_cast<const VisionCorrectionModel &> (*(model_pair.second));

      compare(model_name, model, data.training_set, csv_file);
      compare(model_name, model, data.validation_set, csv_file);
  }
}
