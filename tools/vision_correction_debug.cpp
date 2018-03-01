#include "rhoban_model_learning/humanoid_models/vision_correction_model.h"
#include "rhoban_model_learning/input_reader_factory.h"

#include "rhoban_random/tools.h"

using namespace rhoban_model_learning;

typedef VisionCorrectionModel::VisionInput VisionInput;

void compare(const VisionCorrectionModel & m,
             const SampleVector & samples,
             std::ostream & out) {
  for (const auto & s : samples) {
    const VisionInput & input = dynamic_cast<const VisionInput &>(s->getInput());
    int tag_id = input.data("tag_id");
    Eigen::Vector2d img_px = m.leph2Img(Eigen::Vector2d(input.data("pixel_x"),
                                                        input.data("pixel_y")));
    Eigen::Vector2d pred_leph = m.predictObservation(input, nullptr);
    Eigen::Vector2d pred_px = m.leph2Img(pred_leph);
    Eigen::Vector2d error = img_px - pred_px;
    out << tag_id << "," << error(0) << "," << error(1) << std::endl;
  }
}

int main(int argc, char ** argv) {
  if (argc < 4) {
    std::cerr << "Usage: " << argv[0] << " <vision_correction_model.json> <input_reader.json> <data_file>"
              << std::endl;
    exit(EXIT_FAILURE);
  }

  // Reading model learner from Json
  VisionCorrectionModel model;
  model.loadFile(argv[1]);
  // Reading input reader from Json
  std::unique_ptr<InputReader> input_reader = InputReaderFactory().buildFromJsonFile(argv[2]);

  // Analyze data
  std::default_random_engine engine = rhoban_random::getRandomEngine();
  DataSet data = input_reader->extractSamples(argv[3], &engine);

  std::ofstream csv_file("debug.csv");

  csv_file << "tagId,errX,errY" << std::endl;

  //
  compare(model, data.training_set, csv_file);
  compare(model, data.validation_set, csv_file);

}
