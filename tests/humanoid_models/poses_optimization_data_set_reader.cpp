#include "rhoban_model_learning/humanoid_models/poses_optimization_data_set_reader.h"

#include <gtest/gtest.h>

using namespace std;
using namespace rhoban_model_learning;

typedef PosesOptimizationDataSetReader PODSR;

string getAbsoluteTestFilePrefix() {
    string filePath = __FILE__;
    string currentDirPath = filePath.substr(0, filePath.rfind("/"));
    return currentDirPath + "/../ressources/humanoid_models/";
}

TEST(PosesOptimizationDataSetReader, artificialFile)
{
  PODSR reader;
  std::default_random_engine engine;
  reader.loadFile(getAbsoluteTestFilePrefix()+"poses_optimization_data_set_reader.json");
  DataSet data = reader.extractSamples(getAbsoluteTestFilePrefix() + "poses_optimization_data.csv",
                                       &engine);
  EXPECT_EQ((size_t)24, data.training_set.size());
  EXPECT_EQ((size_t)8, data.validation_set.size());
}
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

