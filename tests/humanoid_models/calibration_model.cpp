#include "rhoban_model_learning/humanoid_models/calibration_model.h"

#include <gtest/gtest.h>

using namespace std;
using namespace rhoban_model_learning;

TEST(calibrationModel, getIndicesFromNames)
{
  CalibrationModel model;
  std::vector<std::string> names = {"camera:centerX", "imuOffset"};
  std::vector<int> expected_indices = {2,12,13,14};
  std::set<int> received_indices = model.getIndicesFromNames(names);
  EXPECT_EQ(expected_indices.size(), received_indices.size());
  for (int index : expected_indices) {
    std::cout << "index: "  << index << std::endl;
    EXPECT_EQ(1,(int)received_indices.count(index));
  }
  for (int index : received_indices) {
    std::cout << "received index: "  << index << std::endl;
  }
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
