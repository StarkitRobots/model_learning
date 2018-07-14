#include "rhoban_model_learning/humanoid_models/vision_correction_model.h"

#include <gtest/gtest.h>

using namespace std;
using namespace Leph;
using namespace rhoban_model_learning;

string getAbsoluteTestFilePath() {
    string filePath = __FILE__;
    string currentDirPath = filePath.substr(0, filePath.rfind("/"));
    return currentDirPath + "/../ressources/VCM.json";
}

/// JsonLoader will be used for 
TEST(jsonLoader, testSuccess)
{
  VisionCorrectionModel vcm;
  vcm.loadFile(getAbsoluteTestFilePath());
  EXPECT_FLOAT_EQ(vcm.getPxStddev(), 10);
  Eigen::Vector3d cam_offsets_rad = vcm.getCameraOffsetsRad();
  EXPECT_FLOAT_EQ(cam_offsets_rad(0),      M_PI / 180);
  EXPECT_FLOAT_EQ(cam_offsets_rad(1), -2 * M_PI / 180);
  EXPECT_FLOAT_EQ(cam_offsets_rad(2),  3 * M_PI / 180);

  const CameraModel & cameraModel = vcm.getCameraModel();
  
  EXPECT_EQ(cameraModel.getImgWidth(), 800);
  EXPECT_EQ(cameraModel.getImgHeight(), 600);
  EXPECT_EQ(cameraModel.getCenterX(), 400);
  EXPECT_EQ(cameraModel.getCenterY(), 300);
  EXPECT_EQ(cameraModel.getFocalX(), 600);
  EXPECT_EQ(cameraModel.getFocalY(), 600);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
