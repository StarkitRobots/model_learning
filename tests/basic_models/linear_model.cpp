#include <gtest/gtest.h>

#include "rhoban_model_learning/basic_models/linear_model.h"

#include "rhoban_model_learning/default_input.h"

using namespace rhoban_model_learning;

TEST(LinearModel, emptyBuilder) {
  LinearModel model;
  EXPECT_EQ(model.getParametersSize(), 2);//Empty model has only bias and std_dev
}

TEST(LinearModel, intBuilder) {
  LinearModel model(2);
  EXPECT_EQ(model.getParametersSize(), 4);
  LinearModel model2(4);
  EXPECT_EQ(model2.getParametersSize(), 6);
}

TEST(LinearModel, setGetParameters) {
  LinearModel model(2);
  Eigen::VectorXd set_params(4);
  set_params << -1, 2, -3, 4;
  model.setParameters(set_params);
  Eigen::VectorXd get_params = model.getParameters();
  EXPECT_EQ(set_params.rows(), get_params.rows());
  for (int row = 0; row < get_params.rows(); row++) {
    EXPECT_EQ(set_params(row), get_params(row));
  }
}

TEST(LinearModel, setInvalidStdDev) {
  try{
    LinearModel model(2);
    Eigen::VectorXd set_params(4);
    set_params << 0,0,0,-1;
    model.setParameters(set_params);
    EXPECT_TRUE(false);
  } catch (const std::invalid_argument & exc) {
  }
}

TEST(LinearModel, predictObservationDeterminist) {
  LinearModel model(2);
  Eigen::VectorXd params(4);
  params << 3,-2,2,1;
  model.setParameters(params);
  std::vector<Eigen::Vector2d> inputs =
    {Eigen::Vector2d(0,0), Eigen::Vector2d(2,0), Eigen::Vector2d(0,2)};
  std::vector<double> outputs = {2,8,-2};
  for (size_t i = 0; i < inputs.size(); i++){
    Eigen::VectorXd prediction = model.predictObservation(DefaultInput(inputs[i]), nullptr);
    EXPECT_DOUBLE_EQ(outputs[i],prediction(0));
  }
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
