#include <gtest/gtest.h>

#include "starkit_model_learning/basic_models/linear_model.h"
#include "starkit_model_learning/basic_models/linear_predictor.h"

#include "starkit_model_learning/default_input.h"

using namespace starkit_model_learning;

TEST(LinearModel, emptyBuilder)
{
  LinearModel model;
  EXPECT_EQ(model.getParametersSize(), 2);  // Empty model has only bias and std_dev
}

TEST(LinearModel, intBuilder)
{
  LinearModel model(2);
  EXPECT_EQ(model.getParametersSize(), 4);
  LinearModel model2(4);
  EXPECT_EQ(model2.getParametersSize(), 6);
}

TEST(LinearModel, setGetParameters)
{
  LinearModel model(2);
  Eigen::VectorXd set_params(4);
  set_params << -1, 2, -3, 4;
  model.setParameters(set_params);
  Eigen::VectorXd get_params = model.getParameters();
  EXPECT_EQ(set_params.rows(), get_params.rows());
  for (int row = 0; row < get_params.rows(); row++)
  {
    EXPECT_EQ(set_params(row), get_params(row));
  }
}

TEST(LinearModel, setInvalidStdDev)
{
  try
  {
    LinearModel model(2);
    Eigen::VectorXd set_params(4);
    set_params << 0, 0, 0, -1;
    model.setParameters(set_params);
    EXPECT_TRUE(false);
  }
  catch (const std::invalid_argument& exc)
  {
  }
}

TEST(LinearModel, predictObservationDeterminist)
{
  LinearModel model(2);
  LinearPredictor predictor;
  Eigen::VectorXd params(4);
  params << 3, -2, 2, 1;
  model.setParameters(params);
  std::vector<Eigen::Vector2d> inputs = { Eigen::Vector2d(0, 0), Eigen::Vector2d(2, 0), Eigen::Vector2d(0, 2) };
  std::vector<double> outputs = { 2, 8, -2 };
  for (size_t i = 0; i < inputs.size(); i++)
  {
    Eigen::VectorXd prediction = predictor.predictObservation(DefaultInput(inputs[i]), model, nullptr);
    EXPECT_DOUBLE_EQ(outputs[i], prediction(0));
  }
}

TEST(LinearModel, getParametersNames)
{
  LinearModel model(2);
  std::vector<std::string> expected_names = { "param1", "param2", "param3", "param4" };
  std::vector<std::string> received_names = model.getParametersNames();
  ASSERT_EQ(received_names.size(), expected_names.size());
  for (size_t idx = 0; idx < received_names.size(); idx++)
  {
    EXPECT_EQ(received_names[idx], expected_names[idx]);
  }
}

TEST(LinearModel, getIndicesFromName)
{
  LinearModel model(2);
  std::vector<int> expected_indices = { 0, 1, 2, 3 };
  std::set<int> received_indices = model.getIndicesFromName("all");
  ASSERT_EQ(expected_indices.size(), received_indices.size());
  for (int index : expected_indices)
  {
    EXPECT_EQ(1, (int)received_indices.count(index));
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
