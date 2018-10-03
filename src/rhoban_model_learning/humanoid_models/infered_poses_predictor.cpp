#include "rhoban_model_learning/humanoid_models/infered_poses_predictor.h"

#include "rhoban_model_learning/humanoid_models/infered_poses_input.h"
#include "rhoban_model_learning/humanoid_models/infered_poses_model.h"

#include "rhoban_random/multivariate_gaussian.h"
#include "rhoban_random/tools.h"

#include <rhoban_utils/angle.h>
#include <rhoban_utils/util.h>

#include <opencv2/core/eigen.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include "Model/CameraModel.hpp"

namespace rhoban_model_learning
{

typedef InferedPosesPredictor IPP;
IPP::InferedPosesPredictor() {
}

Eigen::VectorXd IPP::predictObservation(const Input & raw_input,
                                        const Model & raw_model,
                                        std::default_random_engine * engine) const {
  // Can generate bad_cast error
  const InferedPosesInput & input = dynamic_cast<const InferedPosesInput &>(raw_input);
  const InferedPosesModel & model = dynamic_cast<const InferedPosesModel &>(raw_model);

  // First: infer matrix extrinsic parameters in world
  cv::Mat cameraMatrix = model.getCameraModel().getCameraMatrix();
  cv::Mat cameraDistortionCoeffs = model.getCameraModel().getDistortionCoeffs();

  std::vector<cv::Point3f> points3d;
  std::vector<cv::Point2i> points2d;
  for(const Eigen::Vector3d & tag_to_infer : input.tags_to_infer)
  {
    points2d.push_back(cv::Point2i(tag_to_infer(1), tag_to_infer(2)));
    points3d.push_back(eigen2CV(model.getTagPosition(tag_to_infer(0))));
  }

  cv::Mat t_vec;
  cv::Mat r_vec;
  cv::solvePnP(points3d, points2d, cameraMatrix, cameraDistortionCoeffs, t_vec, r_vec);

  /// Second: predict aruco_tag position in camera frame
  PoseModel camera_pose;
  Eigen::VectorXd params(6);
  params << t_vec.at<double>(0), t_vec.at<double>(1), t_vec.at<double>(3), r_vec.at<double>(0), r_vec.at<double>(1), r_vec.at<double>(2);

  camera_pose.setParameters(params);

  Eigen::Vector3d marker_pos_world = model.getTagPosition(input.aruco_id);
  Eigen::Vector3d marker_pos_camera = camera_pose.getPosInSelf(marker_pos_world);

  /// Fourth : predict aruco_tag position in camera image
  Eigen::Vector2d pixel = cv2Eigen(
    model.getCameraModel().getImgFromObject(eigen2CV(marker_pos_camera))
  );
  
  // Add noise if required
  if (engine != nullptr) {
    std::normal_distribution<double> observation_noise(0, model.getPxStddev());
    pixel(0) += observation_noise(*engine);
    pixel(1) += observation_noise(*engine);
  }
  return pixel;
}

double IPP::computeLogLikelihood(const Sample & sample,
                                 const Model & raw_model,
                                 std::default_random_engine * engine) const {
  const InferedPosesModel & model = dynamic_cast<const InferedPosesModel &>(raw_model);

  (void) engine;
  Eigen::Vector2d prediction = predictObservation(sample.getInput(), model, nullptr);
  Eigen::Vector2d observation = sample.getObservation();

  double px_stddev = model.getPxStddev();
  double px_var = px_stddev * px_stddev;
  Eigen::MatrixXd covar(2,2);
  covar << px_var, 0, 0, px_var;
  rhoban_random::MultivariateGaussian expected_distribution(prediction, covar);

  return expected_distribution.getLogLikelihood(observation);
}

Eigen::VectorXi IPP::getObservationsCircularity() const {
  return Eigen::VectorXi::Zero(2);
}

std::string IPP::getClassName() const {
  return "InferedPosesPredictor";
}

}
