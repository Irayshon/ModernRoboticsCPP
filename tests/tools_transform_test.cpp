#include "DallE/tools.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

TEST(ToolsTransformTest, VecSo3RoundTrip) {
  Eigen::Vector3d w(1, 2, 3);
  auto so3 = DallE::Tools::VecToso3(w);
  auto w2 = DallE::Tools::so3ToVec(so3);
  EXPECT_NEAR((w - w2).norm(), 0.0, 1e-9);
}

TEST(ToolsTransformTest, MatrixExp3MatrixLog3RoundTrip) {
  Eigen::Vector3d w(1.0, -0.5, 2.0);
  w.normalize();
  Eigen::Matrix3d so3 = DallE::Tools::VecToso3(w * 0.4);
  Eigen::Matrix3d R = DallE::Tools::MatrixExp3(so3);
  Eigen::Matrix3d R_round = DallE::Tools::MatrixExp3(DallE::Tools::MatrixLog3(R));
  EXPECT_NEAR((R - R_round).norm(), 0.0, 1e-9);
}

TEST(ToolsTransformTest, MatrixExp6MatrixLog6RoundTrip) {
  Eigen::VectorXd V(6);
  V << 0.3, -0.2, 0.1, 0.5, -0.1, 0.2;
  Eigen::MatrixXd se3 = DallE::Tools::VecTose3(V);
  Eigen::MatrixXd T = DallE::Tools::MatrixExp6(se3);
  Eigen::MatrixXd T_round = DallE::Tools::MatrixExp6(DallE::Tools::MatrixLog6(T));
  EXPECT_NEAR((T - T_round).norm(), 0.0, 1e-9);
}

TEST(ToolsTransformTest, TransInvProducesIdentity) {
  Eigen::Vector3d w(0.2, 0.4, -0.1);
  Eigen::Matrix3d so3 = DallE::Tools::VecToso3(w);
  Eigen::Matrix3d R = DallE::Tools::MatrixExp3(so3);
  Eigen::Vector3d p(0.1, -0.3, 0.2);
  Eigen::MatrixXd T = DallE::Tools::RpToTrans(R, p);
  Eigen::MatrixXd T_inv = DallE::Tools::TransInv(T);
  Eigen::MatrixXd I = T * T_inv;
  EXPECT_NEAR((I - Eigen::MatrixXd::Identity(4, 4)).norm(), 0.0, 1e-9);
}
