#include "DallE/tools.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

TEST(ToolsJacobianTest, JacobianShapes) {
  Eigen::MatrixXd S(6, 2);
  S << 0, 0,
       0, 0,
       1, 1,
       0, 0,
       0, 0,
       0, 0;
  Eigen::Vector2d theta(0.1, 0.2);
  auto Js = DallE::Tools::JacobianSpace(S, theta);
  EXPECT_EQ(Js.rows(), 6);
  EXPECT_EQ(Js.cols(), 2);
}

TEST(ToolsJacobianTest, ConditionNumberAndSingularity) {
  Eigen::Matrix2d J = Eigen::Matrix2d::Identity();
  EXPECT_NEAR(DallE::Tools::ConditionNumber(J), 1.0, 1e-9);
  EXPECT_FALSE(DallE::Tools::IsSingular(J));

  Eigen::Matrix2d J_bad = Eigen::Matrix2d::Identity();
  J_bad(1, 1) = 1e-8;
  EXPECT_GT(DallE::Tools::ConditionNumber(J_bad), 1e6);
  EXPECT_TRUE(DallE::Tools::IsSingular(J_bad));
}
