#include "my_modern_robotics/tools.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

TEST(ToolsTransformTest, VecSo3RoundTrip) {
  Eigen::Vector3d w(1, 2, 3);
  auto so3 = mymr::Tools::VecToso3(w);
  auto w2 = mymr::Tools::so3ToVec(so3);
  EXPECT_NEAR((w - w2).norm(), 0.0, 1e-9);
}
