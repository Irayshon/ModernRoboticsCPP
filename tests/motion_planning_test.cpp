#include "DallE/motion_planning.h"

#include <gtest/gtest.h>

#include <type_traits>

TEST(MotionPlanningTest, Placeholder) {
  static_assert(std::is_default_constructible<DallE::MotionPlanning>::value,
                "MotionPlanning should be default-constructible");
  DallE::MotionPlanning plan;
  (void)plan;
}
