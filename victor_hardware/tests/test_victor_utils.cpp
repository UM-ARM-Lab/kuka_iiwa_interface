#include <gtest/gtest.h>

#include "victor_hardware/victor_utils.hpp"

TEST(VictorUtils, conversion_from_and_to_jvq_does_not_change) {
  using namespace victor_utils;
  std::vector<double> v{1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7};
  EXPECT_EQ(v, jvqToVector(vectorToJvq(v)));
}

GTEST_API_ int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
