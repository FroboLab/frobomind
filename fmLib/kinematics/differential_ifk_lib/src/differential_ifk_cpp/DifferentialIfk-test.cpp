/****************************************************************************
 # FroboMind
 # Licence and description in .hpp file
 # To run this test:
 # catkin_make run_tests_differential_ifk_lib
 ****************************************************************************/
#include <gtest/gtest.h>
#include "DifferentialIfk.hpp"

TEST(KinematicsTest, Forward) {
	DifferentialIfk object(1.0);
    ASSERT_EQ(1.0, object.forward(1.0, 1.0).linear);
    ASSERT_EQ(0.0, object.forward(1.0, 1.0).angular);
}

TEST(KinematicsTest, Inverse) {
	DifferentialIfk object(1.0);
    ASSERT_EQ(0.0, object.inverse(1.0, 1.0).left);
    ASSERT_EQ(2.0, object.inverse(1.0, 1.0).right);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
