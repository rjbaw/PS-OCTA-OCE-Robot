#include "motion_utils.hpp"
#include <gtest/gtest.h>

TEST(MotionUtils, EnvelopeBasics) {
    Eigen::Isometry3d centre = Eigen::Isometry3d::Identity();
    const std::string frame = "world";
    const std::string link = "tcp";
    const double lin_r = 0.1;
    const double ang_r = 1.57; // ~90 deg

    auto c = octa_ros::motion::make_envelope(centre, frame, link, lin_r, ang_r);
    ASSERT_EQ(c.position_constraints.size(), 1u);
    ASSERT_EQ(c.orientation_constraints.size(), 1u);
    EXPECT_EQ(c.position_constraints[0].header.frame_id, frame);
    EXPECT_EQ(c.position_constraints[0].link_name, link);
    EXPECT_EQ(c.orientation_constraints[0].header.frame_id, frame);
    EXPECT_EQ(c.orientation_constraints[0].link_name, link);
    EXPECT_NEAR(
        c.position_constraints[0].constraint_region.primitives[0].dimensions[0],
        lin_r, 1e-6);
    EXPECT_NEAR(c.orientation_constraints[0].absolute_x_axis_tolerance, ang_r,
                1e-6);
}
