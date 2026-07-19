#include <chrono>

#include <gtest/gtest.h>

#include "coordinator_node.hpp"

using namespace std::chrono_literals;

namespace {

using Latch = octa_ros::FullScanRequestLatch;
using Event = Latch::Event;

TEST(FullScanRequestLatchTest, InitialHighStartsImmediatelyAndOnlyOnce) {
    Latch latch(1s);
    const Latch::TimePoint start{};

    EXPECT_EQ(latch.observe(true, start), Event::StartRequested);
    EXPECT_TRUE(latch.active());
    EXPECT_EQ(latch.observe(true, start + 1ms), Event::None);
    EXPECT_EQ(latch.poll(start + 10s), Event::None);
}

TEST(FullScanRequestLatchTest, InitialLowArmsImmediateHighStart) {
    Latch latch(1s);
    const Latch::TimePoint start{};

    EXPECT_EQ(latch.observe(false, start), Event::None);
    EXPECT_EQ(latch.observe(true, start + 1ms), Event::StartRequested);
    EXPECT_TRUE(latch.active());
}

TEST(FullScanRequestLatchTest, OctaLowPulseInsideGraceDoesNotCancel) {
    Latch latch(1s);
    const Latch::TimePoint start{};
    ASSERT_EQ(latch.observe(true, start), Event::StartRequested);

    EXPECT_EQ(latch.observe(false, start + 100ms), Event::None);
    EXPECT_EQ(latch.poll(start + 1098ms), Event::None);
    EXPECT_TRUE(latch.active());
    EXPECT_EQ(latch.observe(true, start + 1099ms), Event::None);
    EXPECT_EQ(latch.poll(start + 10s), Event::None);
    EXPECT_TRUE(latch.active());
}

TEST(FullScanRequestLatchTest, PersistentLowCancelsWhenGracePeriodEnds) {
    Latch latch(1s);
    const Latch::TimePoint start{};
    ASSERT_EQ(latch.observe(true, start), Event::StartRequested);
    ASSERT_EQ(latch.observe(false, start + 100ms), Event::None);

    EXPECT_EQ(latch.poll(start + 1099ms), Event::None);
    EXPECT_TRUE(latch.active());
    EXPECT_EQ(latch.poll(start + 1100ms), Event::PersistentLow);
    EXPECT_FALSE(latch.active());
    EXPECT_EQ(latch.poll(start + 1101ms), Event::None);
    EXPECT_EQ(latch.poll(start + 10s), Event::None);
}

TEST(FullScanRequestLatchTest, RepeatedLowMessagesDoNotExtendGracePeriod) {
    Latch latch(1s);
    const Latch::TimePoint start{};
    ASSERT_EQ(latch.observe(true, start), Event::StartRequested);
    ASSERT_EQ(latch.observe(false, start + 100ms), Event::None);

    EXPECT_EQ(latch.observe(false, start + 400ms), Event::None);
    EXPECT_EQ(latch.observe(false, start + 800ms), Event::None);
    EXPECT_EQ(latch.poll(start + 1100ms), Event::PersistentLow);
}

TEST(FullScanRequestLatchTest, HighBeforeGraceExpiresCancelsPendingLowStop) {
    Latch latch(1s);
    const Latch::TimePoint start{};
    ASSERT_EQ(latch.observe(true, start), Event::StartRequested);
    ASSERT_EQ(latch.observe(false, start + 100ms), Event::None);

    EXPECT_EQ(latch.observe(true, start + 1099ms), Event::None);
    EXPECT_EQ(latch.poll(start + 1100ms), Event::None);
    EXPECT_TRUE(latch.active());
}

TEST(FullScanRequestLatchTest, PersistentLowRearmsFutureHigh) {
    Latch latch(1s);
    const Latch::TimePoint start{};
    ASSERT_EQ(latch.observe(true, start), Event::StartRequested);
    ASSERT_EQ(latch.observe(false, start + 100ms), Event::None);
    ASSERT_EQ(latch.poll(start + 1100ms), Event::PersistentLow);

    EXPECT_EQ(latch.observe(true, start + 2s), Event::StartRequested);
    EXPECT_TRUE(latch.active());
}

TEST(FullScanRequestLatchTest, FinishWhileHighRequiresLaterLowToRearm) {
    Latch latch(1s);
    const Latch::TimePoint start{};
    ASSERT_EQ(latch.observe(true, start), Event::StartRequested);

    latch.finish();

    EXPECT_FALSE(latch.active());
    EXPECT_EQ(latch.observe(true, start + 1s), Event::None);
    EXPECT_EQ(latch.poll(start + 10s), Event::None);
    EXPECT_EQ(latch.observe(false, start + 11s), Event::None);
    EXPECT_EQ(latch.observe(true, start + 12s), Event::StartRequested);
}

TEST(FullScanRequestLatchTest, FinishDuringDropoutIgnoresRecoveryHigh) {
    Latch latch(1s);
    const Latch::TimePoint start{};
    ASSERT_EQ(latch.observe(true, start), Event::StartRequested);
    ASSERT_EQ(latch.observe(false, start + 100ms), Event::None);

    latch.finish();

    // The LOW happened before finish(), so the recovery HIGH cannot rearm.
    EXPECT_EQ(latch.observe(true, start + 200ms), Event::None);
    EXPECT_FALSE(latch.active());

    // A LOW actually observed after finish() rearms the next request.
    EXPECT_EQ(latch.observe(false, start + 300ms), Event::None);
    EXPECT_EQ(latch.observe(true, start + 400ms), Event::StartRequested);
}

TEST(FullScanRequestLatchTest,
     FinishDuringLowRequiresNewPostFinishFallingEdge) {
    Latch latch(1s);
    const Latch::TimePoint start{};
    ASSERT_EQ(latch.observe(true, start), Event::StartRequested);
    ASSERT_EQ(latch.observe(false, start + 100ms), Event::None);

    latch.finish();

    // Repeated samples from the same pre-finish LOW are not a rearm signal.
    EXPECT_EQ(latch.observe(false, start + 200ms), Event::None);
    EXPECT_EQ(latch.observe(false, start + 300ms), Event::None);

    // This HIGH is the OCT-A recovery edge, not a new Fullscan request.
    EXPECT_EQ(latch.observe(true, start + 400ms), Event::None);
    EXPECT_FALSE(latch.active());

    // Only a new post-finish HIGH-to-LOW edge rearms a later HIGH request.
    EXPECT_EQ(latch.observe(false, start + 500ms), Event::None);
    EXPECT_EQ(latch.observe(true, start + 600ms), Event::StartRequested);
    EXPECT_TRUE(latch.active());
}

TEST(FullScanRequestLatchTest, ExplicitCancelUsesSameDisarmedFinishState) {
    Latch latch(1s);
    const Latch::TimePoint start{};
    ASSERT_EQ(latch.observe(true, start), Event::StartRequested);

    latch.finish();

    EXPECT_EQ(latch.observe(true, start + 5s), Event::None);
    EXPECT_EQ(latch.observe(false, start + 6s), Event::None);
    EXPECT_EQ(latch.observe(true, start + 7s), Event::StartRequested);
}

} // namespace
