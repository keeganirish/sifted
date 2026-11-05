#include "sifted/core/clock.hpp"

#include <gtest/gtest.h>

namespace sifted {

TEST(SimulatedClock, StartsAtGivenTime) {
  EXPECT_EQ(SimulatedClock{}.now().ns, 0);
  EXPECT_EQ(SimulatedClock{Timestamp{42}}.now().ns, 42);
}

TEST(SimulatedClock, AdvancesForwardAndHoldsStill) {
  SimulatedClock c;
  EXPECT_TRUE(c.advance_to(Timestamp{100}));
  EXPECT_TRUE(c.advance_to(Timestamp{100}));  // same instant is fine
  EXPECT_EQ(c.now().ns, 100);
}

TEST(SimulatedClock, RefusesToGoBackwards) {
  SimulatedClock c{Timestamp{100}};
  EXPECT_FALSE(c.advance_to(Timestamp{99}));
  EXPECT_EQ(c.now().ns, 100);
}

TEST(SimulatedClock, UsableThroughBaseInterface) {
  SimulatedClock sim{Timestamp{7}};
  const Clock& clock = sim;
  EXPECT_EQ(clock.now().ns, 7);
}

}  // namespace sifted
