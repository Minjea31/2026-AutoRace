// White-box regression for degenerate centre fits, including the crash input.
#include "../src/deep_lane.cpp"
#include <gtest/gtest.h>
TEST(FitChain, DegenerateCentreHasNoCoefficients) {
  for (float step : {0.f, .1f, 1.f}) {
    deep_lite::Path points, out;
    for (int i = 0; i < 4; ++i) points.emplace_back(i * step, 0.f);
    double rms;
    EXPECT_FALSE(deep_lite::fitChain(points, std::vector<float>(4, 1.f), 1e9, out, rms));
    EXPECT_TRUE(out.empty());
  }
}
TEST(FitChain, NormalCentreStillFits) {
  deep_lite::Path points={{0,0},{0,20},{0,40},{0,60}}, out;
  double rms;
  ASSERT_TRUE(deep_lite::fitChain(points, std::vector<float>(4,1.f), 1e9, out, rms));
  EXPECT_GE(out.size(),8u);
  EXPECT_LT(rms,1e-6);
}
