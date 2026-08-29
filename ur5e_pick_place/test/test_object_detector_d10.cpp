#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "ur5e_pick_place/d10_trimmed_mean.hpp"

namespace
{

TEST(D10TrimmedMean, RemovesExactlyFloorTenPercentFromEachTail)
{
  std::vector<double> values;
  for (int value = 1; value <= 10; ++value) {
    values.push_back(static_cast<double>(value));
  }

  const auto result = ur5e_pick_place::d10_trimmed_mean(values);

  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(*result, 5.5);  // Mean of [2, ..., 9].
}

TEST(D10TrimmedMean, HandlesOddAndEvenSampleCounts)
{
  const std::vector<double> even{1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0};
  const std::vector<double> odd{1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0};

  const auto even_result = ur5e_pick_place::d10_trimmed_mean(even);
  const auto odd_result = ur5e_pick_place::d10_trimmed_mean(odd);

  ASSERT_TRUE(even_result.has_value());
  ASSERT_TRUE(odd_result.has_value());
  EXPECT_DOUBLE_EQ(*even_result, 5.5);  // [2, ..., 9]
  EXPECT_DOUBLE_EQ(*odd_result, 6.0);   // [2, ..., 10]
}

TEST(D10TrimmedMean, KeepsAllFiniteValuesForSmallSampleCounts)
{
  const auto one_sample = ur5e_pick_place::d10_trimmed_mean({0.75});
  const auto nine_samples = ur5e_pick_place::d10_trimmed_mean(
    {0.71, 0.72, 0.73, 0.74, 0.75, 0.76, 0.77, 0.78, 0.79});

  ASSERT_TRUE(one_sample.has_value());
  ASSERT_TRUE(nine_samples.has_value());
  EXPECT_DOUBLE_EQ(*one_sample, 0.75);
  EXPECT_DOUBLE_EQ(*nine_samples, 0.75);
}

TEST(D10TrimmedMean, FiltersNonFiniteValuesBeforeTrimming)
{
  const std::vector<double> values{
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity(),
    1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0};

  const auto result = ur5e_pick_place::d10_trimmed_mean(values);

  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(*result, 5.5);
}

TEST(D10TrimmedMean, RejectsInsufficientFiniteSamples)
{
  EXPECT_FALSE(ur5e_pick_place::d10_trimmed_mean({}).has_value());
  EXPECT_FALSE(
    ur5e_pick_place::d10_trimmed_mean(
      {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()})
    .has_value());
}

TEST(D10TrimmedMean, IsDeterministicIndependentOfInputOrder)
{
  const std::vector<double> ordered{0.70, 0.71, 0.72, 0.73, 0.74, 0.75, 0.76, 0.77, 0.78, 5.00};
  const std::vector<double> permuted{5.00, 0.74, 0.70, 0.78, 0.72, 0.77, 0.73, 0.76, 0.71, 0.75};

  const auto ordered_result = ur5e_pick_place::d10_trimmed_mean(ordered);
  const auto permuted_result = ur5e_pick_place::d10_trimmed_mean(permuted);

  ASSERT_TRUE(ordered_result.has_value());
  ASSERT_TRUE(permuted_result.has_value());
  EXPECT_DOUBLE_EQ(*ordered_result, *permuted_result);
}

}  // namespace
