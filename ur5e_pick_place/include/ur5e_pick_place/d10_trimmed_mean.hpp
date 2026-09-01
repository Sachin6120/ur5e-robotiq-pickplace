#ifndef UR5E_PICK_PLACE__D10_TRIMMED_MEAN_HPP_
#define UR5E_PICK_PLACE__D10_TRIMMED_MEAN_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <optional>
#include <vector>

namespace ur5e_pick_place
{

// D10 depth estimator: discard floor(10% * N) values from each sorted tail,
// then average the remaining finite samples.  For N < 10 this intentionally
// reduces to the ordinary mean, so a small valid component keeps the detector's
// existing "at least one valid depth" publication semantics.
inline std::optional<double> d10_trimmed_mean(std::vector<double> values)
{
  values.erase(
    std::remove_if(
      values.begin(), values.end(),
      [](const double value) { return !std::isfinite(value); }),
    values.end());
  if (values.empty()) {
    return std::nullopt;
  }

  std::sort(values.begin(), values.end());
  const std::size_t trim_count = values.size() / 10;
  const auto first = values.cbegin() + static_cast<std::ptrdiff_t>(trim_count);
  const auto last = values.cend() - static_cast<std::ptrdiff_t>(trim_count);
  if (first == last) {
    return std::nullopt;
  }

  double sum = 0.0;
  for (auto it = first; it != last; ++it) {
    sum += *it;
  }
  return sum / static_cast<double>(std::distance(first, last));
}

}  // namespace ur5e_pick_place

#endif  // UR5E_PICK_PLACE__D10_TRIMMED_MEAN_HPP_
