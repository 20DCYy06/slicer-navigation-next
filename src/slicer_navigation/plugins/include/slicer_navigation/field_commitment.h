#pragma once

#include <slicer_navigation/continuity_cost.h>

namespace slicer_navigation {

inline double routeLength(const std::vector<Point2>& route) {
  double length = 0.;
  for (std::size_t i = 1; i < route.size(); ++i) length += norm(route[i] - route[i-1]);
  return length;
}

// Dense samples preserve corners and cap the total along-path horizon.
inline std::vector<Point2> routeSamples(const std::vector<Point2>& route, double spacing, double limit) {
  if (route.empty()) return {};
  std::vector<Point2> samples{route.front()};
  double travelled = 0.;
  for (std::size_t i = 1; i < route.size() && travelled < limit; ++i) {
    const auto delta = route[i] - route[i-1];
    const double length = norm(delta);
    if (length <= 1e-9) continue;
    const double reach = std::min(length, limit - travelled);
    const int count = std::max(1, static_cast<int>(std::ceil(reach / spacing)));
    for (int j = 1; j <= count; ++j)
      samples.push_back(route[i-1] + delta * (reach * j / count / length));
    travelled += reach;
  }
  return samples;
}

template<class KnownFree>
Point2 commitmentAnchor(const std::vector<Point2>& samples, KnownFree known, double margin) {
  if (samples.empty()) return {NAN, NAN};
  std::size_t end = 0;
  while (end < samples.size() && known(samples[end])) ++end;
  while (end > 1) {
    const auto point = samples[--end];
    const auto back = samples[end-1];
    const auto distance = norm(back - point);
    if (distance >= margin) return point + (back - point) * (margin / distance);
    margin -= distance;
  }
  return samples.front();
}

// Use the most pronounced lookahead deflection relative to the task's fixed
// start/goal axis. A dead band prevents sub-cell jitter from choosing a side.
class SideHysteresis {
 public:
  void reset(Point2 start, Point2 goal) {
    start_ = start;
    axis_ = goal - start;
    side_ = 0;
  }
  int side() const { return side_; }
  bool opposes(const std::vector<Point2>& candidate) const {
    return side_ && classify(candidate) == -side_;
  }
  bool oppositeRegion(Point2 point) const {
    return side_ && side_ * lateralOffset(point) < -.25;
  }
  void accept(const std::vector<Point2>& candidate) {
    const int proposed = classify(candidate);
    if (proposed) side_ = proposed;
  }

  int classify(const std::vector<Point2>& path) const {
    double signed_offset = 0.;
    for (const auto& point : path) {
      const double cross = lateralOffset(point);
      if (std::abs(cross) > std::abs(signed_offset)) signed_offset = cross;
    }
    return signed_offset > .25 ? 1 : signed_offset < -.25 ? -1 : 0;
  }

 private:
  double lateralOffset(Point2 point) const {
    const double length = norm(axis_);
    if (length < .5) return 0.;
    const auto offset = point - start_;
    const double fraction = dot(axis_, offset) / (length * length);
    if (fraction < .1 || fraction > .9) return 0.;
    return (axis_.x * offset.y - axis_.y * offset.x) / length;
  }

  Point2 start_{}, axis_{};
  int side_ = 0;
};

}  // namespace slicer_navigation
