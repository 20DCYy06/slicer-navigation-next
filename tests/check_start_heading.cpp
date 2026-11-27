#include <slicer_navigation/start_heading.h>

#include <cmath>
#include <iostream>
#include <vector>

int main() {
  using slicer_navigation::HeadingPoint;
  using slicer_navigation::pathLookaheadChord;
  for (double sign : {-1., 1.}) {
    const std::vector<HeadingPoint> route{{-.5, 0.}, {0., 0.}, {0., sign*.2}, {2., sign*.2}};
    const auto point = [](HeadingPoint value) { return value; };
    const auto short_yaw = pathLookaheadChord(route.begin(), route.end(), point, {0., 0.}, .20);
    const auto yaw = pathLookaheadChord(route.begin(), route.end(), point, {0., 0.}, 1.35);
    const double expected = std::atan2(sign*.2, 1.15);
    if (!std::isfinite(yaw) || !std::isfinite(short_yaw) || std::abs(yaw - expected) > 1e-10 || std::abs(short_yaw - yaw) < 1.) {
      std::cerr << "Start chord did not use 1.35 m along the path: " << yaw
                << " (expected " << expected << ")\n";
      return 1;
    }
    const auto short_route = pathLookaheadChord(route.begin(), route.begin()+3, point, {0., 0.}, 1.35);
    if (!std::isfinite(short_route) || std::abs(short_route - sign*slicer_navigation::kHeadingPi/2.) > 1e-10) return 1;
  }
  std::cout << "Start chord regression passed\n";
}
