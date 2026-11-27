// Run on a dedicated loopback ROS master, with the workspace overlay sourced.
#include <dlfcn.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <dwa_local_planner/dwa_planner_ros.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/base_local_planner.h>
#include <pluginlib/class_loader.hpp>
#include <slicer_navigation/field_commitment.h>
#include <std_msgs/UInt64.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <iostream>
#include <stdexcept>

using Plan = std::vector<geometry_msgs::PoseStamped>;
using slicer_navigation::Point2;

namespace {
int set_plan_calls = 0;
Plan last_dwa_plan;

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

geometry_msgs::PoseStamped pose(double x, double y, ros::Time stamp = {}) {
  geometry_msgs::PoseStamped value;
  value.header.frame_id = "map";
  value.header.stamp = stamp;
  value.pose.position.x = x;
  value.pose.position.y = y;
  value.pose.orientation.w = 1.;
  return value;
}

int side(const Plan& plan) {
  slicer_navigation::SideHysteresis axis;
  axis.reset({-.5, 0.}, {5.5, 0.});
  std::vector<Point2> points;
  for (const auto& p : plan) points.emplace_back(p.pose.position.x, p.pose.position.y);
  return axis.classify(points);
}

void fill(costmap_2d::Costmap2D& grid, double x0, double x1, double y0, double y1,
          unsigned char cost) {
  for (unsigned y = 0; y < grid.getSizeInCellsY(); ++y) {
    for (unsigned x = 0; x < grid.getSizeInCellsX(); ++x) {
      double wx, wy;
      grid.mapToWorld(x, y, wx, wy);
      if (wx >= x0 && wx <= x1 && wy >= y0 && wy <= y1) grid.setCost(x, y, cost);
    }
  }
}

void checkGlobal(costmap_2d::Costmap2DROS& map) {
  ros::NodeHandle nh("~GlobalPlanner");
  nh.setParam("continuity_cost/enabled", true);
  nh.setParam("task_side_hysteresis", true);
  nh.setParam("smoothing/observed_prefix", true);
  nh.setParam("continuity_cost/tracking_reserve_m", .10);
  pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> plugins("nav_core", "nav_core::BaseGlobalPlanner");
  auto planner = plugins.createInstance("slicer_navigation/BSplineGlobalPlanner");
  planner->initialize("GlobalPlanner", &map);
  auto& grid = *map.getCostmap();
  const auto start = pose(-.5, 0.);
  auto goal = pose(5.5, 0., ros::Time::now());
  Plan plan;
  for (int initial_side : {-1, 1}) {
    fill(grid, -20., 20., -20., 20., 0);
    fill(grid, 2., 3.3, -.6, .6, 254);
    if (initial_side > 0) fill(grid, 1.65, 3.7, -1.4, -.65, 70);
    else fill(grid, 1.65, 3.7, .65, 1.4, 70);
    goal.header.stamp = ros::Time::now();
    require(planner->makePlan(start, goal, plan) && side(plan) == initial_side,
            "Initial route selected the wrong side");
    // The unknown stripe cuts the reference before the split. A new point
    // blocks the old line itself while leaving room to replan on its side.
    const auto old_route_point = std::find_if(plan.begin(), plan.end(), [](const auto& waypoint) {
      return waypoint.pose.position.x >= 1.80;
    });
    require(old_route_point != plan.end(), "Old route lacks a point near the rectangle");
    unsigned blocked_x, blocked_y;
    require(grid.worldToMap(old_route_point->pose.position.x, old_route_point->pose.position.y,
                           blocked_x, blocked_y), "Old-route test point is off the map");
    for (int cycle = 0; cycle < 4; ++cycle) {
      fill(grid, 1.65, 3.7, .65, 1.4, cycle % 2 == 0 ? 55 : 0);
      fill(grid, 1.65, 3.7, -1.4, -.65, cycle % 2 == 0 ? 0 : 55);
      fill(grid, .05, .18, -1.0, 1.0, 255);
      grid.setCost(blocked_x, blocked_y, 254);
      require(planner->makePlan(start, goal, plan) && side(plan) == initial_side,
              "Cutting the old reference caused a side change");
    }
    // An obstacle from the rectangle to the observed grid boundary cuts off
    // the remembered corridor, while the opposite corridor stays accessible.
    if (initial_side > 0) fill(grid, 2.55, 2.85, 0., 5., 254);
    else fill(grid, 2.55, 2.85, -5., 0., 254);
    require(planner->makePlan(start, goal, plan) && side(plan) == -initial_side,
            "The planner did not leave the blocked side");
    require(planner->makePlan(start, goal, plan) && side(plan) == -initial_side,
            "The planner did not remember the replacement side");
  }
  std::cout << "Global route: symmetric rectangle retained LEFT/RIGHT, "
               "one-sided closure escaped to the opposite corridor\n";
}

double fixtureRouteCost(const costmap_2d::Costmap2D& grid, const Plan& plan) {
  std::vector<Point2> points;
  for (const auto& p : plan) points.emplace_back(p.pose.position.x, p.pose.position.y);
  const auto samples = slicer_navigation::routeSamples(
      points, grid.getResolution() * .5, slicer_navigation::routeLength(points));
  double total = 0.;
  for (size_t i = 1; i < samples.size(); ++i) {
    const auto middle = (samples[i-1] + samples[i]) * .5;
    unsigned x, y;
    require(grid.worldToMap(middle.x, middle.y, x, y) && grid.getCost(x, y) < 253,
            "Strict-side fixture route is outside the grid or occupied");
    // The fixture enables only normalized obstacle cost (weight 1.0).
    const auto byte = std::round((50. / .8) * grid.getCost(x, y) / 252.);
    total += slicer_navigation::norm(samples[i] - samples[i-1]) * (50. + .8 * byte);
  }
  return total;
}

void checkStrictSide(costmap_2d::Costmap2DROS& map) {
  ros::NodeHandle nh("~GlobalPlanner");
  nh.setParam("continuity_cost/enabled", true);
  nh.setParam("task_side_hysteresis", true);
  nh.setParam("continuity_cost/weight", 0.);
  nh.setParam("continuity_cost/obstacle_weight", 1.);
  nh.setParam("continuity_cost/obstacle_soft_radius_m", 0.);
  nh.setParam("continuity_cost/tracking_reserve_m", .10);
  pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> plugins("nav_core", "nav_core::BaseGlobalPlanner");
  auto planner = plugins.createInstance("slicer_navigation/BSplineGlobalPlanner");
  planner->initialize("GlobalPlanner", &map);
  ros::NodeHandle node("~");
  const auto cache_refresh = node.advertise<std_msgs::UInt64>("map_cache_epoch", 1);
  auto& grid = *map.getCostmap();
  const auto start = pose(-.5, 0.);
  auto goal = pose(5.5, 0., ros::Time::now());

  fill(grid, -20., 20., -20., 20., 0);
  fill(grid, -20., 20., -20., -.15, 252);
  fill(grid, .65, 4.35, -.6, .6, 254);
  Plan initial;
  require(planner->makePlan(start, goal, initial) && side(initial) == 1,
          "Strict-side fixture did not initially lock left");

  // Reverse the cost preference using traversable costs on both sides.
  fill(grid, -20., 20., -20., 20., 0);
  fill(grid, -20., 20., .15, 20., 140);
  fill(grid, .65, 4.35, -.6, .6, 254);
  Plan before_refresh;
  require(planner->makePlan(start, goal, before_refresh) && side(before_refresh) == 1,
          "A higher cost alone switched away from the feasible locked side");

  for (int attempt = 0; cache_refresh.getNumSubscribers() == 0 && attempt < 100; ++attempt)
    ros::WallDuration(.01).sleep();
  require(cache_refresh.getNumSubscribers() > 0, "Planner cache-refresh subscriber did not connect");
  std_msgs::UInt64 version;
  version.data = 1;
  cache_refresh.publish(version);
  ros::WallDuration(.2).sleep();
  Plan after_refresh;
  require(planner->makePlan(start, goal, after_refresh) && side(after_refresh) == 1,
          "Map cache refresh switched away from the feasible locked side");

  goal.header.stamp = ros::Time::now();
  Plan fresh;
  require(planner->makePlan(start, goal, fresh) && side(fresh) == -1,
          "A new goal did not reset the remembered side");
  const double ratio = fixtureRouteCost(grid, after_refresh) / fixtureRouteCost(grid, fresh);
  require(ratio >= 1.50 && ratio <= 1.55, "Strict-side fixture cost ratio must be close to 1.50x");
  std::cout << "Global route: kept the feasible locked side at " << ratio
            << "x cost across a map cache refresh; a new goal chose the cheaper side\n";
}

void checkLocal(costmap_2d::Costmap2DROS& map, tf2_ros::Buffer& tf) {
  fill(*map.getCostmap(), -20., 20., -20., 20., 0);
  pluginlib::ClassLoader<nav_core::BaseLocalPlanner> plugins("nav_core", "nav_core::BaseLocalPlanner");
  auto planner = plugins.createInstance("slicer_navigation/DWAPlannerROS");
  planner->initialize("DWAPlannerROS", &tf, &map);
  ros::NodeHandle nh("~");
  auto goal_pub = nh.advertise<geometry_msgs::PoseStamped>("current_goal", 1, true);
  auto odom_pub = nh.advertise<nav_msgs::Odometry>("test_odom", 1);
  const auto goal = pose(5.5, 0., ros::Time::now());
  goal_pub.publish(goal);
  Plan plan{pose(-.5, 0.), pose(2., 1.), pose(3.5, 1.), goal};
  const auto tick = [&](double yaw) {
    geometry_msgs::TransformStamped transform;
    transform.header.frame_id = "map";
    transform.header.stamp = ros::Time::now();
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = -.5;
    tf2::Quaternion rotation;
    rotation.setRPY(0., 0., yaw);
    transform.transform.rotation = tf2::toMsg(rotation);
    require(tf.setTransform(transform, "route-test", true), "test TF rejected");
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "map";
    odom.pose.pose.orientation = transform.transform.rotation;
    odom_pub.publish(odom);
    ros::WallDuration(.07).sleep();
    geometry_msgs::Twist command;
    require(planner->computeVelocityCommands(command), "heading command unavailable");
    return command;
  };
  ros::WallDuration(.2).sleep();
  require(planner->setPlan(plan), "Initial turn plan rejected");
  require(tick(0.).angular.z > 0., "Initial turn should go left");
  for (int i = 0; i < 3; ++i) {
    plan[2].pose.position.x += .1;
    require(planner->setPlan(plan), "Replan rejected during alignment");
    require(tick(0.).angular.z > 0., "Replan changed the active heading");
  }
  require(set_plan_calls == 0, "The delegate received a path during alignment");
  const double yaw = std::atan2(1., 2.5);
  for (int i = 0; i < 5 && set_plan_calls == 0; ++i)
    require(tick(yaw).linear.x == 0., "Movement before alignment handoff");
  require(set_plan_calls == 1, "Expected one delegate setPlan at alignment, without 0.50 m travel");
  require(last_dwa_plan[2].pose.position.x == plan[2].pose.position.x,
          "The delegate did not receive the latest cached plan");
  require(planner->setPlan(plan) && set_plan_calls == 2, "Cruise did not resume normal replanning");
  std::cout << "DWA setPlan calls during turn/at handoff/after cruise update: 0/1/2\n";
}
}  // namespace

// Count calls on the real Noetic DWA delegate without changing plugin APIs.
bool dwa_local_planner::DWAPlannerROS::setPlan(const Plan& plan) {
  using SetPlan = bool (*)(DWAPlannerROS*, const Plan&);
  static const auto real = reinterpret_cast<SetPlan>(dlsym(RTLD_NEXT,
      "_ZN17dwa_local_planner13DWAPlannerROS7setPlanERKSt6vectorIN13geometry_msgs12PoseStamped_ISaIvEEESaIS5_EE"));
  require(real != nullptr, "Could not resolve Noetic DWA setPlan");
  ++set_plan_calls;
  last_dwa_plan = plan;
  return real(this, plan);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "slicer_route_regression");
  try {
    ros::NodeHandle nh("~");
    nh.setParam("grid/global_frame", "map");
    nh.setParam("grid/robot_base_frame", "base_link");
    nh.setParam("grid/update_frequency", 0.);
    nh.setParam("grid/footprint", "[[-0.145,-0.14],[0.20,-0.14],[0.20,0.14],[-0.145,0.14]]");
    XmlRpc::XmlRpcValue no_layers;
    no_layers.setSize(0);
    nh.setParam("grid/plugins", no_layers);
    nh.setParam("DWAPlannerROS/odom_topic", nh.resolveName("test_odom"));
    ros::AsyncSpinner spinner(2);
    spinner.start();
    tf2_ros::Buffer tf;
    tf.setUsingDedicatedThread(true);
    geometry_msgs::TransformStamped transform;
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = -.5;
    transform.transform.rotation.w = 1.;
    require(tf.setTransform(transform, "route-test", true), "initial TF rejected");
    costmap_2d::Costmap2DROS costmap("grid", tf);
    costmap.getCostmap()->resizeMap(120, 100, .10, -3., -5.);
    if (argc < 2 || std::string(argv[1]) != "local") {
      checkGlobal(costmap);
      checkStrictSide(costmap);
    }
    if (argc < 2 || std::string(argv[1]) != "global") checkLocal(costmap, tf);
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
