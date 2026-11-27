#include <slicer_navigation/start_heading.h>
#include <slicer_navigation/field_commitment.h>
#include <slicer_navigation/PlannerVelocity.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <angles/angles.h>
#include <base_local_planner/goal_functions.h>
#include <costmap_2d/cost_values.h>
#include <dwa_local_planner/dwa_planner_ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.h>
#include <ros/param.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace slicer_navigation {

class HeadingDWAPlanner : public nav_core::BaseLocalPlanner {
 public:
  void initialize(std::string name, tf2_ros::Buffer* tf,
                  costmap_2d::Costmap2DROS* costmap) override {
    if (costmap_) return;
    tf_ = tf;
    costmap_ = costmap;
    ros::NodeHandle nh("~/" + name);
    xy_tolerance_param_ = nh.resolveName("xy_goal_tolerance");
    yaw_tolerance_param_ = nh.resolveName("yaw_goal_tolerance");
    double rate, settle, timeout, kp, kd;
    nh.param("start_heading_rate", rate, .45);
    nh.param("start_heading_tolerance", tolerance_, .12);
    nh.param("start_heading_settle", settle, .25);
    nh.param("start_heading_timeout", timeout, 16.);
    nh.param("start_heading_min_distance", min_distance_, .20);
    nh.param("start_heading_lookahead", lookahead_, 1.35);
    nh.param("start_heading_kp", kp, 1.6);
    nh.param("start_heading_kd", kd, .25);
    for (double value : {rate, tolerance_, settle, timeout, min_distance_, lookahead_, kp, kd})
      if (!std::isfinite(value) || value <= 0.)
        throw std::invalid_argument("start heading parameters must be positive and finite");
    if (lookahead_ < 1.2 || lookahead_ > 1.5 || tolerance_ >= 10.*kHeadingPi/180.)
      throw std::invalid_argument("start heading expects 1.2–1.5 m lookahead and tolerance below 10 degrees");
    double angular_limit;
    nh.param("max_vel_theta", angular_limit, rate);
    nh.param("trans_stopped_vel", trans_stopped_vel_, .025);
    nh.param("theta_stopped_vel", theta_stopped_vel_, .06);
    heading_.reset(new StartHeading(std::min(rate, angular_limit), tolerance_, settle, timeout, kp, kd));
    nh.param("goal_approach_decel", approach_decel_, .60);
    nh.param("goal_approach_response", approach_response_, .25);
    if (!std::isfinite(approach_decel_) || approach_decel_ <= 0. ||
        !std::isfinite(approach_response_) || approach_response_ < 0.)
      throw std::invalid_argument("invalid goal approach deceleration or response time");
    std::string odom_topic;
    nh.param<std::string>("odom_topic", odom_topic, "odom");
    ros::NodeHandle node;
    odometry_ = node.subscribe(odom_topic, 1, &HeadingDWAPlanner::onOdom, this);
    delegate_.initialize(name, tf, costmap);
    command_pub_ = nh.advertise<PlannerVelocity>("command", 1);
    local_plan_preview_ = nh.advertise<nav_msgs::Path>("local_plan", 1);
    ros::NodeHandle move_base("~");
    goals_ = move_base.subscribe("current_goal", 5, &HeadingDWAPlanner::onGoal, this);
  }

  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override {
    if (plan.empty()) return false;
    const auto& target = plan.back();
    if (!has_goal_ || target.header.stamp != goal_.header.stamp || differentGoalPosition(target)) {
      goal_ = target;
      has_goal_ = true;
      clearAlignment();
      readPositionOnlyTolerance();
      task_axis_.reset(point(plan.front()), point(target));
    }
    if (aligned_) return delegate_.setPlan(plan);
    std::vector<Point2> route;
    route.reserve(plan.size());
    for (const auto& pose : plan) route.push_back(point(pose));
    pending_side_ = task_axis_.classify(route);
    pending_plan_ = plan;
    return true;
  }

  bool computeVelocityCommands(geometry_msgs::Twist& command) override {
    PlannerVelocity stamped;
    // Queued output carries its original control time and the exact target of
    // the plan, even if move_base delivers a stale buffered route.
    stamped.header.stamp = ros::Time::now();
    stamped.header.frame_id = "base_link";
    stamped.target_stamp = goal_.header.stamp;
    const bool valid = computeCommand(command);
    if (valid) stamped.twist = command;
    command_pub_.publish(stamped);
    return valid;
  }

 private:
  bool computeCommand(geometry_msgs::Twist& command) {
    command = geometry_msgs::Twist();
    if (!currentGoalMatchesPlan()) return true;
    if (aligned_) return cruise(command);
    if (pending_plan_.empty()) return false;
    geometry_msgs::PoseStamped robot;
    if (!costmap_->getRobotPose(robot)) return false;
    const double distance = distanceToGoal(robot);
    if (!std::isfinite(distance)) return false;
    if (distance <= min_distance_) {
      if (!usePendingPlan()) return false;
      aligned_ = true;
      return cruise(command);
    }

    const auto now = ros::WallTime::now().toSec();
    if (!std::isfinite(bearing_)) {
      HeadingReference reference;
      if (!readHeadingReference(robot, reference)) return false;
      acquireHeading(std::move(reference), now);
    } else if (sideChanged() || prefixBlocked(heading_prefix_)) {
      // Zero before turning toward a new side or a replacement path.
      bearing_ = std::numeric_limits<double>::quiet_NaN();
      heading_prefix_.clear();
      return true;
    }

    const auto odom = freshOdometry();
    if (!odom) {
      ROS_WARN_THROTTLE(2., "RTK start heading needs fresh odometry");
      return false;
    }
    const auto& twist = odom->twist.twist;
    const bool stopped = std::hypot(twist.linear.x, twist.linear.y) <= trans_stopped_vel_ &&
                         std::abs(twist.angular.z) <= theta_stopped_vel_;
    const double robot_yaw = tf2::getYaw(robot.pose.orientation);
    const double error = angles::shortest_angular_distance(robot_yaw, bearing_);
    if (!std::isfinite(error) || !std::isfinite(twist.angular.z)) return false;
    const auto step = heading_->update(error, twist.angular.z, stopped, now);
    if (step.failed) {
      ROS_WARN_THROTTLE(2., "RTK initial turn did not settle within its time limit");
      return false;
    }
    command.angular.z = step.angular;
    if (step.ready) {
      HeadingReference latest;
      if (!readHeadingReference(robot, latest)) {
        bearing_ = std::numeric_limits<double>::quiet_NaN();
        heading_prefix_.clear();
        return false;
      }
      // The buffered same-side path may have moved while the vehicle turned.
      // Complete the alignment against that path before sending it to DWA.
      if (std::abs(angles::shortest_angular_distance(robot_yaw, latest.yaw)) > tolerance_) {
        acquireHeading(std::move(latest), now);
        return true;
      }
      if (!usePendingPlan()) return false;
      heading_prefix_.clear();
      ROS_INFO("RTK start heading settled; handed the latest path to DWA");
      aligned_ = true;
    }
    return true;
  }

 public:
  bool isGoalReached() override {
    if (!currentGoalMatchesPlan()) return false;
    geometry_msgs::PoseStamped robot;
    const bool position_reached = position_only_tolerance_ > 0. &&
        costmap_->getRobotPose(robot) && distanceToGoal(robot) <= position_only_tolerance_;
    if (!position_reached && (!aligned_ || !delegate_.isGoalReached())) return false;
    has_goal_ = false;
    clearAlignment();
    return true;
  }

 private:
  void readPositionOnlyTolerance() {
    // Goal submission follows dynamic reconfigure. Read the committed values
    // once per stamped goal so a replacement cannot inherit an old tolerance.
    position_only_tolerance_ = 0.;
    double yaw, xy;
    if (ros::param::get(yaw_tolerance_param_, yaw) && std::isfinite(yaw) && yaw > kHeadingPi &&
        ros::param::get(xy_tolerance_param_, xy) && std::isfinite(xy) && xy > 0.) {
      position_only_tolerance_ = xy;
    }
  }

  bool usePendingPlan() {
    if (pending_plan_.empty()) return true;
    if (!delegate_.setPlan(pending_plan_)) return false;
    pending_plan_.clear();
    return true;
  }

  void clearAlignment() {
    bearing_ = std::numeric_limits<double>::quiet_NaN();
    heading_prefix_.clear();
    pending_side_ = heading_side_ = 0;
    aligned_ = false;
    pending_plan_.clear();
    last_drive_ = geometry_msgs::Twist();
    last_forward_output_ = 0.;
  }

  struct HeadingReference {
    double yaw = std::numeric_limits<double>::quiet_NaN();
    std::vector<HeadingPoint> prefix;
    std::vector<geometry_msgs::PoseStamped> local_path;
  };

  static Point2 point(const geometry_msgs::PoseStamped& pose) {
    return {pose.pose.position.x, pose.pose.position.y};
  }

  bool sideChanged() const {
    return heading_side_ && pending_side_ == -heading_side_;
  }

  bool readHeadingReference(const geometry_msgs::PoseStamped& robot, HeadingReference& reference) {
    if (!base_local_planner::transformGlobalPlan(*tf_, pending_plan_, robot, *costmap_->getCostmap(),
            costmap_->getGlobalFrameID(), reference.local_path)) return false;
    const auto& position = robot.pose.position;
    reference.yaw = pathLookaheadChord(reference.local_path.begin(), reference.local_path.end(),
        [](const auto& pose) { return HeadingPoint{pose.pose.position.x, pose.pose.position.y}; },
        {position.x, position.y}, lookahead_, [&](HeadingPoint sample) {
          reference.prefix.emplace_back(position.x + sample.first, position.y + sample.second);
        });
    if (!std::isfinite(reference.yaw)) {
      ROS_WARN_THROTTLE(2., "RTK start heading needs a local lookahead chord");
      return false;
    }
    if (prefixBlocked(reference.prefix)) {
      ROS_WARN_THROTTLE(2., "RTK start heading path prefix intersects a hard obstacle");
      return false;
    }
    return true;
  }

  void acquireHeading(HeadingReference reference, double now) {
    bearing_ = reference.yaw;
    heading_side_ = pending_side_;
    heading_prefix_ = std::move(reference.prefix);
    heading_->start(now);
    nav_msgs::Path preview;
    preview.header = reference.local_path.front().header;
    preview.header.stamp = ros::Time::now();
    preview.poses = std::move(reference.local_path);
    local_plan_preview_.publish(preview);
    ROS_INFO("RTK start heading selected from %.2f m of path: %.3f rad", lookahead_, bearing_);
  }

  bool prefixBlocked(const std::vector<HeadingPoint>& prefix) const {
    auto* grid = costmap_->getCostmap();
    std::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*grid->getMutex());
    const double step = grid->getResolution() * .5;
    for (size_t i = 1; i < prefix.size(); ++i) {
      const auto& from = prefix[i-1];
      const double dx = prefix[i].first - from.first;
      const double dy = prefix[i].second - from.second;
      const int steps = std::max(1, static_cast<int>(std::ceil(std::hypot(dx, dy) / step)));
      for (int n = 1; n <= steps; ++n) {
        const double fraction = static_cast<double>(n) / steps;
        unsigned x, y;
        if (!grid->worldToMap(from.first + fraction*dx, from.second + fraction*dy, x, y)) continue;
        const auto cost = grid->getCost(x, y);
        if (cost == costmap_2d::LETHAL_OBSTACLE || cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
          return true;
      }
    }
    return false;
  }

  nav_msgs::Odometry::ConstPtr freshOdometry() {
    std::lock_guard<std::mutex> lock(odom_lock_);
    if (!last_odom_ || last_odom_->header.stamp.isZero()) return {};
    const double age = (ros::Time::now() - last_odom_->header.stamp).toSec();
    return age >= 0. && age <= .5 ? last_odom_ : nav_msgs::Odometry::ConstPtr{};
  }

  double distanceToGoal(const geometry_msgs::PoseStamped& robot) const {
    auto target = goal_;
    if (target.header.frame_id != robot.header.frame_id) {
      target.header.stamp = ros::Time(0);
      try {
        target = tf_->transform(target, robot.header.frame_id, ros::Duration(.02));
      } catch (const tf2::TransformException& error) {
        ROS_WARN_THROTTLE(2., "RTK navigation needs goal TF: %s", error.what());
        return std::numeric_limits<double>::quiet_NaN();
      }
    }
    return std::hypot(target.pose.position.x - robot.pose.position.x,
                      target.pose.position.y - robot.pose.position.y);
  }

  bool cruise(geometry_msgs::Twist& command) {
    const bool planned = delegate_.computeVelocityCommands(command);
    const auto now = ros::WallTime::now();
    if (!planned) {
      // Brake on the last arc through a local planning miss. Each command
      // still goes through the Guard, and the bridge expires at zero speed.
      if (last_drive_.linear.x <= 0.) return false;
      constexpr double braking_rate = 1.0;
      constexpr double maximum_duration = .8;
      const double elapsed = (now - last_drive_at_).toSec();
      const double initial_speed = std::min(last_drive_.linear.x, braking_rate * maximum_duration);
      const double braking_speed = initial_speed - braking_rate * elapsed;
      if (elapsed <= 0. || braking_speed <= 0.) return false;
      const auto odom = freshOdometry();
      if (!odom) return false;
      const double measured = odom->twist.twist.linear.x;
      if (!std::isfinite(measured)) return false;
      const double speed = std::min({braking_speed, measured, last_forward_output_});
      if (speed <= 0.) return false;
      command = geometry_msgs::Twist();
      command.linear.x = speed;
      command.angular.z = last_drive_.angular.z * speed / last_drive_.linear.x;
    }
    if (command.linear.x <= 0.) {
      last_drive_ = command;
      last_forward_output_ = 0.;
      return true;
    }
    geometry_msgs::PoseStamped robot;
    if (!costmap_->getRobotPose(robot)) return false;
    const double distance = distanceToGoal(robot);
    if (!std::isfinite(distance)) return false;

    // Solve distance = speed * response + speed² / (2 * deceleration).
    // Reduce both components to keep the collision-checked arc's curvature.
    const double margin = approach_decel_ * approach_response_;
    const double cap = std::sqrt(margin * margin + 2. * approach_decel_ * distance) - margin;
    if (command.linear.x > cap) {
      command.angular.z *= cap / command.linear.x;
      command.linear.x = cap;
    }
    last_forward_output_ = command.linear.x;
    if (planned) {
      last_drive_ = command;
      last_drive_at_ = now;
    } else {
      ROS_INFO_THROTTLE(1., "RTK DWA planning gap: braking on the last arc");
    }
    return true;
  }

  bool differentGoalPosition(const geometry_msgs::PoseStamped& target) const {
    return target.header.frame_id != goal_.header.frame_id ||
        std::hypot(target.pose.position.x - goal_.pose.position.x,
                   target.pose.position.y - goal_.pose.position.y) > 1e-6;
  }

  bool currentGoalMatchesPlan() {
    std::lock_guard<std::mutex> lock(goal_lock_);
    return has_goal_ && current_goal_ && current_goal_->header.stamp == goal_.header.stamp
        && !differentGoalPosition(*current_goal_);
  }

  void onGoal(const geometry_msgs::PoseStamped::ConstPtr& goal) {
    std::lock_guard<std::mutex> lock(goal_lock_);
    current_goal_ = goal;
  }

  void onOdom(const nav_msgs::Odometry::ConstPtr& sample) {
    std::lock_guard<std::mutex> lock(odom_lock_);
    last_odom_ = sample;
  }

  dwa_local_planner::DWAPlannerROS delegate_;
  std::mutex odom_lock_;
  nav_msgs::Odometry::ConstPtr last_odom_;
  std::unique_ptr<StartHeading> heading_;
  ros::Subscriber goals_, odometry_;
  ros::Publisher local_plan_preview_;
  tf2_ros::Buffer* tf_ = nullptr;
  costmap_2d::Costmap2DROS* costmap_ = nullptr;
  geometry_msgs::PoseStamped goal_;
  geometry_msgs::Twist last_drive_;
  ros::WallTime last_drive_at_;
  double last_forward_output_ = 0.;

  SideHysteresis task_axis_;
  std::vector<HeadingPoint> heading_prefix_;
  std::vector<geometry_msgs::PoseStamped> pending_plan_;
  double bearing_ = std::numeric_limits<double>::quiet_NaN();
  int pending_side_ = 0, heading_side_ = 0;
  ros::Publisher command_pub_;
  double trans_stopped_vel_ = .025, theta_stopped_vel_ = .06;
  std::mutex goal_lock_;
  geometry_msgs::PoseStamped::ConstPtr current_goal_;
  std::string xy_tolerance_param_, yaw_tolerance_param_;
  double position_only_tolerance_ = 0.;
  double min_distance_ = .2, lookahead_ = 1.35, tolerance_ = .12;
  double approach_decel_ = .60, approach_response_ = .25;
  bool has_goal_ = false, aligned_ = false;
};

}  // namespace slicer_navigation

PLUGINLIB_EXPORT_CLASS(slicer_navigation::HeadingDWAPlanner, nav_core::BaseLocalPlanner)
