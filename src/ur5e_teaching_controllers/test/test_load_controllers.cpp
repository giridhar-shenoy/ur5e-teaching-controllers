// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
//
// Integration test with ros2_control's mock hardware (mock_components/GenericSystem from the
// UR5e ur_mocked description): the controller_manager loads both plugins by type, configures and
// activates them with a running update loop, and the hand-guiding controller's published torque
// equals Pinocchio's gravity at the mock's initial joint positions.
#include <gmock/gmock.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "controller_manager/controller_manager.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "dynamics/pinocchio_model.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

using namespace std::chrono_literals;
using controller_interface::return_type;
using StateMsg = control_msgs::msg::JointTrajectoryControllerState;
using SwitchRequest = controller_manager_msgs::srv::SwitchController::Request;

namespace
{
const std::vector<std::string> kJoints{"shoulder_pan_joint", "shoulder_lift_joint",
                                       "elbow_joint",        "wrist_1_joint",
                                       "wrist_2_joint",      "wrist_3_joint"};

std::string read_file(const std::string & path)
{
  std::ifstream f(path);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

std::string write_params_file()
{
  const std::string path = std::string(::testing::TempDir()) + "ur5e_teaching_test_params.yaml";
  std::ofstream f(path);
  const char * joints =
    "    joints: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, "
    "wrist_2_joint, wrist_3_joint]\n";
  f << "track:\n  ros__parameters:\n"
    << joints << "    kp: [600.0, 1200.0, 500.0, 30.0, 8.0, 0.5]\n"
    << "    kd: [50.0, 100.0, 40.0, 1.5, 0.3, 0.02]\n"
    << "    ki: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n"
    << "    i_clamp: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n"
    << "    max_velocity: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]\n"
    << "    max_acceleration: [2.0, 2.0, 2.0, 2.0, 2.0, 2.0]\n"
    << "handguide:\n  ros__parameters:\n"
    << joints << "    payload_mass: 1.0\n"
    << "    damping: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n"
    << "    limit_margin: 0.0\n";
  return path;
}

class LoadControllersTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    urdf_ = read_file(UR5E_TEST_URDF);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    cm_ = std::make_shared<controller_manager::ControllerManager>(
      executor_, urdf_, true, "test_controller_manager");
    const std::string params = write_params_file();
    cm_->set_parameter({"track.type", "ur5e_teaching_controllers/PidFfTrajectoryController"});
    cm_->set_parameter({"track.params_file", params});
    cm_->set_parameter(
      {"handguide.type", "ur5e_teaching_controllers/GravityCompHandGuidingController"});
    cm_->set_parameter({"handguide.params_file", params});
  }

  void TearDown() override { stop_loops(); }

  void start_loops()
  {
    running_ = true;
    rt_thread_ = std::thread([this]() {
      const rclcpp::Duration period(0, 2'000'000);
      while (running_) {
        const auto now = cm_->get_clock()->now();
        cm_->read(now, period);
        cm_->update(now, period);
        cm_->write(now, period);
        std::this_thread::sleep_for(2ms);
      }
    });
    spin_thread_ = std::thread([this]() {
      while (running_) {
        executor_->spin_some(10ms);
      }
    });
  }

  void stop_loops()
  {
    if (!running_) {
      return;
    }
    running_ = false;
    rt_thread_.join();
    spin_thread_.join();
  }

  /// Subscribe to <controller>/controller_state; the latest message is kept in last_state_.
  void listen(const std::string & controller)
  {
    listener_ = std::make_shared<rclcpp::Node>("test_state_listener");
    state_sub_ = listener_->create_subscription<StateMsg>(
      "/" + controller + "/controller_state", 10, [this](const StateMsg & msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        last_state_ = msg;
        ++state_count_;
      });
    executor_->add_node(listener_);
  }

  /// Latest state after at least `fresh` new messages (fails the test on timeout).
  StateMsg latest_state(int fresh = 5)
  {
    int start;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      start = state_count_;
    }
    for (int i = 0; i < 300; ++i) {
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (state_count_ >= start + fresh) {
          return last_state_;
        }
      }
      std::this_thread::sleep_for(10ms);
    }
    ADD_FAILURE() << "no controller_state messages";
    return StateMsg{};
  }

  Eigen::VectorXd gravity_at(const StateMsg & state, double payload_mass)
  {
    ur5e_teaching_controllers::dynamics::PinocchioModel model(urdf_, kJoints, "tool0");
    model.set_payload(payload_mass, Eigen::Vector3d(0.0, 0.0, 0.03));
    Eigen::VectorXd q(6);
    for (int j = 0; j < 6; ++j) {
      q[j] = state.feedback.positions[static_cast<std::size_t>(j)];
    }
    return model.gravity(q);
  }

  void activate(const std::string & name)
  {
    ASSERT_NE(cm_->load_controller(name), nullptr);
    ASSERT_EQ(cm_->configure_controller(name), return_type::OK);
    listen(name);
    start_loops();
    ASSERT_EQ(
      cm_->switch_controller({name}, {}, SwitchRequest::STRICT, true, rclcpp::Duration(5s)),
      return_type::OK);
  }

  std::string urdf_;
  std::shared_ptr<rclcpp::Node> listener_;
  rclcpp::Subscription<StateMsg>::SharedPtr state_sub_;
  std::mutex state_mutex_;
  StateMsg last_state_;
  int state_count_{0};
  std::shared_ptr<rclcpp::Executor> executor_;
  std::shared_ptr<controller_manager::ControllerManager> cm_;
  std::atomic<bool> running_{false};
  std::thread rt_thread_, spin_thread_;
};
}  // namespace

TEST_F(LoadControllersTest, BothPluginsLoadAndConfigure)
{
  ASSERT_NE(cm_->load_controller("track"), nullptr);
  ASSERT_NE(cm_->load_controller("handguide"), nullptr);
  EXPECT_EQ(cm_->configure_controller("track"), return_type::OK);
  EXPECT_EQ(cm_->configure_controller("handguide"), return_type::OK);
}

TEST_F(LoadControllersTest, HandGuidingOutputsModelGravityOnMockHardware)
{
  ASSERT_NE(cm_->load_controller("handguide"), nullptr);
  ASSERT_EQ(cm_->configure_controller("handguide"), return_type::OK);

  auto node = std::make_shared<rclcpp::Node>("test_listener");
  std::atomic<bool> got{false};
  StateMsg last;
  std::mutex m;
  auto sub = node->create_subscription<StateMsg>(
    "/handguide/controller_state", 10, [&](const StateMsg & msg) {
      std::lock_guard<std::mutex> lock(m);
      last = msg;
      got = true;
    });
  executor_->add_node(node);
  start_loops();

  ASSERT_EQ(
    cm_->switch_controller({"handguide"}, {}, SwitchRequest::STRICT, true, rclcpp::Duration(5s)),
    return_type::OK);
  for (int i = 0; i < 200 && !got; ++i) {
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(got);

  // Stop the executor before reading: the callback captures locals of this test body by
  // reference, so it must not run while (or after) they are used and destroyed.
  stop_loops();
  executor_->remove_node(node);
  StateMsg state;
  {
    std::lock_guard<std::mutex> lock(m);
    state = last;
  }

  // Expected: g(q) of the same URDF with the same 1 kg model payload at the mock's joint state.
  ur5e_teaching_controllers::dynamics::PinocchioModel model(urdf_, kJoints, "tool0");
  model.set_payload(1.0, Eigen::Vector3d(0.0, 0.0, 0.03));  // payload_com default
  Eigen::VectorXd q(6);
  for (int j = 0; j < 6; ++j) {
    q[j] = state.feedback.positions[static_cast<std::size_t>(j)];
  }
  const Eigen::VectorXd g = model.gravity(q);
  for (int j = 0; j < 6; ++j) {
    EXPECT_NEAR(state.output.effort[static_cast<std::size_t>(j)], g[j], 1e-9) << kJoints[j];
  }
  EXPECT_GT(g.cwiseAbs().maxCoeff(), 1.0);  // the mock pose is not a trivial zero-gravity pose
}

TEST_F(LoadControllersTest, TrackingControllerActivatesAndHoldsWithoutTrajectory)
{
  ASSERT_NE(cm_->load_controller("track"), nullptr);
  ASSERT_EQ(cm_->configure_controller("track"), return_type::OK);
  start_loops();
  ASSERT_EQ(
    cm_->switch_controller({"track"}, {}, SwitchRequest::STRICT, true, rclcpp::Duration(5s)),
    return_type::OK);
  std::this_thread::sleep_for(100ms);
  const auto controllers = cm_->get_loaded_controllers();
  ASSERT_EQ(controllers.size(), 1u);
  EXPECT_EQ(
    controllers[0].c->get_lifecycle_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
}

TEST_F(LoadControllersTest, EffortControllersConflictOnTheSameInterfaces)
{
  ASSERT_NE(cm_->load_controller("track"), nullptr);
  ASSERT_NE(cm_->load_controller("handguide"), nullptr);
  ASSERT_EQ(cm_->configure_controller("track"), return_type::OK);
  ASSERT_EQ(cm_->configure_controller("handguide"), return_type::OK);
  start_loops();
  ASSERT_EQ(
    cm_->switch_controller({"track"}, {}, SwitchRequest::STRICT, true, rclcpp::Duration(5s)),
    return_type::OK);
  // Activating the second effort claimant without releasing the first must be refused.
  EXPECT_NE(
    cm_->switch_controller({"handguide"}, {}, SwitchRequest::STRICT, true, rclcpp::Duration(5s)),
    return_type::OK);
  // Strict hand-over (deactivate the holder in the same request) succeeds.
  EXPECT_EQ(
    cm_->switch_controller(
      {"handguide"}, {"track"}, SwitchRequest::STRICT, true, rclcpp::Duration(5s)),
    return_type::OK);
}

TEST_F(LoadControllersTest, TrackingHoldOutputIsModelGravity)
{
  activate("track");
  const StateMsg state = latest_state();
  stop_loops();
  // Holding: zero error, zero integral, so the command is exactly RNEA(q, 0, 0) = g(q).
  const Eigen::VectorXd g = gravity_at(state, 0.0);
  for (int j = 0; j < 6; ++j) {
    EXPECT_NEAR(state.output.effort[static_cast<std::size_t>(j)], g[j], 1e-9) << kJoints[j];
    EXPECT_NEAR(state.error.positions[static_cast<std::size_t>(j)], 0.0, 1e-12) << kJoints[j];
  }
}

TEST_F(LoadControllersTest, TrackingReordersJointsAndRejectsInvalidTrajectories)
{
  activate("track");
  const StateMsg hold = latest_state();
  auto pub = listener_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/track/joint_trajectory", 10);
  for (int i = 0; i < 100 && pub->get_subscription_count() == 0; ++i) {
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_GT(pub->get_subscription_count(), 0u);

  const std::vector<double> offset{0.1, -0.1, 0.2, 0.05, -0.05, 0.1};
  std::vector<double> target(6);
  for (std::size_t j = 0; j < 6; ++j) {
    target[j] = hold.reference.positions[j] + offset[j];
  }
  auto make = [&](const std::vector<double> & goal_in_our_order) {
    // Joint names deliberately reversed: the controller must map by name, not by index.
    trajectory_msgs::msg::JointTrajectory msg;
    msg.joint_names.assign(kJoints.rbegin(), kJoints.rend());
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(goal_in_our_order.rbegin(), goal_in_our_order.rend());
    point.time_from_start = rclcpp::Duration(0, 500'000'000);
    msg.points.push_back(point);
    return msg;
  };

  auto nan_goal = target;
  nan_goal[3] = std::numeric_limits<double>::quiet_NaN();
  pub->publish(make(nan_goal));
  auto beyond_limit = target;
  beyond_limit[2] = 4.0;  // elbow is limited to +-pi
  pub->publish(make(beyond_limit));
  std::this_thread::sleep_for(300ms);
  StateMsg state = latest_state();
  for (std::size_t j = 0; j < 6; ++j) {
    EXPECT_DOUBLE_EQ(state.reference.positions[j], hold.reference.positions[j]) << kJoints[j];
  }

  pub->publish(make(target));
  bool reached = false;
  for (int i = 0; i < 40 && !reached; ++i) {
    state = latest_state();
    reached = true;
    for (std::size_t j = 0; j < 6; ++j) {
      reached &= std::abs(state.reference.positions[j] - target[j]) < 1e-9;
    }
    std::this_thread::sleep_for(50ms);
  }
  stop_loops();
  EXPECT_TRUE(reached) << "reference did not reach the reordered target";
}

TEST_F(LoadControllersTest, RuntimeParametersAreAdoptedOrRejectedAsAWhole)
{
  activate("track");
  auto controller = cm_->get_loaded_controllers()[0].c;
  // A valid change is adopted: without feedforward the hold command at zero error is zero.
  ASSERT_TRUE(controller->get_node()->set_parameter({"feedforward", false}).successful);
  std::this_thread::sleep_for(100ms);
  StateMsg state = latest_state();
  for (std::size_t j = 0; j < 6; ++j) {
    EXPECT_NEAR(state.output.effort[j], 0.0, 1e-9) << kJoints[j];
  }
  // A per-joint array of the wrong size passes the parameter validators but must not reach the
  // control law (it would be read out of bounds): the controller keeps running on the old set.
  ASSERT_TRUE(controller->get_node()->set_parameter({"kp", std::vector<double>{1.0}}).successful);
  std::this_thread::sleep_for(100ms);
  state = latest_state();
  EXPECT_EQ(
    controller->get_lifecycle_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  for (std::size_t j = 0; j < 6; ++j) {
    EXPECT_TRUE(std::isfinite(state.output.effort[j])) << kJoints[j];
    EXPECT_NEAR(state.output.effort[j], 0.0, 1e-9) << kJoints[j];
  }
  // Fixing the array makes the whole set valid again; feedforward comes back on with it.
  ASSERT_TRUE(controller->get_node()
                ->set_parameter({"kp", std::vector<double>{600.0, 1200.0, 500.0, 30.0, 8.0, 0.5}})
                .successful);
  ASSERT_TRUE(controller->get_node()->set_parameter({"feedforward", true}).successful);
  std::this_thread::sleep_for(100ms);
  state = latest_state();
  stop_loops();
  const Eigen::VectorXd g = gravity_at(state, 0.0);
  for (int j = 0; j < 6; ++j) {
    EXPECT_NEAR(state.output.effort[static_cast<std::size_t>(j)], g[j], 1e-9) << kJoints[j];
  }
}
