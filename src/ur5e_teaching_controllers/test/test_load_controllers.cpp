// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
//
// Integration test with ros2_control's mock hardware (mock_components/GenericSystem from the
// UR5e ur_mocked description): the controller_manager loads both plugins by type, configures and
// activates them with a running update loop, and the hand-guiding controller's published torque
// equals Pinocchio's gravity at the mock's initial joint positions.
#include <gmock/gmock.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <fstream>
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

  std::string urdf_;
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
  model.set_payload(1.0, Eigen::Vector3d(0.0, 0.0, 0.05));
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
