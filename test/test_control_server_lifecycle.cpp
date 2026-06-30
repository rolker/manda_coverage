// Copyright 2026 Roland Arsenault
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// End-to-end lifecycle test for the marine_control ControlServer adoption
// (issue #6). Drives MandaCoverageActionServer through configure()/activate()
// and asserts it publishes a ControlSet heartbeat carrying the seven bound
// coverage knobs; then deactivate() and asserts the heartbeat stops (the server
// is torn down). Exercises the on_activate construct+bind and on_deactivate
// reset paths that the rclcpp::Node-based test_parameters.cpp cannot reach.

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <set>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "manda_coverage/action_server.h"
#include "marine_control_interfaces/msg/control_set.hpp"

using marine_control_interfaces::msg::ControlSet;
using namespace std::chrono_literals;

class ControlServerLifecycleTest : public ::testing::Test
{
public:
  // Initialise the context once for the suite: the fixture's executor_ member is
  // constructed (and creates a guard condition) before SetUp() runs, so the
  // context must already exist at fixture-construction time.
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

protected:
  void SetUp() override
  {
    node_ = std::make_shared<manda_coverage::MandaCoverageActionServer>();

    // A plain helper node carries the subscription so the lifecycle node under
    // test is exercised purely through its transitions.
    sub_node_ = std::make_shared<rclcpp::Node>("control_state_listener");

    // Match the server's state QoS: RELIABLE + VOLATILE (ADR-0003 D5).
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    sub_ = sub_node_->create_subscription<ControlSet>(
      "/manda_coverage_action_server/control/state", qos,
      [this](ControlSet::SharedPtr msg)
      {
        last_set_ = *msg;
        ++count_;
      });
    executor_.add_node(sub_node_);
  }

  void TearDown() override
  {
    // Remove the lifecycle node from the executor before dropping our owning
    // reference, so the executor never holds a node that is being destroyed.
    // (The test adds node_ via get_node_base_interface(); remove_node is a no-op
    // if a path left it un-added.)
    executor_.remove_node(node_->get_node_base_interface());
    executor_.remove_node(sub_node_);
    sub_.reset();
    sub_node_.reset();
    node_.reset();
  }

  // Spin (non-blocking) until `pred` holds or `timeout` elapses. Polls on a
  // steady-clock deadline rather than a fixed sleep so the test is robust to
  // discovery / scheduling jitter in CI.
  template<typename Pred>
  bool spin_until(Pred pred, std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      if (pred()) {return true;}
      std::this_thread::sleep_for(10ms);
    }
    executor_.spin_some();
    return pred();
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<manda_coverage::MandaCoverageActionServer> node_;
  std::shared_ptr<rclcpp::Node> sub_node_;
  rclcpp::Subscription<ControlSet>::SharedPtr sub_;
  ControlSet last_set_;
  int count_ = 0;
};

TEST_F(ControlServerLifecycleTest, PublishesBoundKnobsThenStopsOnDeactivate)
{
  // Lifecycle transitions are synchronous; run them before adding the node to
  // the executor so the ControlServer's callback group (created in on_activate)
  // is present when the executor collects entities.
  ASSERT_EQ(node_->configure().label(), "inactive");
  ASSERT_EQ(node_->activate().label(), "active");
  executor_.add_node(node_->get_node_base_interface());

  // The heartbeat republishes once per second; allow generous slack for CI.
  ASSERT_TRUE(spin_until([this] {return count_ > 0;}, 10s))
    << "no ControlSet heartbeat received after activate()";

  EXPECT_EQ(last_set_.device_name, "Manda Coverage");
  ASSERT_EQ(last_set_.items.size(), 7u)
    << "expected the seven bound coverage knobs in the ControlSet";

  const std::set<std::string> expected = {
    "swath_overlap", "max_bend_angle", "swath_record_interval",
    "min_allowable_swath", "waypoint_distance_threshold",
    "lead_in_distance", "lead_out_distance"};
  std::set<std::string> got;
  for (const auto & item : last_set_.items) {
    EXPECT_EQ(item.group, "Coverage")
      << item.name << " not in the Coverage group";
    got.insert(item.name);
  }
  EXPECT_EQ(got, expected) << "bound control names do not match the seven knobs";

  // Deactivate tears the ControlServer down: no NEW heartbeat must arrive.
  ASSERT_EQ(node_->deactivate().label(), "inactive");

  // Drain any heartbeat already in transit before snapshotting the baseline. A
  // single spin_some() can miss a RELIABLE sample published just before
  // deactivate() but not yet delivered, which would later read as a spurious
  // "heartbeat continued". Spin until the count goes quiet (no new message for a
  // short settle window) or a bounded deadline elapses.
  int last_count = -1;
  const auto drain_deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < drain_deadline && count_ != last_count) {
    last_count = count_;
    spin_until([&] {return count_ > last_count;}, 200ms);
  }
  const int count_at_deactivate = count_;

  // Spin past more than one heartbeat period; expect no further messages.
  const bool got_new =
    spin_until([&] {return count_ > count_at_deactivate;}, 3s);
  EXPECT_FALSE(got_new) << "heartbeat continued after deactivate()";

  // Re-activate: on_activate constructs a fresh ControlServer and re-binds the
  // seven knobs. Its callback group is created with the rclcpp default
  // automatically_add_to_executor_with_node=true, so the already-added node's new
  // timer/subscription are collected on the next spin cycle and the heartbeat
  // resumes. This exercises the deactivate->activate re-bind path that a single
  // activate cannot. (Production runs on a SingleThreadedExecutor; this test
  // matches that — see main.cpp.)
  last_set_ = ControlSet{};
  const int count_before_reactivate = count_;
  ASSERT_EQ(node_->activate().label(), "active");
  ASSERT_TRUE(spin_until([&] {return count_ > count_before_reactivate;}, 10s))
    << "no ControlSet heartbeat received after re-activate()";

  EXPECT_EQ(last_set_.device_name, "Manda Coverage");
  ASSERT_EQ(last_set_.items.size(), 7u)
    << "expected the seven bound coverage knobs after re-activate()";
  std::set<std::string> got_reactivate;
  for (const auto & item : last_set_.items) {
    EXPECT_EQ(item.group, "Coverage")
      << item.name << " not in the Coverage group after re-activate()";
    got_reactivate.insert(item.name);
  }
  EXPECT_EQ(got_reactivate, expected)
    << "bound control names do not match the seven knobs after re-activate()";
}
