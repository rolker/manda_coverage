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
// Tests for the coverage-density tuning parameters declared by SurveyPath, and
// the RecordSwath swath thresholding that backs the min_allowable_swath knob.

#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "manda_coverage/RecordSwath.h"
#include "manda_coverage/SurveyPath.h"

using manda_coverage::BoatSide;
using manda_coverage::NodeInterfaces;
using manda_coverage::RecordSwath;
using manda_coverage::SurveyPath;

// ---------------------------------------------------------------------------
// SurveyPath parameter declaration tests
// ---------------------------------------------------------------------------

class SurveyPathParameterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_survey_path");
    survey_path_ = std::make_unique<SurveyPath>(NodeInterfaces(*node_));
    survey_path_->configure();
  }

  void TearDown() override
  {
    survey_path_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<SurveyPath> survey_path_;
};

TEST_F(SurveyPathParameterTest, DefaultParameters)
{
  EXPECT_DOUBLE_EQ(node_->get_parameter("swath_overlap").as_double(), 0.2);
  EXPECT_DOUBLE_EQ(node_->get_parameter("max_bend_angle").as_double(), 60.0);
  EXPECT_DOUBLE_EQ(node_->get_parameter("swath_record_interval").as_double(), 10.0);
  EXPECT_DOUBLE_EQ(node_->get_parameter("min_allowable_swath").as_double(), 0.0);
}

TEST_F(SurveyPathParameterTest, OutOfRangeRejection)
{
  auto reject = [&](const std::string & name, double value)
    {
      auto result = node_->set_parameter(rclcpp::Parameter(name, value));
      EXPECT_FALSE(result.successful)
        << name << " unexpectedly accepted out-of-range value " << value;
    };

  // swath_overlap: [0.0, 1.0]
  reject("swath_overlap", -0.1);
  reject("swath_overlap", 1.1);
  // max_bend_angle: [0.0, 90.0]
  reject("max_bend_angle", -1.0);
  reject("max_bend_angle", 90.5);
  // swath_record_interval: (0.0, max] -- 0.0 is below from_value 0.001
  reject("swath_record_interval", 0.0);
  // min_allowable_swath: [0.0, max]
  reject("min_allowable_swath", -1.0);
}

TEST_F(SurveyPathParameterTest, InRangeAccepted)
{
  auto accept = [&](const std::string & name, double value)
    {
      auto result = node_->set_parameter(rclcpp::Parameter(name, value));
      EXPECT_TRUE(result.successful)
        << name << " unexpectedly rejected in-range value " << value;
    };

  accept("swath_overlap", 0.5);
  accept("max_bend_angle", 45.0);
  accept("swath_record_interval", 25.0);
  accept("min_allowable_swath", 3.0);
}

// ---------------------------------------------------------------------------
// RecordSwath::SwathWidth() thresholding tests
// ---------------------------------------------------------------------------

// Builds a RecordSwath holding a single recorded swath of the given width.
// Two records separated by more than the interval force MinInterval() to
// populate m_min_record so SwathWidth(side, 0) returns a real entry.
static RecordSwath makeRecordWithSwath(double width)
{
  RecordSwath record(1.0);
  record.SetOutputSide(BoatSide::Stbd);
  // Heading 45 keeps the first record distinct from the zero-initialised
  // previous record so the duplicate-location guard does not drop it. The
  // 10 m separation exceeds the 1 m interval, forcing MinInterval() to store
  // the swath in m_min_record where SwathWidth(side, 0) can read it.
  record.AddRecord(width, width, 0.0, 0.0, 45.0, 10.0);
  record.AddRecord(width, width, 10.0, 0.0, 45.0, 10.0);
  return record;
}

TEST(RecordSwathThreshold, AboveThresholdReturnsRealWidth)
{
  RecordSwath record = makeRecordWithSwath(10.0);
  record.SetMinAllowableSwath(5.0);
  EXPECT_DOUBLE_EQ(record.SwathWidth(BoatSide::Stbd, 0), 10.0);
}

TEST(RecordSwathThreshold, BelowThresholdReturnsZero)
{
  RecordSwath record = makeRecordWithSwath(10.0);
  record.SetMinAllowableSwath(15.0);
  EXPECT_DOUBLE_EQ(record.SwathWidth(BoatSide::Stbd, 0), 0.0);
}

TEST(RecordSwathThreshold, DefaultThresholdReturnsRealWidth)
{
  // Default min_allowable_swath of 0.0 must not zero out any positive width.
  RecordSwath record = makeRecordWithSwath(10.0);
  EXPECT_DOUBLE_EQ(record.SwathWidth(BoatSide::Stbd, 0), 10.0);
}

TEST(RecordSwathThreshold, AtThresholdReturnsRealWidth)
{
  // Boundary: the threshold is a strict `<`, so a width exactly equal to
  // min_allowable_swath is still valid coverage and returns the real width.
  RecordSwath record = makeRecordWithSwath(10.0);
  record.SetMinAllowableSwath(10.0);
  EXPECT_DOUBLE_EQ(record.SwathWidth(BoatSide::Stbd, 0), 10.0);
}
