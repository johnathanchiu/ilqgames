/*
 * Copyright (c) 2019, The Regents of the University of California (Regents).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this library if you have any questions.
 * Authors: David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// Roundabout merging example.
//
///////////////////////////////////////////////////////////////////////////////

#include <ilqgames/constraint/polyline2_signed_distance_constraint.h>
#include <ilqgames/constraint/proximity_constraint.h>
#include <ilqgames/constraint/single_dimension_constraint.h>
#include <ilqgames/cost/curvature_cost.h>
#include <ilqgames/cost/final_time_cost.h>
#include <ilqgames/cost/locally_convex_proximity_cost.h>
#include <ilqgames/cost/nominal_path_length_cost.h>
#include <ilqgames/cost/proximity_cost.h>
#include <ilqgames/cost/quadratic_cost.h>
#include <ilqgames/cost/quadratic_difference_cost.h>
#include <ilqgames/cost/quadratic_polyline2_cost.h>
#include <ilqgames/cost/semiquadratic_cost.h>
#include <ilqgames/cost/semiquadratic_polyline2_cost.h>
#include <ilqgames/cost/weighted_convex_proximity_cost.h>
#include <ilqgames/dynamics/concatenated_dynamical_system.h>
#include <ilqgames/dynamics/single_player_car_6d.h>
#include <ilqgames/dynamics/single_player_unicycle_4d.h>
#include <ilqgames/examples/roundabout_lane_center.h>
#include <ilqgames/examples/roundabout_merging_example.h>
#include <ilqgames/geometry/polyline2.h>
#include <ilqgames/solver/ilq_solver.h>
#include <ilqgames/solver/lq_feedback_solver.h>
#include <ilqgames/solver/problem.h>
#include <ilqgames/solver/solver_params.h>
#include <ilqgames/utils/initialize_along_route.h>
#include <ilqgames/utils/solver_log.h>
#include <ilqgames/utils/strategy.h>
#include <ilqgames/utils/types.h>

#include <math.h>
#include <memory>
#include <vector>

namespace ilqgames {

namespace {
// Time.
static constexpr Time kTimeStep = 0.1;     // s
static constexpr Time kTimeHorizon = 10.0; // s
static constexpr size_t kNumTimeSteps =
    static_cast<size_t>(kTimeHorizon / kTimeStep);

// Car inter-axle distance.
static constexpr float kInterAxleLength = 4.0; // m

// Cost weights.
static constexpr float kStateRegularization = 1.0;
static constexpr float kControlRegularization = 5.0;

static constexpr float kOmegaCostWeight = 500.0;
static constexpr float kACostWeight = 50.0;
static constexpr float kJerkCostWeight = 5.0;

static constexpr float kMaxVCostWeight = 1000.0;
static constexpr float kNominalVCostWeight = 10.0;

static constexpr float kGoalCostWeight = 0.1;
static constexpr float kLaneCostWeight = 25.0;
static constexpr float kLaneBoundaryCostWeight = 100.0;

static constexpr float kMinProximity = 6.0;
static constexpr float kP1ProximityCostWeight = 100.0;
static constexpr float kP2ProximityCostWeight = 100.0;
static constexpr float kP3ProximityCostWeight = 100.0;
static constexpr float kP4ProximityCostWeight = 100.0;
using ProxCost = ProximityCost;

static constexpr bool kOrientedRight = true;

// Lane width.
static constexpr float kLaneHalfWidth = 2.5; // m

// Goal points.
static constexpr float kP1GoalX = -6.0;  // m
static constexpr float kP1GoalY = 600.0; // m

static constexpr float kP2GoalX = 500.0; // m
static constexpr float kP2GoalY = 12.0;  // m

static constexpr float kP3GoalX = 100.0; // m
static constexpr float kP3GoalY = 16.0;  // m

// Nominal and max speed.
static constexpr float kP1MaxV = 12.0; // m/s
static constexpr float kP2MaxV = 12.0; // m/s
static constexpr float kP3MaxV = 12.0; // m/s
static constexpr float kP4MaxV = 12.0; // m/s
static constexpr float kMinV = 1.0;    // m/s

static constexpr float kP1NominalV = 10.0; // m/s
static constexpr float kP2NominalV = 10.0; // m/s
static constexpr float kP3NominalV = 10.0; // m/s
static constexpr float kP4NominalV = 10.0; // m/s

// Initial distance from roundabout.
static constexpr float kP1InitialDistanceToRoundabout = 25.0; // m
static constexpr float kP2InitialDistanceToRoundabout = 10.0; // m
static constexpr float kP3InitialDistanceToRoundabout = 25.0; // m
static constexpr float kP4InitialDistanceToRoundabout = 10.0; // m

static constexpr float kP1InitialSpeed = 3.0; // m/s
static constexpr float kP2InitialSpeed = 2.0; // m/s
static constexpr float kP3InitialSpeed = 3.0; // m/s
static constexpr float kP4InitialSpeed = 2.0; // m/s

// State dimensions.
static constexpr float kInterAxleDistance = 4.0; // m
using P1 = SinglePlayerCar6D;
using P2 = SinglePlayerCar6D;
using P3 = SinglePlayerCar6D;
using P4 = SinglePlayerCar6D;

static const Dimension kP1XIdx = P1::kPxIdx;
static const Dimension kP1YIdx = P1::kPyIdx;
static const Dimension kP1HeadingIdx = P1::kThetaIdx;
static const Dimension kP1VIdx = P1::kVIdx;
static const Dimension kP1AIdx = P1::kAIdx;

static const Dimension kP2XIdx = P1::kNumXDims + P2::kPxIdx;
static const Dimension kP2YIdx = P1::kNumXDims + P2::kPyIdx;
static const Dimension kP2HeadingIdx = P1::kNumXDims + P2::kThetaIdx;
static const Dimension kP2VIdx = P1::kNumXDims + P2::kVIdx;
static const Dimension kP2AIdx = P1::kNumXDims + P2::kAIdx;

static const Dimension kP3XIdx = P1::kNumXDims + P2::kNumXDims + P3::kPxIdx;
static const Dimension kP3YIdx = P1::kNumXDims + P2::kNumXDims + P3::kPyIdx;
static const Dimension kP3HeadingIdx =
    P1::kNumXDims + P2::kNumXDims + P3::kThetaIdx;
static const Dimension kP3VIdx = P1::kNumXDims + P2::kNumXDims + P3::kVIdx;
static const Dimension kP3AIdx = P1::kNumXDims + P2::kNumXDims + P3::kAIdx;

static const Dimension kP4XIdx =
    P1::kNumXDims + P2::kNumXDims + P3::kNumXDims + P4::kPxIdx;
static const Dimension kP4YIdx =
    P1::kNumXDims + P2::kNumXDims + P3::kNumXDims + P4::kPyIdx;
static const Dimension kP4HeadingIdx =
    P1::kNumXDims + P2::kNumXDims + P3::kNumXDims + P4::kThetaIdx;
static const Dimension kP4VIdx =
    P1::kNumXDims + P2::kNumXDims + P3::kNumXDims + P4::kVIdx;
static const Dimension kP4AIdx =
    P1::kNumXDims + P2::kNumXDims + P3::kNumXDims + P4::kAIdx;
// Control dimensions.
static const Dimension kP1OmegaIdx = 0;
static const Dimension kP1JerkIdx = 1;
static const Dimension kP2OmegaIdx = 0;
static const Dimension kP2JerkIdx = 1;
static const Dimension kP3OmegaIdx = 0;
static const Dimension kP3JerkIdx = 1;
static const Dimension kP4OmegaIdx = 0;
static const Dimension kP4JerkIdx = 1;

} // anonymous namespace

RoundaboutMergingExample::RoundaboutMergingExample(const SolverParams &params) {
  // Create dynamics.
  const std::shared_ptr<const ConcatenatedDynamicalSystem> dynamics(
      new ConcatenatedDynamicalSystem(
          {std::make_shared<P1>(kInterAxleDistance),
           std::make_shared<P2>(kInterAxleDistance),
           std::make_shared<P3>(kInterAxleDistance),
           std::make_shared<P4>(kInterAxleDistance)},
          kTimeStep));

  // Set up initial strategies and operating point.
  strategies_.reset(new std::vector<Strategy>());
  for (PlayerIndex ii = 0; ii < dynamics->NumPlayers(); ii++)
    strategies_->emplace_back(kNumTimeSteps, dynamics->XDim(),
                              dynamics->UDim(ii));

  operating_point_.reset(
      new OperatingPoint(kNumTimeSteps, dynamics->NumPlayers(), 0.0, dynamics));

  // Set up lanes for each player.
  constexpr float kAngleOffset = M_PI_2 * 0.5;
  constexpr float kWedgeSize = M_PI;
  const std::vector<float> angles = {kAngleOffset,
                                     kAngleOffset + 2.0 * M_PI / 4.0,
                                     kAngleOffset + 2.0 * 2.0 * M_PI / 4.0,
                                     kAngleOffset + 3.0 * 2.0 * M_PI / 4.0};
  const PointList2 lane1 = RoundaboutLaneCenter(
      angles[0], angles[0] + kWedgeSize, kP1InitialDistanceToRoundabout);
  const PointList2 lane2 = RoundaboutLaneCenter(
      angles[1], angles[1] + kWedgeSize, kP2InitialDistanceToRoundabout);
  const PointList2 lane3 = RoundaboutLaneCenter(
      angles[2], angles[2] + kWedgeSize, kP3InitialDistanceToRoundabout);
  const PointList2 lane4 = RoundaboutLaneCenter(
      angles[3], angles[3] + kWedgeSize, kP4InitialDistanceToRoundabout);

  const Polyline2 lane1_polyline(lane1);
  const Polyline2 lane2_polyline(lane2);
  const Polyline2 lane3_polyline(lane3);
  const Polyline2 lane4_polyline(lane4);

  // // Initialize operating points to follow these lanes at the nominal speed.
  // InitializeAlongRoute(lane1_polyline, 0.0, kP1InitialSpeed, {kP1XIdx,
  // kP1YIdx},
  //                      kTimeStep, operating_point_.get());
  // InitializeAlongRoute(lane2_polyline, 0.0, kP2InitialSpeed, {kP2XIdx,
  // kP2YIdx},
  //                      kTimeStep, operating_point_.get());
  // InitializeAlongRoute(lane3_polyline, 0.0, kP3InitialSpeed, {kP3XIdx,
  // kP3YIdx},
  //                      kTimeStep, operating_point_.get());
  // InitializeAlongRoute(lane4_polyline, 0.0, kP4InitialSpeed, {kP4XIdx,
  // kP4YIdx},
  //                      kTimeStep, operating_point_.get());

  // Set up initial state.
  x0_ = VectorXf::Zero(dynamics->XDim());
  x0_(kP1XIdx) = lane1[0].x();
  x0_(kP1YIdx) = lane1[0].y();
  x0_(kP1HeadingIdx) = lane1_polyline.Segments()[0].Heading();
  x0_(kP1VIdx) = kP1InitialSpeed;
  x0_(kP2XIdx) = lane2[0].x();
  x0_(kP2YIdx) = lane2[0].y();
  x0_(kP2HeadingIdx) = lane2_polyline.Segments()[0].Heading();
  x0_(kP2VIdx) = kP2InitialSpeed;
  x0_(kP3XIdx) = lane3[0].x();
  x0_(kP3YIdx) = lane3[0].y();
  x0_(kP3HeadingIdx) = lane3_polyline.Segments()[0].Heading();
  x0_(kP3VIdx) = kP3InitialSpeed;
  x0_(kP4XIdx) = lane4[0].x();
  x0_(kP4YIdx) = lane4[0].y();
  x0_(kP4HeadingIdx) = lane4_polyline.Segments()[0].Heading();
  x0_(kP4VIdx) = kP4InitialSpeed;

  // Set up costs for all players.
  PlayerCost p1_cost, p2_cost, p3_cost, p4_cost;

  // To edit below: (08-18-2020)

  // Stay in lanes.
  const std::shared_ptr<QuadraticPolyline2Cost> p1_lane_cost(
      new QuadraticPolyline2Cost(kLaneCostWeight, lane1_polyline,
                                 {kP1XIdx, kP1YIdx}, "LaneCenter"));
  const std::shared_ptr<SemiquadraticPolyline2Cost> p1_lane_r_cost(
      new SemiquadraticPolyline2Cost(kLaneBoundaryCostWeight, lane1_polyline,
                                     {kP1XIdx, kP1YIdx}, kLaneHalfWidth,
                                     kOrientedRight, "LaneRightBoundary"));
  const std::shared_ptr<SemiquadraticPolyline2Cost> p1_lane_l_cost(
      new SemiquadraticPolyline2Cost(kLaneBoundaryCostWeight, lane1_polyline,
                                     {kP1XIdx, kP1YIdx}, -kLaneHalfWidth,
                                     !kOrientedRight, "LaneLeftBoundary"));
  p1_cost.AddStateCost(p1_lane_cost);
  p1_cost.AddStateCost(p1_lane_r_cost);
  p1_cost.AddStateCost(p1_lane_l_cost);

  const std::shared_ptr<QuadraticPolyline2Cost> p2_lane_cost(
      new QuadraticPolyline2Cost(kLaneCostWeight, lane2_polyline,
                                 {kP2XIdx, kP2YIdx}, "LaneCenter"));
  const std::shared_ptr<SemiquadraticPolyline2Cost> p2_lane_r_cost(
      new SemiquadraticPolyline2Cost(kLaneBoundaryCostWeight, lane2_polyline,
                                     {kP2XIdx, kP2YIdx}, kLaneHalfWidth,
                                     kOrientedRight, "LaneRightBoundary"));
  const std::shared_ptr<SemiquadraticPolyline2Cost> p2_lane_l_cost(
      new SemiquadraticPolyline2Cost(kLaneBoundaryCostWeight, lane2_polyline,
                                     {kP2XIdx, kP2YIdx}, -kLaneHalfWidth,
                                     !kOrientedRight, "LaneLeftBoundary"));
  p2_cost.AddStateCost(p2_lane_cost);
  p2_cost.AddStateCost(p2_lane_r_cost);
  p2_cost.AddStateCost(p2_lane_l_cost);

  const std::shared_ptr<QuadraticPolyline2Cost> p3_lane_cost(
      new QuadraticPolyline2Cost(kLaneCostWeight, lane3_polyline,
                                 {kP3XIdx, kP3YIdx}, "LaneCenter"));
  const std::shared_ptr<SemiquadraticPolyline2Cost> p3_lane_r_cost(
      new SemiquadraticPolyline2Cost(kLaneBoundaryCostWeight, lane3_polyline,
                                     {kP3XIdx, kP3YIdx}, kLaneHalfWidth,
                                     kOrientedRight, "LaneRightBoundary"));
  const std::shared_ptr<SemiquadraticPolyline2Cost> p3_lane_l_cost(
      new SemiquadraticPolyline2Cost(kLaneBoundaryCostWeight, lane3_polyline,
                                     {kP3XIdx, kP3YIdx}, -kLaneHalfWidth,
                                     !kOrientedRight, "LaneLeftBoundary"));
  p3_cost.AddStateCost(p3_lane_cost);
  p3_cost.AddStateCost(p3_lane_r_cost);
  p3_cost.AddStateCost(p3_lane_l_cost);

  const std::shared_ptr<QuadraticPolyline2Cost> p4_lane_cost(
      new QuadraticPolyline2Cost(kLaneCostWeight, lane4_polyline,
                                 {kP4XIdx, kP4YIdx}, "LaneCenter"));
  const std::shared_ptr<SemiquadraticPolyline2Cost> p4_lane_r_cost(
      new SemiquadraticPolyline2Cost(kLaneBoundaryCostWeight, lane4_polyline,
                                     {kP4XIdx, kP4YIdx}, kLaneHalfWidth,
                                     kOrientedRight, "LaneRightBoundary"));
  const std::shared_ptr<SemiquadraticPolyline2Cost> p4_lane_l_cost(
      new SemiquadraticPolyline2Cost(kLaneBoundaryCostWeight, lane4_polyline,
                                     {kP4XIdx, kP4YIdx}, -kLaneHalfWidth,
                                     !kOrientedRight, "LaneLeftBoundary"));
  p4_cost.AddStateCost(p4_lane_cost);
  p4_cost.AddStateCost(p4_lane_r_cost);
  p4_cost.AddStateCost(p4_lane_l_cost);

  // Max/min/nominal speed costs.
  const auto p1_min_v_cost = std::make_shared<SemiquadraticCost>(
      kMaxVCostWeight, kP1VIdx, kMinV, !kOrientedRight, "MinV");
  const auto p1_max_v_cost = std::make_shared<SemiquadraticCost>(
      kMaxVCostWeight, kP1VIdx, kP1MaxV, kOrientedRight, "MaxV");
  const auto p1_nominal_v_cost = std::make_shared<QuadraticCost>(
      kNominalVCostWeight, kP1VIdx, kP1NominalV, "NominalV");
  p1_cost.AddStateCost(p1_min_v_cost);
  p1_cost.AddStateCost(p1_max_v_cost);
  p1_cost.AddStateCost(p1_nominal_v_cost);

  const auto p2_min_v_cost = std::make_shared<SemiquadraticCost>(
      kMaxVCostWeight, kP2VIdx, kMinV, !kOrientedRight, "MinV");
  const auto p2_max_v_cost = std::make_shared<SemiquadraticCost>(
      kMaxVCostWeight, kP2VIdx, kP2MaxV, kOrientedRight, "MaxV");
  const auto p2_nominal_v_cost = std::make_shared<QuadraticCost>(
      kNominalVCostWeight, kP2VIdx, kP2NominalV, "NominalV");
  p2_cost.AddStateCost(p2_min_v_cost);
  p2_cost.AddStateCost(p2_max_v_cost);
  p2_cost.AddStateCost(p2_nominal_v_cost);

  const auto p3_min_v_cost = std::make_shared<SemiquadraticCost>(
      kMaxVCostWeight, kP3VIdx, kMinV, !kOrientedRight, "MinV");
  const auto p3_max_v_cost = std::make_shared<SemiquadraticCost>(
      kMaxVCostWeight, kP3VIdx, kP3MaxV, kOrientedRight, "MaxV");
  const auto p3_nominal_v_cost = std::make_shared<QuadraticCost>(
      kNominalVCostWeight, kP3VIdx, kP3NominalV, "NominalV");
  p3_cost.AddStateCost(p3_min_v_cost);
  p3_cost.AddStateCost(p3_max_v_cost);
  p3_cost.AddStateCost(p3_nominal_v_cost);

  const auto p4_min_v_cost = std::make_shared<SemiquadraticCost>(
      kMaxVCostWeight, kP4VIdx, kMinV, !kOrientedRight, "MinV");
  const auto p4_max_v_cost = std::make_shared<SemiquadraticCost>(
      kMaxVCostWeight, kP4VIdx, kP4MaxV, kOrientedRight, "MaxV");
  const auto p4_nominal_v_cost = std::make_shared<QuadraticCost>(
      kNominalVCostWeight, kP4VIdx, kP4NominalV, "NominalV");
  p4_cost.AddStateCost(p4_min_v_cost);
  p4_cost.AddStateCost(p4_max_v_cost);
  p4_cost.AddStateCost(p4_nominal_v_cost);

  // Penalize acceleration.
  const auto p1_a_cost = std::make_shared<QuadraticCost>(kACostWeight, kP1AIdx,
                                                         0.0, "Acceleration");
  p1_cost.AddStateCost(p1_a_cost);
  const auto p2_a_cost = std::make_shared<QuadraticCost>(kACostWeight, kP1AIdx,
                                                         0.0, "Acceleration");
  p2_cost.AddStateCost(p2_a_cost);
  const auto p3_a_cost = std::make_shared<QuadraticCost>(kACostWeight, kP1AIdx,
                                                         0.0, "Acceleration");
  p3_cost.AddStateCost(p3_a_cost);
  const auto p4_a_cost = std::make_shared<QuadraticCost>(kACostWeight, kP1AIdx,
                                                         0.0, "Acceleration");
  p4_cost.AddStateCost(p4_a_cost);

  // Penalize control effort.
  const auto p1_omega_cost = std::make_shared<QuadraticCost>(
      kOmegaCostWeight, kP1OmegaIdx, 0.0, "Steering");
  const auto p1_j_cost =
      std::make_shared<QuadraticCost>(kJerkCostWeight, kP1JerkIdx, 0.0, "Jerk");
  p1_cost.AddControlCost(0, p1_omega_cost);
  p1_cost.AddControlCost(0, p1_j_cost);

  const auto p2_omega_cost = std::make_shared<QuadraticCost>(
      kOmegaCostWeight, kP2OmegaIdx, 0.0, "Steering");
  const auto p2_j_cost =
      std::make_shared<QuadraticCost>(kJerkCostWeight, kP2JerkIdx, 0.0, "Jerk");
  p2_cost.AddControlCost(1, p2_omega_cost);
  p2_cost.AddControlCost(1, p2_j_cost);

  const auto p3_omega_cost = std::make_shared<QuadraticCost>(
      kOmegaCostWeight, kP3OmegaIdx, 0.0, "Steering");
  const auto p3_j_cost =
      std::make_shared<QuadraticCost>(kJerkCostWeight, kP3JerkIdx, 0.0, "Jerk");
  p3_cost.AddControlCost(2, p3_omega_cost);
  p3_cost.AddControlCost(2, p3_j_cost);

  const auto p4_omega_cost = std::make_shared<QuadraticCost>(
      kOmegaCostWeight, kP4OmegaIdx, 0.0, "Steering");
  const auto p4_j_cost =
      std::make_shared<QuadraticCost>(kJerkCostWeight, kP4JerkIdx, 0.0, "Jerk");
  p4_cost.AddControlCost(3, p4_omega_cost);
  p4_cost.AddControlCost(3, p4_j_cost);

  // Pairwise proximity costs.
  const std::shared_ptr<ProxCost> p1p2_proximity_cost(
      new ProxCost(kP1ProximityCostWeight, {kP1XIdx, kP1YIdx},
                   {kP2XIdx, kP2YIdx}, kMinProximity, "ProximityP2"));
  const std::shared_ptr<ProxCost> p1p3_proximity_cost(
      new ProxCost(kP1ProximityCostWeight, {kP1XIdx, kP1YIdx},
                   {kP3XIdx, kP3YIdx}, kMinProximity, "ProximityP3"));
  const std::shared_ptr<ProxCost> p1p4_proximity_cost(
      new ProxCost(kP1ProximityCostWeight, {kP1XIdx, kP1YIdx},
                   {kP4XIdx, kP4YIdx}, kMinProximity, "ProximityP4"));
  p1_cost.AddStateCost(p1p2_proximity_cost);
  //  p1_cost.AddStateCost(p1p3_proximity_cost);
  p1_cost.AddStateCost(p1p4_proximity_cost);

  // p2p1_proximity cost:  Modified to include adversarial phase

  const std::shared_ptr<InitialTimeCost> p2p1_initial_proximity_cost(
      new InitialTimeCost(
          std::shared_ptr<QuadraticDifferenceCost>(new QuadraticDifferenceCost(
              kP2ProximityCostWeight, {kP2XIdx, kP2YIdx}, {kP1XIdx, kP1YIdx})),
          params.adversarial_time, "InitialProximityCostP1"));
  p2_cost.AddStateCost(p2p1_initial_proximity_cost);
  initial_time_costs_.push_back(p2p1_initial_proximity_cost);

  const std::shared_ptr<FinalTimeCost> p2p1_final_proximity_cost(
      new FinalTimeCost(std::shared_ptr<ProxCost>(new ProxCost(
                            kP2ProximityCostWeight, {kP2XIdx, kP2YIdx},
                            {kP1XIdx, kP1YIdx}, kMinProximity)),

                        params.adversarial_time, "FinalProximityCostP1"));
  p2_cost.AddStateCost(p2p1_final_proximity_cost);
  final_time_costs_.push_back(p2p1_final_proximity_cost);

  // const std::shared_ptr<ProxCost> p2p1_proximity_cost(
  //     new ProxCost(kP2ProximityCostWeight, {kP2XIdx, kP2YIdx},
  //                  {kP1XIdx, kP1YIdx}, kMinProximity, "ProximityP1"));
  // p2_cost.AddStateCost(p2p1_proximity_cost);

  const std::shared_ptr<ProxCost> p2p3_proximity_cost(
      new ProxCost(kP2ProximityCostWeight, {kP2XIdx, kP2YIdx},
                   {kP3XIdx, kP3YIdx}, kMinProximity, "ProximityP3"));
  const std::shared_ptr<ProxCost> p2p4_proximity_cost(
      new ProxCost(kP2ProximityCostWeight, {kP2XIdx, kP2YIdx},
                   {kP4XIdx, kP4YIdx}, kMinProximity, "ProximityP4"));

  p2_cost.AddStateCost(p2p3_proximity_cost);
  //  p2_cost.AddStateCost(p2p4_proximity_cost);

  // p3p1_proximity_cost:  Modified to include adversarial phase

  const std::shared_ptr<InitialTimeCost> p3p1_initial_proximity_cost(
      new InitialTimeCost(
          std::shared_ptr<QuadraticDifferenceCost>(new QuadraticDifferenceCost(
              kP3ProximityCostWeight, {kP3XIdx, kP3YIdx}, {kP1XIdx, kP1YIdx})),
          params.adversarial_time, "InitialProximityCostP1"));
  p3_cost.AddStateCost(p3p1_initial_proximity_cost);
  initial_time_costs_.push_back(p3p1_initial_proximity_cost);

  const std::shared_ptr<FinalTimeCost> p3p1_final_proximity_cost(
      new FinalTimeCost(std::shared_ptr<ProxCost>(new ProxCost(
                            kP3ProximityCostWeight, {kP3XIdx, kP3YIdx},
                            {kP1XIdx, kP1YIdx}, kMinProximity)),
                        params.adversarial_time, "FinalProximityCostP1"));
  p3_cost.AddStateCost(p3p1_final_proximity_cost);
  final_time_costs_.push_back(p3p1_final_proximity_cost);

  // const std::shared_ptr<ProxCost> p3p1_proximity_cost(
  //     new ProxCost(kP3ProximityCostWeight, {kP3XIdx, kP3YIdx},
  //                  {kP1XIdx, kP1YIdx}, kMinProximity, "ProximityP1"));
  // p3_cost.AddStateCost(p3p1_proximity_cost);

  const std::shared_ptr<ProxCost> p3p2_proximity_cost(
      new ProxCost(kP3ProximityCostWeight, {kP3XIdx, kP3YIdx},
                   {kP2XIdx, kP2YIdx}, kMinProximity, "ProximityP2"));
  const std::shared_ptr<ProxCost> p3p4_proximity_cost(
      new ProxCost(kP3ProximityCostWeight, {kP3XIdx, kP3YIdx},
                   {kP4XIdx, kP4YIdx}, kMinProximity, "ProximityP4"));
  //  p3_cost.AddStateCost(p3p1_proximity_cost);
  p3_cost.AddStateCost(p3p2_proximity_cost);
  p3_cost.AddStateCost(p3p4_proximity_cost);

  // p4p1_proximity_cost:  Modified to include adversarial phase

  const std::shared_ptr<InitialTimeCost> p4p1_initial_proximity_cost(
      new InitialTimeCost(
          std::shared_ptr<QuadraticDifferenceCost>(new QuadraticDifferenceCost(
              kP4ProximityCostWeight, {kP4XIdx, kP4YIdx}, {kP1XIdx, kP1YIdx})),
          params.adversarial_time, "InitialProximityCostP1"));
  p4_cost.AddStateCost(p4p1_initial_proximity_cost);
  initial_time_costs_.push_back(p4p1_initial_proximity_cost);

  const std::shared_ptr<FinalTimeCost> p4p1_final_proximity_cost(
      new FinalTimeCost(std::shared_ptr<ProxCost>(new ProxCost(
                            kP4ProximityCostWeight, {kP4XIdx, kP4YIdx},
                            {kP1XIdx, kP1YIdx}, kMinProximity)),
                        params.adversarial_time, "FinalProximityCostP1"));
  p4_cost.AddStateCost(p4p1_final_proximity_cost);
  final_time_costs_.push_back(p4p1_final_proximity_cost);

  // const std::shared_ptr<ProxCost> p4p1_proximity_cost(
  //     new ProxCost(kP4ProximityCostWeight, {kP4XIdx, kP4YIdx},
  //                  {kP1XIdx, kP1YIdx}, kMinProximity, "ProximityP1"));
  // p4_cost.AddStateCost(p4p1_proximity_cost);

  const std::shared_ptr<ProxCost> p4p2_proximity_cost(
      new ProxCost(kP4ProximityCostWeight, {kP4XIdx, kP4YIdx},
                   {kP2XIdx, kP2YIdx}, kMinProximity, "ProximityP2"));
  const std::shared_ptr<ProxCost> p4p3_proximity_cost(
      new ProxCost(kP4ProximityCostWeight, {kP4XIdx, kP4YIdx},
                   {kP3XIdx, kP3YIdx}, kMinProximity, "ProximityP3"));

  //  p4_cost.AddStateCost(p4p2_proximity_cost);
  p4_cost.AddStateCost(p4p3_proximity_cost);

  // Set up solver.
  // SolverParams revised_params(params);
  // revised_params.trust_region_dimensions = {kP1XIdx, kP1YIdx, kP2XIdx,
  // kP2YIdx,
  //                                           kP3XIdx, kP3YIdx, kP4XIdx,
  //                                           kP4YIdx};

  solver_.reset(new ILQSolver(dynamics, {p1_cost, p2_cost, p3_cost, p4_cost},
                              kTimeHorizon, params));
}

inline std::vector<float>
RoundaboutMergingExample::Xs(const VectorXf &x) const {
  return {x(kP1XIdx), x(kP2XIdx), x(kP3XIdx), x(kP4XIdx)};
}

inline std::vector<float>
RoundaboutMergingExample::Ys(const VectorXf &x) const {
  return {x(kP1YIdx), x(kP2YIdx), x(kP3YIdx), x(kP4YIdx)};
}

inline std::vector<float>
RoundaboutMergingExample::Thetas(const VectorXf &x) const {
  return {x(kP1HeadingIdx), x(kP2HeadingIdx), x(kP3HeadingIdx),
          x(kP4HeadingIdx)};
}

} // namespace ilqgames
