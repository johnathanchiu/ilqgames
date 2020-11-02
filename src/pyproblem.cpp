#include <ilqgames/constraint/polyline2_signed_distance_constraint.h>
#include <ilqgames/constraint/single_dimension_constraint.h>
#include <ilqgames/cost/extreme_value_cost.h>
#include <ilqgames/cost/proximity_cost.h>
#include <ilqgames/cost/quadratic_cost.h>
#include <ilqgames/cost/quadratic_polyline2_cost.h>
#include <ilqgames/dynamics/concatenated_dynamical_system.h>
#include <ilqgames/dynamics/single_player_car_5d.h>
#include <ilqgames/geometry/draw_shapes.h>
#include <ilqgames/geometry/polyline2.h>
#include <ilqgames/solver/ilq_solver.h>
#include <ilqgames/solver/problem.h>
#include <ilqgames/solver/solver_params.h>
#include <ilqgames/utils/types.h>
#include <ilqgames/simulator/pyproblem.h>

#include <math.h>
#include <memory>
#include <vector>

namespace ilqgames {
namespace {
// Time.
static constexpr Time kTimeStep = 0.1;      // s
static constexpr Time kTimeHorizon = 10.0;  // s
static constexpr size_t kNumTimeSteps =
    static_cast<size_t>(kTimeHorizon / kTimeStep);

// Input contraints.
static constexpr float kOmegaMax = 1.5;  // rad/s
static constexpr float kAMax = 4.0;      // m/s

// Cost weights.
// Step 1. Try changing these.
static constexpr float kOmegaCostWeight = 25.0;
static constexpr float kACostWeight = 15.0;
static constexpr float kNominalVCostWeight = 10.0;
static constexpr float kLaneCostWeight = 25.0;
static constexpr float kProximityCostWeight = 100.0;

// Nominal speed.
// Step 2. Try changing these.
static constexpr float kP1NominalV = 27.8;  // m/s

// Initial state.
static constexpr float kP1InitialX = -25163.9477;    // m
static constexpr float kP1InitialY = 33693.3450;  // m

static constexpr float kP1InitialTheta = M_PI_2;   // rad

static constexpr float kP1InitialV = 0.0;  // m/s


// runway dimensions
static constexpr float runwayOriginX = -25163.9477;
static constexpr float runwayOriginY = 33693.3450;

static constexpr float runwayEndX = -22742.57617;
static constexpr float runwayEndY = 31956.02344;

// State dimensions.
using P1 = SinglePlayerCar5D;

static const Dimension kP1XIdx = P1::kPxIdx;
static const Dimension kP1YIdx = P1::kPyIdx;
static const Dimension kP1ThetaIdx = P1::kThetaIdx;
static const Dimension kP1VIdx = P1::kVIdx;
}  // anonymous namespace

PyProblem::PyProblem(const SolverParams& params) {

  static constexpr float kInterAxleDistance = 4.0;  // m
  const std::shared_ptr<const ConcatenatedDynamicalSystem> dynamics(
      new ConcatenatedDynamicalSystem(
          {std::make_shared<P1>(kInterAxleDistance)},
          kTimeStep));
    
  dimension = dynamics->XDim();

  // Set up initial state. Initially, this is zero, but then we override
  // individual dimensions to match the desired initial conditions above.
  x0_ = VectorXf::Zero(dimension);
  x0_(kP1XIdx) = kP1InitialX;
  x0_(kP1YIdx) = kP1InitialY;
  x0_(kP1ThetaIdx) = kP1InitialTheta;
  x0_(kP1VIdx) = kP1InitialV;

  // Set up initial strategies and operating point. A "Strategy" is a
  // time-varying affine feedback law for each player, and an "OperatingPoint"
  // is just a fixed state/control trajectory of the game. Here, we initialize
  // both to zero (of the correct dimension).
  strategies_.reset(new std::vector<Strategy>());
  for (PlayerIndex ii = 0; ii < dynamics->NumPlayers(); ii++)
    strategies_->emplace_back(kNumTimeSteps, dynamics->XDim(),
                              dynamics->UDim(ii));

  operating_point_.reset(
      new OperatingPoint(kNumTimeSteps, dynamics->NumPlayers(), 0.0, dynamics));

  // Set up costs for all players. These are containers for holding each
  // player's constituent cost functions and constraints that hold pointwise in
  // time and can apply to either state or control (for *any* player).
  // These costs can also build in regularization on the state or the control,
  // which essentially boils down to adding a scaled identity matrix to each's
  // Hessian.
  PlayerCost p1_cost("P1");

  // Quadratic control costs.
  const auto p1_omega_cost = std::make_shared<QuadraticCost>(
      kOmegaCostWeight, P1::kOmegaIdx, 0.0, "OmegaCost");
  const auto p1_a_cost = std::make_shared<QuadraticCost>(
      kACostWeight, P1::kAIdx, 0.0, "AccelerationCost");
  p1_cost.AddControlCost(0, p1_omega_cost);
  p1_cost.AddControlCost(0, p1_a_cost);

  // Constrain each control input to lie in an interval.
  // Step 3. Try uncommenting these blocks.
  // const auto p1_omega_max_constraint =
  //     std::make_shared<SingleDimensionConstraint>(
  //         P1::kOmegaIdx, kOmegaMax, false, "Omega Constraint (Max)");
  // const auto p1_omega_min_constraint =
  //     std::make_shared<SingleDimensionConstraint>(
  //         P1::kOmegaIdx, -kOmegaMax, true, "Omega Constraint (Min)");
  // const auto p1_a_max_constraint = std::make_shared<SingleDimensionConstraint>(
  //     P1::kAIdx, kAMax, false, "Acceleration Constraint (Max)");
  // const auto p1_a_min_constraint = std::make_shared<SingleDimensionConstraint>(
  //     P1::kAIdx, -kAMax, true, "Acceleration Constraint (Min)");
  // p1_cost.AddControlConstraint(0, p1_omega_max_constraint);
  // p1_cost.AddControlConstraint(0, p1_omega_min_constraint);
  // p1_cost.AddControlConstraint(0, p1_a_max_constraint);
  // p1_cost.AddControlConstraint(0, p1_a_min_constraint);

  // Encourage each player to go a given nominal speed.
  const auto p1_nominal_v_cost = std::make_shared<QuadraticCost>(
      kNominalVCostWeight, kP1VIdx, kP1NominalV, "NominalV");
  p1_cost.AddStateCost(p1_nominal_v_cost);

  // Encourage each player to remain near the lane center. Could also add
  // constraints to stay in the lane.
  const Polyline2 lane1(
      {Point2(runwayOriginX, runwayOriginY), Point2(runwayEndX, runwayEndY)});
    
  const std::shared_ptr<QuadraticPolyline2Cost> p1_lane_cost(
      new QuadraticPolyline2Cost(kLaneCostWeight, lane1, {kP1XIdx, kP1YIdx},
                                 "LaneCenter"));
  p1_cost.AddStateCost(p1_lane_cost);

  // Set up solver.
  solver_.reset(
      new ILQSolver(dynamics, {p1_cost}, kTimeHorizon, params));
}

}  // namespace ilqgames

