#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <ilqgames/solver/problem.h>
#include <ilqgames/utils/solver_log.h>
#include <ilqgames/dynamics/single_player_car_5d.h>
#include <ilqgames/examples/receding_horizon_simulator.h>
#include <ilqgames/solver/solution_splicer.h>
#include <ilqgames/simulator/pyproblem.h>

#include <optional>


ilqgames::SolverLog solver(ilqgames::PyProblem& problem, std::optional<py::list> states) {
    
    if (states) {
        Eigen::VectorXf x0 = Eigen::VectorXf::Zero(problem.dimension);
        x0(ilqgames::SinglePlayerCar5D::kPxIdx) = states.value()[ilqgames::SinglePlayerCar5D::kPxIdx].cast<float>();
        x0(ilqgames::SinglePlayerCar5D::kPyIdx) = states.value()[ilqgames::SinglePlayerCar5D::kPyIdx].cast<float>();
        x0(ilqgames::SinglePlayerCar5D::kThetaIdx) = states.value()[ilqgames::SinglePlayerCar5D::kThetaIdx].cast<float>();
        x0(ilqgames::SinglePlayerCar5D::kVIdx) = states.value()[ilqgames::SinglePlayerCar5D::kVIdx].cast<float>();
        
        problem.ResetInitialState(x0);
    }
    
    return *(problem.Solve());
}

std::vector<std::vector<float>> getOperatingPointStates(ilqgames::OperatingPoint& op) {
    std::vector<Eigen::VectorXf> states = op.xs;
    std::vector<float> headings, velocities;

    for (int i = 0; i < states.size(); ++i) {
        headings.push_back(states[i](ilqgames::SinglePlayerCar5D::kThetaIdx));
        velocities.push_back(states[i](ilqgames::SinglePlayerCar5D::kVIdx));
    }

    std::vector<std::vector<float>> unwrappedStates;
    unwrappedStates.push_back(headings);
    unwrappedStates.push_back(velocities);

    return unwrappedStates;
}


PYBIND11_MODULE(ilqgamespy, m) {
    py::class_<ilqgames::SolverParams>(m, "SolverParams")
        .def(py::init<>())
        .def_readwrite("convergence_tolerance", &ilqgames::SolverParams::convergence_tolerance)
        .def_readwrite("max_solver_iters", &ilqgames::SolverParams::max_solver_iters)
        .def_readwrite("linesearch", &ilqgames::SolverParams::linesearch)
        .def_readwrite("initial_alpha_scaling", &ilqgames::SolverParams::initial_alpha_scaling)
        .def_readwrite("geometric_alpha_scaling", &ilqgames::SolverParams::geometric_alpha_scaling)
        .def_readwrite("max_backtracking_steps", &ilqgames::SolverParams::max_backtracking_steps)
        .def_readwrite("enforce_constraints_in_linesearch", &ilqgames::SolverParams::enforce_constraints_in_linesearch)
        .def_readwrite("trust_region_size", &ilqgames::SolverParams::trust_region_size)
        .def_readwrite("trust_region_dimensions", &ilqgames::SolverParams::trust_region_dimensions)
        .def_readwrite("barrier_scaling_iters", &ilqgames::SolverParams::geometric_alpha_scaling)
        .def_readwrite("geometric_barrier_scaling", &ilqgames::SolverParams::geometric_alpha_scaling)
        .def_readwrite("open_loop", &ilqgames::SolverParams::open_loop)
        .def_readwrite("state_regularization", &ilqgames::SolverParams::state_regularization)
        .def_readwrite("control_regularization", &ilqgames::SolverParams::control_regularization);
    
    py::class_<ilqgames::PyProblem>(m, "PyProblem")
        .def(py::init<ilqgames::SolverParams &>())
        .def_readwrite("dimension", &ilqgames::PyProblem::dimension);
    
    py::class_<ilqgames::OperatingPoint>(m, "OperatingPoint")
        .def(py::init<const size_t &, const ilqgames::PlayerIndex &, const ilqgames::Time &>())
        .def_readwrite("xs", &ilqgames::OperatingPoint::xs)
        .def_readwrite("us", &ilqgames::OperatingPoint::us);
    
    py::class_<ilqgames::Strategy>(m, "Strategy")
        .def(py::init<const size_t &, const ilqgames::Dimension &, const ilqgames::Dimension &>())
        .def_readwrite("Ps", &ilqgames::Strategy::Ps)
        .def_readwrite("alphas", &ilqgames::Strategy::alphas);

    py::class_<ilqgames::SolutionSplicer>(m, "SolutionSplicer")
        .def(py::init<const ilqgames::SolverLog &>())
        .def("Splice", &ilqgames::SolutionSplicer::Splice)
        .def("ContainsTime", &ilqgames::SolutionSplicer::ContainsTime)
        .def("CurrentStrategies", &ilqgames::SolutionSplicer::CurrentStrategies)
        .def("CurrentOperatingPoint", &ilqgames::SolutionSplicer::CurrentOperatingPoint);

    py::class_<ilqgames::SolverLog>(m, "SolverLog")
        .def(py::init<const ilqgames::Time &>())
        .def("FinalOperatingPoint", &ilqgames::SolverLog::FinalOperatingPoint);
    
    m.def("solver", &solver, "ilqgames solver", py::arg("problem"), py::arg("states") = py::none());
    m.def("getStates", &getOperatingPointStates, "get states from operating point");
    
    m.attr("kPxIdx") = ilqgames::SinglePlayerCar5D::kPxIdx;
    m.attr("kPyIdx") = ilqgames::SinglePlayerCar5D::kPyIdx;
    m.attr("kThetaIdx") = ilqgames::SinglePlayerCar5D::kThetaIdx;
    m.attr("kVIdx") = ilqgames::SinglePlayerCar5D::kVIdx;

}

