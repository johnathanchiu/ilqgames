#include <pybind11/pybind11.h>

#include <ilqgames/examples/skeleton_example.h>
#include <ilqgames/gui/control_sliders.h>
#include <ilqgames/gui/cost_inspector.h>
#include <ilqgames/gui/top_down_renderer.h>
#include <ilqgames/solver/problem.h>
#include <ilqgames/utils/check_local_nash_equilibrium.h>
#include <ilqgames/utils/compute_strategy_costs.h>
#include <ilqgames/utils/solver_log.h>

// Optional log saving and visualization.
DEFINE_bool(save, false, "Optionally save solver logs to disk.");
DEFINE_bool(viz, true, "Visualize results in a GUI.");
DEFINE_bool(last_traj, false,
            "Should the solver only dump the last trajectory?");
DEFINE_string(experiment_name, "", "Name for the experiment.");

// Regularization.
DEFINE_double(regularization, 0.0, "Regularization.");

// Linesearch parameters.
DEFINE_bool(linesearch, true, "Should the solver linesearch?");
DEFINE_double(initial_alpha_scaling, 0.1, "Initial step size in linesearch.");
DEFINE_double(trust_region_size, 0.5, "L_infradius for trust region.");
DEFINE_double(convergence_tolerance, 0.1, "L_inf tolerance for convergence.");

int add(int i, int j) {
    return i + j;
}


PYBIND11_MODULE(example, m) {
    m.doc() = "pybind11 example plugin"; // optional module docstring

    m.def("add", &add, "A function which adds two numbers");
}

