#ifndef ILQGAMES_SOLVER_TOP_DOWN_RENDERABLE_PROBLEM_H
#define ILQGAMES_SOLVER_TOP_DOWN_RENDERABLE_PROBLEM_H

#include <ilqgames/solver/problem.h>
#include <ilqgames/utils/types.h>
#include <ilqgames/solver/solver_params.h>

namespace ilqgames {

class PyProblem : public Problem {
 public:
  virtual ~PyProblem() {}
  PyProblem(const SolverParams& params);
    
  Dimension dimension;

};  // class PyProblem

}  // namespace ilqgames

#endif
