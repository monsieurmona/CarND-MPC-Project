#include "MPC.hpp"
#include <cppad/ipopt/solve.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Householder"


using CppAD::AD;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
//   simulator around in a circle with a constant steering angle and velocity on
//   a flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
//   presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

class FG_eval {
public:
   using Dvector = CPPAD_TESTVECTOR(double);

   // Fitted polynomial coefficients
   MPC::PolynomialCoefficient m_coeffs;
   MPC::ModelVariables m_modelVariables;

   FG_eval(const MPC::PolynomialCoefficient & coeffs, const MPC::StateVars & states, const size_t nSteps, const size_t nActuator, const double dt, const double ref_v)
      : m_coeffs(coeffs)
      , m_modelVariables(states, nSteps, nActuator, dt, ref_v)
   { }

   void solve(const std::string & options, CppAD::ipopt::solve_result<Dvector> & solution)
   {
      // see an example how ipopt wors
      // https://www.coin-or.org/CppAD/Doc/ipopt_solve_get_started.cpp.htm
      // https://projects.coin-or.org/CoinBinary/export/837/CoinAll/trunk/Installer/files/doc/Short%20tutorial%20Ipopt.pdf
      //
      // https://www.khanacademy.org/math/multivariable-calculus/applications-of-multivariable-derivatives/lagrange-multipliers-and-constrained-optimization/v/constrained-optimization-introduction
      //
      CppAD::ipopt::solve<Dvector, FG_eval>(
               options,
               m_modelVariables.m_vars,
               m_modelVariables.m_varsLowerbound,
               m_modelVariables.m_varsUpperbound,
               m_modelVariables.m_constraintsLowerbound,
               m_modelVariables.m_constraintsUpperbound,
               *this,
               solution);
   }

   typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
   void operator()(ADvector& fg, const ADvector& vars)
   {
      std::cout << "solve" << std::endl;
      const size_t x_startIdx =     m_modelVariables.m_x_startIdx;
      const size_t y_startIdx =     m_modelVariables.m_y_startIdx;
      const size_t psi_startIdx =   m_modelVariables.m_psi_startIdx;
      const size_t v_startIdx =     m_modelVariables.m_v_startIdx;
      const size_t cte_startIdx =   m_modelVariables.m_cte_startIdx;
      const size_t epsi_startIdx =  m_modelVariables.m_epsi_startIdx;
      const size_t delta_startIdx = m_modelVariables.m_delta_startIdx;
      const size_t a_startIdx =     m_modelVariables.m_a_startIdx;

      const size_t nSteps = m_modelVariables.m_nSteps;
      const double ref_v  = m_modelVariables.m_ref_v;
      const double dt     = m_modelVariables.m_dt;

      // Weights to influence the importance of costs
      const double cte_CostWeight = 2000;
      const double epsi_CostWeight = 2000;
      const double v_costWeight = 1;
      const double delta_costWeight = 10;
      const double a_costWeight = 10;
      const double delta_changeCostWeight = 100;
      const double a_changeCostWeight = 10;

      // The cost is stored is the first element of `fg`.
      // Any additions to the cost is added to `fg[0]`.
      //
      // This refers to f(x), the equation that shall be minimized
      // the variable vars contain the all the x's, the state parameters
      // for all time steps
      fg[0] = 0;

      // The part of the cost based on the reference state.
      // Cost for CTE, orientation error and velocity
      for (size_t t = 0; t < nSteps; ++t) {
         fg[0] += cte_CostWeight  * CppAD::pow(vars[cte_startIdx  + t], 2);         // position error
         fg[0] += epsi_CostWeight * CppAD::pow(vars[epsi_startIdx + t], 2);         // car direction error
         fg[0] += v_costWeight    * CppAD::pow(vars[v_startIdx    + t] - ref_v, 2); // speed error
      }

      // Minimize the use of actuators.
      // Cost for steering and acceleration
      for (size_t t = 0; t < nSteps - 1; ++t) {
         fg[0] += delta_costWeight * CppAD::pow(vars[delta_startIdx + t], 2); // penalize hard steering
         fg[0] += a_costWeight     * CppAD::pow(vars[a_startIdx     + t], 2); // penalize hard acceleration
      }

      // Minimize the value gap between sequential actuations.
      // Costs steering and acceleration changes (makes the ride smoother)
      for (size_t t = 0; t < nSteps - 2; ++t) {
         fg[0] += delta_changeCostWeight * CppAD::pow(vars[delta_startIdx + t + 1] - vars[delta_startIdx + t], 2); // penalize steering changes
         fg[0] += a_changeCostWeight     * CppAD::pow(vars[a_startIdx     + t + 1] - vars[a_startIdx     + t], 2); // penalize acceleleration changes
      }

      //
      // setup model constraints
      //

      // Initial constraints
      //
      // The contraints are model equations for all states. In other words:
      // The cost function f(x) shall be minimized under the contraints of state
      // equations g(x).
      //
      //
      // We add 1 to each of the starting indices due to cost being located at
      // index 0 of `fg`.
      // This bumps up the position of all the other values.
      //
      // Previously, we have set the corresponding constraints_lowerbound and
      // the constraints_upperbound values to 0. That means the solver will
      // force this value of fg to always be 0.
      //
      fg[1 + x_startIdx]    = vars[x_startIdx];
      fg[1 + y_startIdx]    = vars[y_startIdx];
      fg[1 + psi_startIdx]  = vars[psi_startIdx];
      fg[1 + v_startIdx]    = vars[v_startIdx];
      fg[1 + cte_startIdx]  = vars[cte_startIdx];
      fg[1 + epsi_startIdx] = vars[epsi_startIdx];

      // The rest of the constraints
      for (size_t t = 1; t < nSteps; ++t) {
         // The state at time t+1 .
         AD<double> x1    = vars[x_startIdx + t];
         AD<double> y1    = vars[y_startIdx + t];
         AD<double> psi1  = vars[psi_startIdx + t];
         AD<double> v1    = vars[v_startIdx + t];
         AD<double> cte1  = vars[cte_startIdx + t];
         AD<double> epsi1 = vars[epsi_startIdx + t];

         // The state at time t.
         AD<double> x0    = vars[x_startIdx + t - 1];
         AD<double> y0    = vars[y_startIdx + t - 1];
         AD<double> psi0  = vars[psi_startIdx + t - 1];
         AD<double> v0    = vars[v_startIdx + t - 1];
         // AD<double> cte0  = vars[cte_startIdx + t - 1];
         AD<double> epsi0 = vars[epsi_startIdx + t - 1];

         // Consider the actuation at time t.
         AD<double> delta0 = vars[delta_startIdx + t - 1];
         AD<double> a0     = vars[a_startIdx + t - 1];

         if (t > 1)
         {
            // use previous actuations (to account for latency)
            a0 = vars[a_startIdx + t - 2];
            delta0 = vars[delta_startIdx + t - 2];
         }

         assert(m_coeffs.size() > 1);

         // this solves the polynomial
         // f(x) = a + bx + cx^2 ... zx^n
         //
         // AD<double> f0 = m_coeffs[0] +
         //                 m_coeffs[1] * x0 +
         //                 m_coeffs[2] * CppAD::pow(x0, 2) +
         //                 m_coeffs[3] * CppAD::pow(x0, 3);
         //
         // AD<double> psides0 = CppAD::atan(
         //                         m_coeffs[1] +
         //                         2 * m_coeffs[2] * x0 +
         //                         3 * m_coeffs[3] * CppAD::pow(x0, 2)
         //                      );

         AD<double> f0 = m_coeffs(0) + m_coeffs(1) * x0;
         AD<double> atanParm = m_coeffs[1];

         for (int parameter = 2; parameter < m_coeffs.size(); ++parameter)
         {
            const double coeff = m_coeffs(parameter);
            f0       += coeff * CppAD::pow(x0, parameter);
            atanParm += coeff * CppAD::pow(x0, parameter - 1) * parameter;
         }

         AD<double> psides0 = CppAD::atan(atanParm);

         // Here's `x` to get you started.
         // The idea here is to constraint this value to be 0.
         //
         // Recall the equations for the model:
         // x_[t+1]   = x[t]    + v[t] * cos(psi[t]) * dt
         // y_[t+1]   = y[t]    + v[t] * sin(psi[t]) * dt
         // psi_[t+1] = psi[t]  + v[t] / Lf * delta[t] * dt
         // v_[t+1]   = v[t]    + a[t] * dt
         // cte[t+1]  = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dtepsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
         // epsi[t+1] = psi[t]  - psides[t] + v[t] * delta[t] / Lf * dt

         fg[1 + x_startIdx + t]    = x1    - (x0   + v0 * CppAD::cos(psi0) * dt);
         fg[1 + y_startIdx + t]    = y1    - (y0   + v0 * CppAD::sin(psi0) * dt);
         fg[1 + psi_startIdx + t]  = psi1  - (psi0 - v0/Lf * delta0 * dt); // this is negative as the steering angle for the simulator is negative
         fg[1 + v_startIdx + t]    = v1    - (v0   + a0 * dt);
         fg[1 + cte_startIdx + t]  = cte1  - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
         fg[1 + epsi_startIdx + t] = epsi1 - ((psi0 - psides0) - v0/Lf * delta0 * dt); // this is negative as the steering angle for the simulator is negative
      }
   }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

void MPC::Coordinates::transformToCarPerspective(const double carPosX, const double carPosY, const double carOrientation)
{
   for (size_t i = 0; i < size(); ++i) {
      double & x = m_x[i];
      double & y = m_y[i];
      const double dx = x - carPosX;
      const double dy = y - carPosY;
      x = dx * cos(-carOrientation) - dy * sin(-carOrientation);
      y = dx * sin(-carOrientation) + dy * cos(-carOrientation);
   }
}

MPC::ModelVariables::ModelVariables(const StateVars & states, const size_t nSteps, const size_t nActuator, const double dt, const double ref_v)
   : m_nSteps(nSteps)
   , m_nActuator(nActuator)
   /**
    * Set the number of model variables (includes both states and inputs).
    * For example: If the state is a 6 element vector, the actuators is a 2
    *   element vector and there are 10 timesteps. The number of variables is:
    *   6 * 10 + 2 * 9
    */
   , m_nVars(m_nSteps * StateVars::size() + (m_nSteps - 1) * m_nActuator)
   , m_nContraints(StateVars::size() * m_nSteps)
   , m_dt(dt)
   , m_ref_v(ref_v)
   , m_vars(m_nVars)
   , m_varsLowerbound(m_nVars)
   , m_varsUpperbound(m_nVars)
   , m_constraintsLowerbound(m_nContraints)
   , m_constraintsUpperbound(m_nContraints)
   , m_x_startIdx(0)
   , m_y_startIdx(m_x_startIdx + m_nSteps)
   , m_psi_startIdx(m_y_startIdx + m_nSteps)
   , m_v_startIdx(m_psi_startIdx + m_nSteps)
   , m_cte_startIdx(m_v_startIdx + m_nSteps)
   , m_epsi_startIdx(m_cte_startIdx + m_nSteps)
   , m_delta_startIdx(m_epsi_startIdx + m_nSteps)
   , m_a_startIdx(m_delta_startIdx + m_nSteps - 1)
{
   // reset all variables to 0.0
   std::fill_n(m_vars.data(), m_nVars, 0.0);

   // Set the initial model variable values
   m_vars[m_x_startIdx]    = states.m_x;
   m_vars[m_y_startIdx]    = states.m_y;
   m_vars[m_psi_startIdx]  = states.m_psi;
   m_vars[m_v_startIdx]    = states.m_v;
   m_vars[m_cte_startIdx]  = states.m_cte;
   m_vars[m_epsi_startIdx] = states.m_epsi;

   // Set all non-actuators upper and lowerlimits
   // to the max negative and positive values.
   std::fill_n(&m_varsLowerbound[0], m_delta_startIdx - 0, -1.0e19);
   std::fill_n(&m_varsUpperbound[0], m_delta_startIdx - 0,  1.0e19);

   // The upper and lower limits of delta are set to -25 and 25
   // degrees (values in radians).
   // NOTE: Feel free to change this to something else.
   std::fill_n(&m_varsLowerbound[m_delta_startIdx], m_a_startIdx - m_delta_startIdx, -0.436332);
   std::fill_n(&m_varsUpperbound[m_delta_startIdx], m_a_startIdx - m_delta_startIdx,  0.436332);

   // Acceleration/decceleration upper and lower limits.
   // NOTE: Feel free to change this to something else.
   std::fill_n(&m_varsLowerbound[m_a_startIdx], m_nVars - m_a_startIdx, -1.0);
   std::fill_n(&m_varsUpperbound[m_a_startIdx], m_nVars - m_a_startIdx,  1.0);

   // Lower and upper limits for the constraints
   // Should be 0 besides initial state.
   std::fill_n(m_constraintsLowerbound.data(), m_nContraints, 0.0);
   std::fill_n(m_constraintsUpperbound.data(), m_nContraints, 0.0);

   m_constraintsLowerbound[m_x_startIdx]    = states.m_x;
   m_constraintsLowerbound[m_y_startIdx]    = states.m_y;
   m_constraintsLowerbound[m_psi_startIdx]  = states.m_psi;
   m_constraintsLowerbound[m_v_startIdx]    = states.m_v;
   m_constraintsLowerbound[m_cte_startIdx]  = states.m_cte;
   m_constraintsLowerbound[m_epsi_startIdx] = states.m_epsi;;

   m_constraintsUpperbound[m_x_startIdx]    = states.m_x;
   m_constraintsUpperbound[m_y_startIdx]    = states.m_y;
   m_constraintsUpperbound[m_psi_startIdx]  = states.m_psi;
   m_constraintsUpperbound[m_v_startIdx]    = states.m_v;
   m_constraintsUpperbound[m_cte_startIdx]  = states.m_cte;
   m_constraintsUpperbound[m_epsi_startIdx] = states.m_epsi;
}

MPC::Actuation MPC::solve(const StateVars &state,
                          const PolynomialCoefficient &coeffs) {
   bool ok = true;
   using Dvector = CPPAD_TESTVECTOR(double);

   // object that computes objective and constraints
   FG_eval fg_eval(coeffs, state, m_nSteps, m_nActuator, m_dt, m_ref_v);

   // NOTE: You don't have to worry about these options
   // options for IPOPT solver
   std::string options;

   // Uncomment this if you'd like more print information
   options += "Integer print_level  0\n";

   // NOTE: Setting sparse to true allows the solver to take advantage
   //   of sparse routines, this makes the computation MUCH FASTER. If you can
   //   uncomment 1 of these and see if it makes a difference or not but if you
   //   uncomment both the computation time should go up in orders of magnitude.
   options += "Sparse  true        forward\n";
   options += "Sparse  true        reverse\n";

   // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
   // Change this as you see fit.
   options += "Numeric max_cpu_time          0.5\n";

   // place to return solution
   CppAD::ipopt::solve_result<Dvector> solution;

   // solve the problem
   fg_eval.solve(options, solution);

   // Check some of the solution values
   ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

   // Cost
   auto cost = solution.obj_value;
   std::cout << "Cost " << cost << std::endl;

   Actuation actuation;
   assert(actuation.size() == m_nActuator * m_nSteps);

   actuation[0] = solution.x[fg_eval.m_modelVariables.m_delta_startIdx];
   actuation[1] = solution.x[fg_eval.m_modelVariables.m_a_startIdx];

   const double * const mpcX = &solution.x[fg_eval.m_modelVariables.m_x_startIdx + 1];
   const double * const mpcY = &solution.x[fg_eval.m_modelVariables.m_y_startIdx + 1];
   const size_t nMpcCoordinates = m_nSteps - 1;

   std::copy_n(mpcX, nMpcCoordinates, &actuation[2]);
   std::copy_n(mpcY, nMpcCoordinates, &actuation[2 + nMpcCoordinates]);

   return actuation;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
MPC::PolynomialCoefficient MPC::Coordinates::polyfit() const
{
   assert(size() > 0);
   PolynomialCoefficient coeffs;

   const size_t order = coeffs.size() - 1;
   assert(order > 1);

   Eigen::MatrixXd A(size(), order + 1);

   for (size_t i = 0; i < size(); ++i)
   {
      A(i, 0) = 1.0;
   }

   for (size_t j = 0; j < size(); ++j)
   {
      for (size_t i = 0; i < order; ++i)
      {
         A(j, i + 1) = A(j, i) * getX(j);
      }
   }

   Eigen::Map<const Eigen::VectorXd> y(getYs().data(), getYs().size());

   auto Q = A.householderQr();
   coeffs = Q.solve(y);
   return coeffs;
}

double MPC::polyeval(const PolynomialCoefficient & coeffs, const double x)
{
  double result = 0.0;

  for (int i = 0; i < coeffs.size(); ++i)
  {
     result += coeffs[i] * pow(x, i);
  }

  return result;
}

void MPC::StateVars::setError(const PolynomialCoefficient & coeffs)
{
   m_cte = coeffs(0); // MPC::polyeval(coeffs, 0);
   m_epsi = -atan(coeffs(1));
}
