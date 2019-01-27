#ifndef MPC_H
#define MPC_H

#include <vector>
#include <array>
#include <cppad/cppad.hpp>


#include "Eigen-3.3/Eigen/Core"

class MPC {
public:
   static constexpr const size_t m_nSteps = 10; // time steps for future prediction
   static constexpr const size_t m_nActuator = 2; // Actuators are steering and throttle

   using PolynomialCoefficient = Eigen::Vector4d;
   using Actuation = std::array<double, m_nActuator * m_nSteps>;

   // evaluate polynomial
   static double polyeval(const PolynomialCoefficient & coeffs, const double x);

   class Coordinates
   {
   public:
      using Doubles = std::vector<double>;

      Coordinates(const Doubles & x, const Doubles & y)
         : m_x(x)
         , m_y(y)
      {
         assert(x.size() == y.size());
      }

      // it shall not be possible to copy this object
      Coordinates(const Coordinates &) = delete;

      size_t size() const {return m_x.size();}

      const Doubles & getXs() const { return m_x; }
      const Doubles & getYs() const { return m_y; }

      double getX(const size_t i) const { return getXs()[i]; }
      double getY(const size_t i) const { return getYs()[i]; }

      PolynomialCoefficient polyfit() const;

      // transform global coordinates to
      // car coordinate system
      void transformToCarPerspective(const double carPosX, const double carPosY, const double carOrientation);

   private:
      Doubles m_x;
      Doubles m_y;
   };

   class StateVars
   {
   public:
      double m_x; // position x
      double m_y; // position y
      double m_psi; // angle
      double m_v; // speed
      double m_cte; // cross track error
      double m_epsi; // angle error

      using StateVector = Eigen::Map<Eigen::VectorXd>;

      StateVars(const double x, const double y, const double psi, const double v, const PolynomialCoefficient & coeffs)
         : m_x(x)
         , m_y(y)
         , m_psi(psi)
         , m_v(v)
         , m_cte(0.0)
         , m_epsi(0.0)
      {
         setError(coeffs);
      }

      StateVector getVector()
      {
         return StateVector(reinterpret_cast<double*>(this), size());
      }

      static constexpr size_t size()
      {
         return sizeof(StateVars) / sizeof(double);
      }

   private:
      void setError(const PolynomialCoefficient & coeffs);
   };

   class ModelVariables
   {
   public:
      typedef CPPAD_TESTVECTOR(double) Dvector;

      ModelVariables(const StateVars & states, const size_t nSteps, const size_t nActuator, const double dt, const double ref_v);

      const size_t m_nSteps;
      const size_t m_nActuator;
      const size_t m_nVars;
      const size_t m_nContraints;
      const double m_dt;
      const double m_ref_v; // speed reference

      Dvector m_vars;
      Dvector m_varsLowerbound;
      Dvector m_varsUpperbound;
      Dvector m_constraintsLowerbound;
      Dvector m_constraintsUpperbound;

      const size_t m_x_startIdx;
      const size_t m_y_startIdx;
      const size_t m_psi_startIdx;
      const size_t m_v_startIdx;
      const size_t m_cte_startIdx;
      const size_t m_epsi_startIdx;
      const size_t m_delta_startIdx;
      const size_t m_a_startIdx;
   };

   MPC();

   virtual ~MPC();

   // Solve the model given an initial state and polynomial coefficients.
   // Return the first actuations.
   Actuation solve(const StateVars &state,
                   const PolynomialCoefficient &coeffs);

private:
   const double m_dt = 0.1; // delta time between steps
   const double m_ref_v = 50; // reference speed
};

#endif  // MPC_H
