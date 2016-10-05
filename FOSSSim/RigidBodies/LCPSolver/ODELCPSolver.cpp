#include "ODELCPSolver.h"

namespace lcputils
{

void solveLCPwithODE( const Eigen::MatrixXd& A, const Eigen::VectorXd& b, Eigen::VectorXd& lambda )
{
  assert( A.rows() == A.cols() );
  assert( b.size() == A.rows() );
  assert( lambda.size() == b.size() );

  int nrows = b.size();

  // ODE's LCP solver expects padding at the end of each row
  int ncols = dPAD(nrows);
  Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> odeA = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>::Zero(nrows,ncols);
  odeA.topLeftCorner(nrows,nrows) = A;

  // ODE's LCP solver has a different sign convention than we are using
  Eigen::VectorXd odeb = -b;

  // Slack variable. Always 0 for our purpouses.
  Eigen::VectorXd w = Eigen::VectorXd::Zero(nrows);
  // Always 0 for our purpouses.
  int nub = 0; 
  // Always 0 for our purpouses.
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(nrows);
  // Always infinity for our purpouses.
  Eigen::VectorXd hi = Eigen::VectorXd::Constant(nrows,dInfinity);

  dSolveLCPBasic(nrows,odeA.data(),lambda.data(),odeb.data(),w.data(),nub,lo.data(),hi.data());
}

}