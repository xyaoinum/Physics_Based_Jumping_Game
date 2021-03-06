#include "main.h"
#include <Eigen/MPRealSupport>

using namespace mpfr;
using namespace std;
using namespace Eigen;

void test_mpreal_support()
{
  // set precision to 256 bits (double has only 53 bits)
  mpreal::set_default_prec(256);
  typedef Matrix<mpreal,Eigen::Dynamic,Eigen::Dynamic> MatrixXmp;

  std::cerr << "epsilon =         " << NumTraits<mpreal>::epsilon() << "\n";
  std::cerr << "dummy_precision = " << NumTraits<mpreal>::dummy_precision() << "\n";
  std::cerr << "highest =         " << NumTraits<mpreal>::highest() << "\n";
  std::cerr << "lowest =          " << NumTraits<mpreal>::lowest() << "\n";

  for(int i = 0; i < g_repeat; i++) {
    int s = ei_random<int>(1,100);
    MatrixXmp A = MatrixXmp::Random(s,s);
    MatrixXmp B = MatrixXmp::Random(s,s);
    MatrixXmp S = A.adjoint() * A;
    MatrixXmp X;

    // Cholesky
    X = S.selfadjointView<Lower>().llt().solve(B);
    VERIFY_IS_APPROX((S.selfadjointView<Lower>()*X).eval(),B);

    // partial LU
    X = A.lu().solve(B);
    VERIFY_IS_APPROX((A*X).eval(),B);

    // symmetric eigenvalues
    SelfAdjointEigenSolver<MatrixXmp> eig(S);
    VERIFY_IS_EQUAL(eig.info(), Success);
    VERIFY_IS_APPROX((S.selfadjointView<Lower>() * eig.eigenvectors()),
                      eig.eigenvectors() * eig.eigenvalues().asDiagonal());
  }
}

#include "mpreal.cpp"
