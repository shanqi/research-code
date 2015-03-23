// Author: Qi Shan <shanqi@cs.washington.edu>

#ifndef POISSON_SOLVER_POISSON_SOLVER_H_
#define POISSON_SOLVER_POISSON_SOLVER_H_

#include "common/common.h"
#include "poisson_solver/eigen_headers.h"

namespace poisson {

class PoissonSolver2D {

public:
  PoissonSolver2D();
  PoissonSolver2D(mylong width_in, mylong height_in);
  ~PoissonSolver2D();
  
public:
  void Init(mylong width_in, mylong height_in);
  void Destroy();
  void SetDivergence(double* div_data_in);
  void SetDivergence(double* gx, double* gy);
  void SetDataConstraint(double* data_term, bool* mask, double lambda);
  void SetDataConstraintMat(bool* mask, double lambda);
  void SetDataConstraintRhs(double* data_term, bool* mask, double lambda);
  void Solve(double* xret);
  
public:   // help functions
  static void SetPoissonSpMatrix(SpMat& mat, mylong width, mylong height);
  static int Solve(SparseMatrix<double>& A, VectorXd& b, VectorXd& x);
  static int SolveDirect(SparseMatrix<double>& A, VectorXd& b, VectorXd& x);
  static int SolveSPDLLT(SparseMatrix<double>& A, VectorXd& b, VectorXd& x);
  static int SolveSPDLDLT(SparseMatrix<double>& A, VectorXd& b, VectorXd& x);
  static int SolveSPDConjugateGradient(SparseMatrix<double>& A, VectorXd& b, VectorXd& x);
  static int SolveSquareBiCGSTAB(SparseMatrix<double>& A, VectorXd& b, VectorXd& x);
  static int SolveSquareSparseLU(SparseMatrix<double>& A, VectorXd& b, VectorXd& x);
  static int SolveSparseQR(SparseMatrix<double>& A, VectorXd& b, VectorXd& x);
  
public:
  mylong width;
  mylong height;
  SpMat matA;
  VectorXd rhs;
  
};

} // namespace poisson

#endif  // POISSON_SOLVER_POISSON_SOLVER_H_

