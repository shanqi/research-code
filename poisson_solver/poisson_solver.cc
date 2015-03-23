// Author: Qi Shan <shanqi@cs.washington.edu>

#include "poisson_solver/poisson_solver.h"

//#define _POISSON_DISPLAY_

namespace poisson {
  
PoissonSolver2D::PoissonSolver2D() {
  Init(0,0);
}
  
PoissonSolver2D::PoissonSolver2D(mylong width_in, mylong height_in) {
  Init(width_in, height_in);
}

PoissonSolver2D::~PoissonSolver2D() {
}

void PoissonSolver2D::Init(mylong width_in, mylong height_in) {
  width = width_in;
  height = height_in;
  mylong nele_c = width_in*height_in;
  
  rhs = VectorXd(nele_c);
  SetPoissonSpMatrix(matA, width_in, height_in);
  
}
  
void PoissonSolver2D::Destroy() {
}

typedef Eigen::Triplet<double> dtlet;

void PoissonSolver2D::SetPoissonSpMatrix(SpMat& mat, mylong width, mylong height) {
  mylong nele_c = width*height;
  mat = SpMat(nele_c,nele_c);
  
  mylong estimation_of_entries = nele_c*5;
  std::vector<dtlet> tripletList;
  tripletList.reserve(estimation_of_entries);
  
  for (mylong iy=0; iy<height; ++iy) {
    mylong xstart = iy*width;
    for (mylong ix=0; ix<width; ++ix) {
      mylong tid = ix + xstart;
      
      mylong tidxm1 = (ix==0) ? tid : tid-1;
      mylong tidxp1 = (ix==(width-1)) ? tid : tid+1;
      mylong tidym1 = (iy==0) ? tid : tid-width;
      mylong tidyp1 = (iy==(height-1)) ? tid : tid+width;
      
      tripletList.push_back(dtlet(tid,tid,4));
      tripletList.push_back(dtlet(tid,tidxm1,-1));
      tripletList.push_back(dtlet(tid,tidxp1,-1));
      tripletList.push_back(dtlet(tid,tidym1,-1));
      tripletList.push_back(dtlet(tid,tidyp1,-1));
    }
  }
  
  mat.setFromTriplets(tripletList.begin(), tripletList.end());
}
  
void PoissonSolver2D::SetDivergence(double* gx, double* gy) {
  mylong nele_c = width*height;
  double* div_data = new double [nele_c];
  memset(div_data, 0, sizeof(double)*nele_c);
  for (mylong iy=0; iy<height; ++iy) {
    mylong xstart = iy*width;
    for (mylong ix=1; ix<width; ++ix) {
      mylong tid = xstart+ix;
      div_data[tid] = gx[tid-1] - gx[tid];
    }
  }
  for (mylong iy=1; iy<height; ++iy) {
    mylong xstart = iy*width;
    for (mylong ix=0; ix<width; ++ix) {
      mylong tid = xstart+ix;
      div_data[tid] += (gy[tid-width]-gy[tid]);
    }
  }
  SetDivergence(div_data);
  
  SAFE_DELETE_ARRAY(div_data);
}

void PoissonSolver2D::SetDivergence(double* div_data_in) {
#ifdef _POISSON_DISPLAY_
  my_printf(2, "SetDivergence\n");
#endif
  mylong nele_c = width*height;
#pragma omp parallel for num_threads(NUM_THREADS)
  for (mylong j=0; j<nele_c; ++j) {
    rhs(j) = div_data_in[j];
  }
}
  
void PoissonSolver2D::SetDataConstraint(double* data_term, bool* mask, double lambda) {
#ifdef _POISSON_DISPLAY_
  my_printf(2, "SetDataConstraint\n");
#endif
  mylong nele_c = width*height;
  for (mylong j=0; j<nele_c; ++j) {
    if (mask[j]) {
      rhs(j) = rhs(j) + lambda*data_term[j];
      matA.insert(j, j) = lambda;
    }
  }
}

void PoissonSolver2D::SetDataConstraintMat(bool* mask, double lambda) {
  mylong nele_c = width*height;
  for (mylong j=0; j<nele_c; ++j) {
    if (mask[j]) {
      //matA.insert(j, j) = lambda;
      matA.coeffRef(j,j) += lambda;
    }
  }
}

void PoissonSolver2D::SetDataConstraintRhs(double* data_term, bool* mask, double lambda) {
#ifdef _POISSON_DISPLAY_
  my_printf(2, "SetDataConstraintRhs\n");
#endif
  mylong nele_c = width*height;
  for (mylong j=0; j<nele_c; ++j) {
    if (mask[j]) {
      rhs(j) = rhs(j) + lambda*data_term[j];
    }
  }
}

void PoissonSolver2D::Solve(double* xret) {
#ifdef _POISSON_DISPLAY_
  my_printf(2, "Solve the Poisson equation\n");
  double ttime = getCPUTime();
#endif
  mylong nele_c = width*height;
  VectorXd x(nele_c);
  Solve(matA, rhs, x);
#pragma omp parallel for num_threads(NUM_THREADS)
  for (mylong j=0; j<nele_c; ++j) {
    xret[j] = x(j);
  }
#ifdef _POISSON_DISPLAY_
  my_printf(2, "Done sovling the Poisson equation. CPU time %f sec.\n", getCPUTime()-ttime);
#endif
}
  
int PoissonSolver2D::Solve(SparseMatrix<double>& A, VectorXd& b, VectorXd& x) {
  //return SolveSparseQR(A, b, x);
  return SolveDirect(A, b, x);
}
  
int PoissonSolver2D::SolveDirect(SparseMatrix<double>& A, VectorXd& b, VectorXd& x) {
  Eigen::SimplicialCholesky<SpMat> chol(A);  // performs a Cholesky factorization of A
  x = chol.solve(b);         // use the factorization to solve for the given right hand side
  return 0;
}
  
int PoissonSolver2D::SolveSPDLLT(SparseMatrix<double>& A, VectorXd& b, VectorXd& x) {
  SimplicialLLT<SparseMatrix<double>> solver;
  solver.compute(A);
  if(solver.info()!=Success) {
    // decomposition failed
    return -1;
  }
  x = solver.solve(b);
  if(solver.info()!=Success) {
    // solving failed
    return -2;
  }
  return 0;
}
  
int PoissonSolver2D::SolveSPDLDLT(SparseMatrix<double>& A, VectorXd& b, VectorXd& x) {
  SimplicialLDLT<SparseMatrix<double>> solver;
  solver.compute(A);
  if(solver.info()!=Success) {
    // decomposition failed
    return -1;
  }
  x = solver.solve(b);
  if(solver.info()!=Success) {
    // solving failed
    return -2;
  }
  return 0;
}

int PoissonSolver2D::SolveSPDConjugateGradient(SparseMatrix<double>& A, VectorXd& b, VectorXd& x) {
  ConjugateGradient<SparseMatrix<double>> solver;
  solver.compute(A);
  if(solver.info()!=Success) {
    // decomposition failed
    return -1;
  }
  x = solver.solve(b);
  if(solver.info()!=Success) {
    // solving failed
    return -2;
  }
  return 0;
}
  
int PoissonSolver2D::SolveSquareBiCGSTAB(SparseMatrix<double>& A, VectorXd& b, VectorXd& x) {
  BiCGSTAB<SparseMatrix<double>> solver;
  solver.compute(A);
  if(solver.info()!=Success) {
    // decomposition failed
    return -1;
  }
  x = solver.solve(b);
  if(solver.info()!=Success) {
    // solving failed
    return -2;
  }
  return 0;
}
  
int PoissonSolver2D::SolveSquareSparseLU(SparseMatrix<double>& A, VectorXd& b, VectorXd& x) {
  SparseLU<SparseMatrix<double>> solver;
  solver.compute(A);
  if(solver.info()!=Success) {
    // decomposition failed
    return -1;
  }
  x = solver.solve(b);
  if(solver.info()!=Success) {
    // solving failed
    return -2;
  }
  return 0;
}

int PoissonSolver2D::SolveSparseQR(SparseMatrix<double>& A, VectorXd& b, VectorXd& x) {
  SparseQR<SparseMatrix<double>, COLAMDOrdering<int>> solver;
  //SparseQR<SparseMatrix<double>, NaturalOrdering<int>> solver;
  solver.compute(A);
  if(solver.info()!=Success) {
    // decomposition failed
    return -1;
  }
  x = solver.solve(b);
  if(solver.info()!=Success) {
    // solving failed
    return -2;
  }
  return 0;
}

}       // namespace poisson

