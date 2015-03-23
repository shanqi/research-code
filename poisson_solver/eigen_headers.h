// Author: Qi Shan <shanqi@cs.washington.edu>

#ifndef POISSON_SOLVER_POISSON_HEADER_H_
#define POISSON_SOLVER_POISSON_HEADER_H_

#include "third_party/Eigen/Core"
#include "third_party/Eigen/LU"
#include "third_party/Eigen/SVD"
#include "third_party/Eigen/QR"
#include "third_party/Eigen/Sparse"
#include "third_party/Eigen/SparseCore"
#include "third_party/Eigen/SparseLU"
#include "third_party/Eigen/SparseCholesky"
#include "third_party/Eigen/SparseQR"

typedef Eigen::SparseMatrix<double> SpMat; // declares a column-major sparse matrix type of double

using namespace Eigen;

#endif

