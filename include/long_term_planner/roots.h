#include <iostream>
#include <iomanip>
#include <limits>
#include <tuple>
#include <complex>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues> 

#ifndef roots_H
#define roots_H

namespace long_term_planner {

/**
 * @brief Calculate all roots of a polynomial using the companion matrix.
 * 
 * @tparam T 
 * @param poly_vals vector of polynomial factors starting with the highest exponent.
 * @return Eigen::Matrix<T, Eigen::Dynamic, 1> Vector of roots.
 */
template <class T>
Eigen::Matrix<std::complex<T>, Eigen::Dynamic, Eigen::Dynamic> roots(Eigen::Matrix<T, Eigen::Dynamic, 1> poly_vals) {
  using namespace Eigen;
  int poly_size = poly_vals.size();
  // Build companion matrix as in https://en.wikipedia.org/wiki/Companion_matrix
  // and https://de.mathworks.com/help/matlab/ref/roots.html
  Matrix<T, Dynamic, Dynamic> C = Array<T, Dynamic, Dynamic>::Zero(poly_size-1, poly_size-1);
  C.bottomLeftCorner(poly_size-2, poly_size-2) = Matrix<T, Dynamic, Dynamic>::Identity(poly_size-2, poly_size-2);
  poly_vals = -1 * poly_vals / poly_vals[0];
  C.col(poly_size-2) = poly_vals.reverse().head(poly_size-1);
  EigenSolver<Matrix<T, Dynamic, Dynamic>> es(C);
  return es.eigenvalues();
};

/**
 * @brief Get the Smallest Positive Non Complex Root object
 * 
 * @tparam T 
 * @param r get the roots from the roots() function.
 * @return T Smallest Positive Non Complex Root
 */
template <class T>
T getSmallestPositiveNonComplexRoot(Eigen::Matrix<std::complex<T>, Eigen::Dynamic, Eigen::Dynamic> r) {
  T smallest_val = INFINITY;
  for (const auto& root : r.col(0)) {
    if (root.imag() == 0 && root.real() > 1e-7) smallest_val = std::min(smallest_val, root.real());
  }
  return smallest_val;
};

} // namespace long_term_planner
#endif // roots_H