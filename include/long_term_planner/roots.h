#include <iostream>
#include <iomanip>
#include <limits>
#include <tuple>

#include <boost/math/tools/roots.hpp>
#include <boost/math/special_functions/next.hpp>
#include <boost/math/special_functions/cbrt.hpp>
#include <boost/math/special_functions/pow.hpp>

#ifndef roots_H
#define roots_H

namespace long_term_planner {

template <class T>

/**
 * @brief Functor for a 4th degree polynomial.
 * a_4 * x^4 + a_3 * x^3 + a_2 * x^2 + a_1 * x + a_0 = 0
 */
struct fourth_functor_2deriv {
 private:
  T a_4;
  T a_3;
  T a_2;
  T a_1;
  T a_0;
 public:
  // Functor returning both 1st and 2nd derivatives.
  fourth_functor_2deriv(T const& a_4, T const& a_3, T const& a_2, T const& a_1, T const& a_0) : 
    a_4(a_4),
    a_3(a_3),
    a_2(a_2),
    a_1(a_1),
    a_0(a_0) {}

  std::tuple<T, T, T> operator()(T const& x) {
    // Return f(x), f'(x) and f''(x).
    T fx = a_4 * boost::math::pow<4>(x) + 
           a_3 * boost::math::pow<3>(x) + 
           a_2 * boost::math::pow<2>(x) + 
           a_1 * x + a_0;
    T dx = 4.0 * a_4 * boost::math::pow<3>(x) +
           3.0 * a_3 * boost::math::pow<2>(x) +
           2.0 * a_2 * x + a_1;
    T d2x = 12.0 * a_4 * boost::math::pow<2>(x) +
            6.0 * a_3 * x +
            2.0 * a_2;
    return std::make_tuple(fx, dx, d2x);
  }
};

template <class T>
T fourth_2deriv(T a_4, T a_3, T a_2, T a_1, T a_0) {
  // return fourth root of x using 1st and 2nd derivatives and Halley.
  using namespace std;                  // Help ADL of std functions.
  using namespace boost::math::tools;   // for halley_iterate.

  int exponent;
  frexp(a_0, &exponent);                 // Get exponent of z (ignore mantissa).
  T guess = ldexp(1., exponent / 4);     // Rough guess is to divide the exponent by four.
  T min = 0.0;                           // Minimum possible value is zero.
  T max = ldexp(10., exponent / 4);      // Maximum possible value is ten times our guess.
  // Stop when slightly more than one of the digits are correct:
  const int digits = 8;
  const boost::uintmax_t maxit = 50;
  boost::uintmax_t it = maxit;
  T result = halley_iterate(fourth_functor_2deriv<T>(a_4, a_3, a_2, a_1, a_0), guess, min, max, digits, it);
  return result;
}

} // namespace long_term_planner
#endif // roots_H