//------------------- hpp Part

#pragma once
#include <cmath>
#include <vector>
#include "Eigen/Eigen"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "Eigen/StdVector"

/*********************************************************************
 * Point2d
 *********************************************************************/
using Point2d = Eigen::Vector2d;

using Points2d = std::vector<Point2d, Eigen::aligned_allocator<Point2d>>;

/*********************************************************************
 * Vector2d
 *********************************************************************/
/// \brief 2D Vector
using Vector2d = Eigen::Vector2d;

/// \brief Vector of 2D Vectors
using Vectors2d = std::vector<Vector2d, Eigen::aligned_allocator<Vector2d>>;

/// \brief Inline function for the square of a number.
/// \param x Number to square
/// \return Square of a number
template<class T>
inline T square(const T x) { return x*x; }

/// \brief Modifies the given angle to translate it into the [0,2pi[ range.
/// \note Take care of not instancing this template for integer numbers, since it only works for float, double and long double.
/// \param a Angle to wrap
template <class T>
inline void wrapTo2PiInPlace(T &a)
{
  bool was_neg = a<0;
  a = fmod(a, static_cast<T>(2.0*M_PI) );
  if (was_neg) a+=static_cast<T>(2.0*M_PI);
}

/// \brief Modifies the given angle to translate it into the [0,2pi[ range.
/// \note Take care of not instancing this template for integer numbers, since it only works for float, double and long double.
/// \return Wraped angle
template <class T>
inline T wrapTo2Pi(T a)
{
  wrapTo2PiInPlace(a);
  return a;
}

/// \brief Modifies the given angle to translate it into the ]-pi,pi] range.
/// \note Take care of not instancing this template for integer numbers, since it only works for float, double and long double.
/// \return Wrapped angle
template <class T>
inline T wrapToPi(T a)
{
  return wrapTo2Pi( a + static_cast<T>(M_PI) )-static_cast<T>(M_PI);
}

/// \brief Modifies the given angle to translate it into the ]-pi,pi] range.
/// \note Take care of not instancing this template for integer numbers, since it only works for float, double and long double.
/// \param a Angle to wrap
template <class T>
inline void wrapToPiInPlace(T &a)
{
  a = wrapToPi(a);
}

