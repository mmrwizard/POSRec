/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */
#ifndef INTERPOL_SU2_HPP
#define INTERPOL_SU2_HPP

#include <Eigen/Geometry>
#include <iomanip>

#include "QuaternionMapping.h"

/**
 * Sinus cardinalis. Not normed version
 * @param t
 * @return sin(t)/t
 */
template <typename T> T sinc(T theta) {
    return std::sin(theta) / theta; // si
    // return sin(M_PI*theta) / (M_PI*theta); normed //sinc
} // needed in QuaternionMapping.h

// using namespace px; //to import logq, expq into global namespace!

// Added to namespace Eigen so that ADL works (otherwise won't compile on CLANG:
// https://clang.llvm.org/compatibility.html#dep_lookup)
namespace Eigen {
/**
 * Exp map applied to quaternions.
 *
 * The usual exp : so(3) -> SO(3) maps
 * from the Lie algebra (a tangent space) so(3) of skew-symmetric matrices of the from [wx,wy,wz]_x
 * to   the Lie group (a smooth manifold) SO(3) of orthonormal rotation matrices  : R e 3x3 : RR^T=I, |R|=1
 *
 * https://github.com/hengli/vmav-ros-pkg/blob/master/calibration/hand_eye_calibration/include/hand_eye_calibration/QuaternionMapping.h
 *
 * @param q' so(3) tangent space quaternion (if it was unit quaternion before, then its w is 0)
 * @return q = exp(q') SO(3) group quaternion
 */
template <typename T> inline Eigen::Quaternion<T> expq(const Eigen::Quaternion<T> &q) { return px::expq(q); }

/**
 * Log map applied to quaternions.
 *
 * If q was a unit quaternion, then w will be 0 in q' (since log(1)=0)
 *
 * The usual log : SO(3) -> so(3) maps
 * from the Lie group (a smooth manifold) SO(3) of orthonormal rotation matrices  : R e 3x3 : RR^T=I, |R|=1
 * to   the Lie algebra (a tangent space) so(3) of skew-symmetric matrices of the from [wx,wy,wz]_x
 *
 * https://github.com/hengli/vmav-ros-pkg/blob/master/calibration/hand_eye_calibration/include/hand_eye_calibration/QuaternionMapping.h
 *
 * @param q SO(3) group quaternion
 * @return q' = log(q) so(3) tangent space quaternion
 */
template <typename T> inline Eigen::Quaternion<T> logq(Eigen::Quaternion<T> q) { return px::logq(q); }

// printing of Quaternion, Vector3 and Map<Vector3> works with this (even for Ceres Jet types)
template <typename T> static std::ostream &operator<<(std::ostream &stream, const Eigen::QuaternionBase<T> &q) {
    return stream << std::setprecision(2) << std::fixed << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z();
}
template <typename T> static std::ostream &operator<<(std::ostream &stream, const Eigen::Matrix<T, 3, 1> &q) {
    return stream << std::setprecision(2) << std::fixed << q.x() << ", " << q.y() << ", " << q.z();
}
template <typename T>
static std::ostream &operator<<(std::ostream &stream, const Eigen::Map<Eigen::Matrix<T, 3, 1>> &q) {
    return stream << std::setprecision(2) << std::fixed << q.x() << ", " << q.y() << ", " << q.z();
}

/**
 * Addition of quaternion coefficients.
 * \warning This operation has no direct geometric interpretation, and in the general case result in a non unit
 * quaternion.
 * @return the addition of \c *this and \a other
 */
template <typename T>
static Eigen::Quaternion<T> operator+(const Eigen::Quaternion<T> &a, const Eigen::Quaternion<T> &b) {
    return Eigen::Quaternion<T>(a.coeffs() + b.coeffs());
}
/**
 * Substraction of quaternion coefficients.
 * \warning This operation has no direct geometric interpretation, and in the general case result in a non unit
 * quaternion.
 * @return the substraction of \c *this and \a other
 */
template <typename T>
static Eigen::Quaternion<T> operator-(const Eigen::Quaternion<T> &a, const Eigen::Quaternion<T> &b) {
    return Eigen::Quaternion<T>(a.coeffs() - b.coeffs());
}
/**
 * Lhs multiplication of quaternion coefficients with a scalar.
 * Only useful in log space of unit quaternion (when w==0)
 */
template <typename T> static Eigen::Quaternion<T> operator*(const T &b, const Eigen::Quaternion<T> &a) {
    return Eigen::Quaternion<T>(a.coeffs() * b);
}
/**
 * Rhs multiplication of quaternion coefficients with a scalar.
 * Only useful in log space of unit quaternion (when w==0)
 */
template <typename T> static Eigen::Quaternion<T> operator*(const Eigen::Quaternion<T> &a, const T &b) {
    return Eigen::Quaternion<T>(a.coeffs() * b);
};

} // namespace Eigen

#endif // INTERPOL_SU2_HPP
