/**
 * This file is part of https://github.com/adrelino/interpolation-methods
 *
 * Copyright (c) 2018 Adrian Haarbach <mail@adrian-haarbach.de>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */
#ifndef INTERPOL_BEZIER_HPP
#define INTERPOL_BEZIER_HPP

#include "spline.h"

#include <Eigen/Core>

namespace interpol {

namespace bezier {

template <unsigned int Dim> class Spline {
public:
    Spline() = default;
    Spline(std::vector<Eigen::Matrix<double, Dim, 1>> &pts, std::vector<double> &t_vec, bool firstDerivBoundary = false,
           Eigen::Matrix<double, Dim, 1> leftBoundary = Eigen::Matrix<double, Dim, 1>::Zero(),
           Eigen::Matrix<double, Dim, 1> rightBoundary = Eigen::Matrix<double, Dim, 1>::Zero()) {
        n = pts.size();
        assert(n >= 2);
        assert(n == t_vec.size());

        std::vector<double> x_vecs[Dim];

        for (size_t i = 0; i < n; i++) {
            /*            double tGlobal = i*1.0/(n-1);
                        t_vec.push_back(tGlobal);*/

            const Eigen::Matrix<double, Dim, 1> &pos = pts[i];
            for (unsigned int j = 0; j < Dim; j++) {
                x_vecs[j].push_back(pos(j));
            }
        }

        for (unsigned int j = 0; j < Dim; j++) {
            if (firstDerivBoundary) {
                splines[j].set_boundary(tk::spline::bd_type::first_deriv, leftBoundary(j),
                                        tk::spline::bd_type::first_deriv, rightBoundary(j));
            } else {
                splines[j].set_boundary(tk::spline::bd_type::second_deriv, leftBoundary(j),
                                        tk::spline::bd_type::second_deriv, rightBoundary(j));
            }
            splines[j].set_points(t_vec, x_vecs[j]);
        }
    }

    Eigen::Matrix<double, Dim, 1> eval(double tGlobal) const {
        Eigen::Matrix<double, Dim, 1> p;
        for (unsigned int j = 0; j < Dim; j++) {
            p(j) = splines[j](tGlobal);
        }
        return p;
    }

    std::vector<Eigen::Matrix<double, Dim, 1>> getBasisPoints() {
        std::vector<Eigen::Matrix<double, Dim, 1>> pts;
        for (unsigned int i = 0; i < n - 1; i++) {

            for (int k = 0; k <= 3; k++) {
                Eigen::Matrix<double, Dim, 1> p;
                for (unsigned int j = 0; j < Dim; j++) {
                    p(j) = splines[j].control_polygon(i, k);
                }
                pts.push_back(p);
            }
        }

        return pts;
    }

private:
    // interpolated positions
    tk::spline splines[Dim];
    size_t n;
};

typedef Spline<3> Spline3d;
typedef Spline<4> Spline4d;

} // namespace bezier

} // namespace interpol

#endif // INTERPOL_BEZIER_HPP
