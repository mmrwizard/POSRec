
/*
 * Created Date: Tuesday, September 8th 2020, 11:19:45 am
 * Author: han
 * 基于 poisson 的点云内插
 * Copyright (c) 2020 Vrlab
 */
#pragma once

#include <base/geo_image.h>

namespace h2o {
GeoImagePtr interpolate_am(const MatrixXf &points, double gsd, double w_v = 1000.0, double w_lap = 0.0,
                           double w_bilap = 1.0, int depth = -1);
GeoImagePtr interpolate_am(const MatrixXf &points, const MatrixXf &gradients, double gsd, double w_v = 1000.0, double w_g = 1.0,
                           double w_lap = 0.0, double w_bilap = 1.0, int depth = -1);
} // namespace h2o
