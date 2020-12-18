/*!
 * \file epipolar_polynomial.cpp
 *
 * \author Han
 * \date 2017/05/11
 *
 * 实例化采用多项式模型进行核线纠正
 */
#include <sat/epipolar.h>
#include <sat/oneview.h>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LambertConformalConic.hpp>

namespace h2o {
class EpipolarRectifierPolynomial : public EpipolarRectifier {
public:
    EpipolarRectifierPolynomial(const PolynomialCamera &fwd_camera, const PolynomialCamera &bwd_camera)
        : fwd_camera_(fwd_camera), bwd_camera_(bwd_camera) {
        P_epipolar_to_lambert_ = Matrix2d::Identity();
        P_lambert_to_epipolar_ = Matrix2d::Identity();
    }

public:
    virtual void set_projection(std::shared_ptr<GeographicLib::LambertConformalConic> converter, double lon0) {
        converter_ = converter;
        lon0_ = lon0;

        if (!geocentric_) {
            double a = converter_->MajorRadius();
            double f = converter_->Flattening();
            geocentric_ = std::make_shared<GeographicLib::Geocentric>(a, f);
        }
    }

protected:
    virtual Vector2i get_camera_size(bool is_forward = true) const override;
    virtual Vector2d project_latlon(const Vector3d &latlon, bool is_forward) const override;
    virtual Vector2d backproject_latlon(const Vector2d &pixel, double z, bool is_forward) const override;

private:
    PolynomialCamera fwd_camera_;
    PolynomialCamera bwd_camera_;
    std::shared_ptr<GeographicLib::Geocentric> geocentric_;
};
Vector2i EpipolarRectifierPolynomial::get_camera_size(bool is_forward) const {
    if (is_forward) {
        return {fwd_camera_.width, fwd_camera_.height};
    } else {
        return {bwd_camera_.width, bwd_camera_.height};
    }
}
Vector2d EpipolarRectifierPolynomial::project_latlon(const Vector3d &latlon, bool is_forward) const {
    Vector3d geoc;
    geocentric_->Forward(latlon(0), latlon(1), latlon(2), geoc(0), geoc(1), geoc(2));

    if (is_forward) {
        return project(geoc, fwd_camera_);
    } else {
        return project(geoc, bwd_camera_);
    }
}
Vector2d EpipolarRectifierPolynomial::backproject_latlon(const Vector2d &pixel, double z, bool is_forward) const {
    double a = converter_->MajorRadius();
    double f = converter_->Flattening();
    if (is_forward) {
        return project_to_geographic_z(fwd_camera_, z, pixel, a, f).head(2);
    } else {
        return project_to_geographic_z(bwd_camera_, z, pixel, a, f).head(2);
    }
}

std::shared_ptr<EpipolarRectifier> create_epipolar_rectifier_poly_camera(const PolynomialCamera &fwd_camera,
                                                                         const PolynomialCamera &bwd_camera) {
    std::shared_ptr<EpipolarRectifier> rectifier =
        std::make_shared<EpipolarRectifierPolynomial>(fwd_camera, bwd_camera);
    return rectifier;
}

} // namespace h2o
