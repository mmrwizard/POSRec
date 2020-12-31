/*
 * Created Date: Thursday, December 31st 2020, 5:44:24 pm
 * Author: Han
 * 测试全景相机几何信息
 * Copyright (c) 2020 Vrlab
 */
#define CATCH_CONFIG_MAIN
#include <catch.hpp>

#include <base/common.h>
#include <mms/ladybug_stream.h>
#include <mms/pos.h>

using namespace h2o;

LadybugImage generate_image() {
    Vector3d ypr(90.0 * DEG2RAD, 0.0, 0.0);
    Matrix3d R = Pos::ypr_to_rotation(ypr(0), ypr(1), ypr(2));

    LadybugImage image;
    image.width = 4096;
    image.height = 2048;
    image.type = 0;
    image.q_body = R;

    return image;
}

TEST_CASE("Test1", "Test local geometry of panorama images") {
    constexpr double m_pi = 3.14159265358979323846;
    LadybugImage image = generate_image();

    Vector3d world(0.0, 1.0, 0.0);
    Vector3d local = image.to_local(world);

    REQUIRE((local - Vector3d(-1, 0, 0)).norm() == Approx(0.0).margin(1e-6));

    Vector2d angle = image.to_angle(local);
    REQUIRE(sin(angle.x()) == Approx(0.0).margin(1e-6));
    REQUIRE(sin(angle.y()) == Approx(0.0).margin(1e-6));

    Vector2d pixel = image.to_pixel(angle);
    REQUIRE(pixel.y() == Approx(1023.5).margin(1e-6));
}
