/*
 * Filename: /home/han/Documents/LinSAR/tests/test_proj.cpp
 * Path: /home/han/Documents/LinSAR/tests
 * Created Date: Monday, January 13th 2020, 10:07:10 pm
 * Author: han
 * 测试 proj 库
 * Copyright (c) 2020 Vrlab
 */

#define CATCH_CONFIG_MAIN
#include <catch.hpp>

#include <base/base.h>
#include <proj.h>

PJ_CONTEXT *ctx = nullptr;

void init_proj();

using namespace h2o;

TEST_CASE("test datum", "test read projection") {
    init_proj();

    std::string path = join_paths(H2O_DATA_DIR, "TDX-1_20121205T101849_SRTM.tif");
    std::string wkt = gdal_get_projection_wkt(path);

    PJ *pj = proj_create(ctx, wkt.c_str());
    std::string proj = proj_as_proj_string(ctx, pj, PJ_PROJ_4, nullptr);
    LOG(INFO) << proj;

    proj_destroy(pj);
    proj_context_destroy(ctx);
}

TEST_CASE("test EGM96", "test convert egm96") {
    init_proj();

    std::string path = join_paths(H2O_DATA_DIR, "TDX-1_20121205T101849_SRTM.tif");
    std::string wkt = gdal_get_projection_wkt(path);

    PJ *pj = proj_create(ctx, wkt.c_str());
    std::string proj = proj_as_proj_string(ctx, pj, PJ_PROJ_4, nullptr);
    proj_destroy(pj);

    proj += " +geoidgrids=egm96_15.gtx";
    pj = proj_create(ctx, proj.c_str());
    wkt = proj_as_wkt(ctx, pj, PJ_WKT1_GDAL, nullptr);
    LOG(INFO) << wkt;

    proj_destroy(pj);
    proj_context_destroy(ctx);
}

TEST_CASE("test convert", "test create convert object") {
    init_proj();

    std::string path = join_paths(H2O_DATA_DIR, "TDX-1_20121205T101849_SRTM.tif");
    std::string wkt_wgs84 = gdal_get_projection_wkt(path);

    PJ *pj = proj_create(ctx, wkt_wgs84.c_str());
    std::string proj = proj_as_proj_string(ctx, pj, PJ_PROJ_5, nullptr);
    proj_destroy(pj);

    proj += " +vunits=m";
    pj = proj_create(ctx, proj.c_str());
    wkt_wgs84 = proj_as_wkt(ctx, pj, PJ_WKT2_2018, nullptr);
    LOG(INFO) << proj_as_proj_string(ctx, pj, PJ_PROJ_5, nullptr);
    proj_destroy(pj);

    proj += " +geoidgrids=egm96_15.gtx";
    pj = proj_create(ctx, proj.c_str());
    std::string wkt_egm96 = proj_as_wkt(ctx, pj, PJ_WKT2_2018, nullptr);
    LOG(INFO) << proj_as_proj_string(ctx, pj, PJ_PROJ_5, nullptr);
    proj_destroy(pj);

    pj = proj_create_crs_to_crs(ctx, wkt_wgs84.c_str(), wkt_egm96.c_str(), nullptr);
    LOG(INFO) << proj_as_proj_string(ctx, pj, PJ_PROJ_5, nullptr);

    proj_destroy(pj);
    proj_context_destroy(ctx);
}

TEST_CASE("test convert image", "test convert srtm image") {
    init_proj();

    std::string path = join_paths(H2O_DATA_DIR, "TDX-1_20121205T101849_SRTM.tif");
    GeoImage image;
    image.read_image_unchanged(path);
    std::string wkt_wgs84 = image.get_wkt();

    PJ *pj = proj_create(ctx, wkt_wgs84.c_str());
    std::string proj_wgs84 = proj_as_proj_string(ctx, pj, PJ_PROJ_5, nullptr);
    proj_destroy(pj);

    // 必须要加这一行，否则 PROJ 不知道是三维坐标
    proj_wgs84 += " +vunits=m";
    pj = proj_create(ctx, proj_wgs84.c_str());
    proj_destroy(pj);

    std::string proj_egm96 = proj_wgs84 + " +geoidgrids=egm96_15.gtx";
    pj = proj_create(ctx, proj_egm96.c_str());
    proj_egm96 = proj_as_proj_string(ctx, pj, PJ_PROJ_5, nullptr);
    proj_destroy(pj);

    pj = proj_create_crs_to_crs(ctx, proj_egm96.c_str(), proj_wgs84.c_str(), nullptr);
    LOG(INFO) << proj_as_proj_string(ctx, pj, PJ_PROJ_5, nullptr);

    int rows = image.get_size().y();
    int cols = image.get_size().x();

    float height = image.get_image().at<float>(0, 0);
    Vector2d position = image.pixel_to_projection(Vector2d(0, 0));
    PJ_COORD coord = proj_coord(position.x(), position.y(), height, 0);

    LOG(INFO) << coord.lpz.lam;
    LOG(INFO) << coord.lpz.phi;
    LOG(INFO) << coord.lpz.z;

    PJ_COORD trans = proj_trans(pj, PJ_FWD, coord);
    LOG(INFO) << trans.lpz.lam;
    LOG(INFO) << trans.lpz.phi;
    LOG(INFO) << trans.lpz.z;

    trans = proj_trans(pj, PJ_INV, trans);
    LOG(INFO) << trans.lpz.lam;
    LOG(INFO) << trans.lpz.phi;
    LOG(INFO) << trans.lpz.z;

    proj_destroy(pj);
    proj_context_destroy(ctx);
}

TEST_CASE("test tutorial", "test the proj quick start") {
    init_proj();

    PJ_CONTEXT *C = ctx;
    PJ *P;
    PJ *P_for_GIS;
    PJ_COORD a, b;

    P = proj_create_crs_to_crs(C, "EPSG:4326", "+proj=utm +zone=32 +datum=WGS84", /* or EPSG:32632 */
                               NULL);

    /* This will ensure that the order of coordinates for the input CRS */
    /* will be longitude, latitude, whereas EPSG:4326 mandates latitude, */
    /* longitude */
    P_for_GIS = proj_normalize_for_visualization(C, P);
    proj_destroy(P);
    P = P_for_GIS;

    /* a coordinate union representing Copenhagen: 55d N, 12d E
     Given that we have used proj_normalize_for_visualization(), the order of
     coordinates is longitude, latitude, and values are expressed in degrees. */
    a = proj_coord(122, 55, 0, 0);

    /* transform to UTM zone 32, then back to geographical */
    b = proj_trans(P, PJ_FWD, a);
    printf("easting: %.3f, northing: %.3f\n", b.xy.x, b.xy.y);
    b = proj_trans(P, PJ_INV, b);
    printf("longitude: %g, latitude: %g\n", b.xy.x, b.xy.y);

    /* Clean up */
    proj_destroy(P);
    proj_context_destroy(C); /* may be omitted in the single threaded case */
}

TEST_CASE("test spatial", "test SpatialEgm96") {
    std::string dir = h2o::join_paths(H2O_DATA_DIR, "resources/proj/");
    set_proj_directory(dir);

    std::string path = join_paths(H2O_DATA_DIR, "TDX-1_20121205T101849_SRTM.tif");
    GeoImage image;
    image.read_image_unchanged(path);
    std::string wkt_wgs84 = image.get_wkt();
    SpatialEgm96 spatial(wkt_wgs84);

    {
        Vector3d point;
        int rows = image.get_size().y();
        int cols = image.get_size().x();
        point.z() = image.get_image().at<float>(0, 0);
        point.head(2) = image.pixel_to_projection(Vector2d(0, 0));

        Vector3d point2 = spatial.forward(point);
        Vector3d point3 = spatial.backward(point2);

        REQUIRE(std::abs(point.z() - point3.z()) < 1e-3);
    }
    {
        std::vector<Vector3d> points(10);
        for (int i = 0; i < 10; ++i) {
            points[i].z() = image.get_image().at<float>(0, 0);
            points[i].head(2) = image.pixel_to_projection(Vector2d(0, 0));
        }

        auto points2 = spatial.forward(points);
        auto points3 = spatial.backward(points2);

        REQUIRE(std::abs(points[0].z() - points3[0].z()) < 1e-3);
    }
}
void init_proj() {
    std::string dir = h2o::join_paths(H2O_DATA_DIR, "resources/proj/");

    const char *paths[] = {dir.c_str()};

    ctx = proj_context_create();
    proj_context_set_search_paths(ctx, 1, paths);
}
