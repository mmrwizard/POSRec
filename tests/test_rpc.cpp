/*
 * Filename: /home/han/Documents/LinSAR/tests/test_rpc.cpp
 * Path: /home/han/Documents/LinSAR/tests
 * Created Date: Friday, March 6th 2020, 11:12:54 am
 * Author: han
 * 测试 RPC 文件读取
 * Copyright (c) 2020 Vrlab
 */

#define CATCH_CONFIG_MAIN
#include <catch.hpp>

#include "sat/camera_rpc.h"
#include <base/base.h>

using namespace h2o;

TEST_CASE("LOAD", "Test load RPC file") {
    std::string file =
        join_paths(H2O_DATA_DIR, "bad-zy302a_bwd_016942_244080_20190618174540_01_sec_0001_1906216003_rpc.txt");
    RpcCoefficient rpc = load_rpc(file);

    REQUIRE(std::abs(rpc.line_off - 9997.5) < 0.1);
    REQUIRE(std::abs(rpc.line_den_coeff(0) - 1.0) < 0.001);
}
