/*
 * @Author: Han 
 * @Date: 2019-11-27 18:38:03 
 * 测试UTF-8得转换
 */

#define CATCH_CONFIG_MAIN

#include <base/base.h>
#include <catch.hpp>

#include <fstream>
#include <QFile>

using namespace h2o;

TEST_CASE("Test 1", "Test fstream"){
    std::string utf8 = join_paths(H2O_DATA_DIR, "文件.txt");
    QString qutf8 = QString::fromStdString(utf8);
    QFile qfile(qutf8);
    REQUIRE(qfile.open(QFile::ReadOnly));

    std::string local = string_utf8_to_fstream(utf8);

    std::ifstream ifile(local);
    REQUIRE(ifile.is_open());
};
