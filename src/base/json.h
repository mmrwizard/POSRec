/*
 * @Author: Han Hu
 * @Date: 2019-01-22 14:06:00
 * 读取json文件
 */
#pragma once

#include <nlohmann/json.hpp>
#include <xml2json.hpp>

#include <QFile>
#include <QTextStream>
#include <sstream>
#include <string>

namespace h2o {
inline nlohmann::json read_json_file(const std::string &filename) {
    QFile file(QString::fromStdString(filename));
    file.open(QFile::ReadOnly);
    std::string text = file.readAll().toStdString();

    if (string_to_lower(get_extension_name(filename)) == ".xml") {
        text = xml2json(text.c_str());
    }

    std::istringstream ss(text);

    nlohmann::json j;
    ss >> j;
    return j;
}

inline void save_json_file(const nlohmann::json &j, const std::string &filename) {
    std::string data = j.dump(4);
    QFile file(QString::fromStdString(filename));
    file.open(QFile::WriteOnly);
    QTextStream ts(&file);
    ts.setCodec("UTF-8");
    ts << QString::fromStdString(data);
    file.close();
}

} // namespace h2o
