/*
 * @Author: Han
 * @Date: 2018-03-02 10:57:11
 * 完成license注册和检查
 */
#include <base/aes.h>
#include <base/filesystem.h>
#include <base/license.h>

#include <QFile>
#include <QNetworkInterface>
#include <QSettings>
#include <chrono>
#include <ctime>

namespace h2o {

std::string get_mac_address() {
    std::vector<QString> all_addresses;
    for (const auto &net_interface : QNetworkInterface::allInterfaces()) {
        all_addresses.push_back(net_interface.hardwareAddress());
    }

    std::sort(begin(all_addresses), end(all_addresses));

    if (all_addresses.size() > 0) {
        return all_addresses.back().toStdString();
    }
    return std::string();
}

std::vector<uint8_t> get_mac_address_data() {
    std::string mac = get_mac_address();
    std::vector<uint8_t> data(mac.begin(), mac.end());
    data = aes_padding(data);
    return data;
}

std::vector<std::vector<uint8_t>> get_all_mac_addresses_data() {
    std::vector<QString> all_addresses;
    for (const auto &net_interface : QNetworkInterface::allInterfaces()) {
        all_addresses.push_back(net_interface.hardwareAddress());
    }

    std::vector<std::vector<uint8_t>> all_data;
    for (const auto &address : all_addresses) {
        std::string mac = address.toStdString();
        std::vector<uint8_t> data(mac.begin(), mac.end());
        data = aes_padding(data);
        all_data.push_back(data);
    }
    return all_data;
}

void save_license_data(const std::vector<uint8_t> &data, const std::string &path) {
    QFile ofile(QString::fromStdString(path));
    ofile.open(QFile::WriteOnly);
    int length = data.size();
    ofile.write((char *)&length, 4);
    ofile.write((char *)data.data(), length);
    ofile.close();
}

std::vector<uint8_t> load_license_data(const std::string &path) {
    QFile ifile(QString::fromStdString(path));
    ifile.open(QFile::ReadOnly);
    int length;
    ifile.read((char *)&length, 4);
    std::vector<uint8_t> data(length);
    ifile.read((char *)data.data(), length);
    return data;
}

void create_license_file(const std::string &path, int64_t days) {
    std::string mac = get_mac_address();
    auto data = create_license_data(mac, days);
    save_license_data(data, path);
}

std::vector<uint8_t> create_license_data(const std::string &mac, int64_t days) {
    std::vector<uint8_t> data(mac.begin(), mac.end());
    data = aes_padding(data);
    return create_license_data(data, days);
}

std::vector<uint8_t> create_license_data(const std::vector<uint8_t> &mac_data, int64_t days) {
    CHECK(mac_data.size() % 16 == 0);
    std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::vector<uint8_t> time_buffer(16);
    memcpy(time_buffer.data(), &now, 8);
    memcpy(time_buffer.data() + 8, &days, 8);

    std::vector<uint8_t> all_buffer = mac_data;
    all_buffer.insert(end(all_buffer), begin(time_buffer), end(time_buffer));

    std::vector<uint8_t> encrypted = aes_encrypt(all_buffer);
    return encrypted;
}

bool check_license_by_serial(const std::string &path) {
    if (!file_exist(path)) {
        return false;
    }

    std::vector<uint8_t> data = load_license_data(path);
    data = aes_decrypt(data);
    auto macs = get_all_mac_addresses_data();

    std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::time_t license_time;
    int64_t days;
    memcpy(&license_time, data.data() + data.size() - 16, 8);
    memcpy(&days, data.data() + data.size() - 8, 8);

    bool time_valid = false;
    double period = std::difftime(now, license_time) / (60 * 60 * 24);

    if (period > days) {
        return false;
    }
    if (period < -10) {
        return false;
    }

    data.resize(data.size() - 16);

    bool same = std::any_of(begin(macs), end(macs), [&](const auto &mac) { return aes_is_same(data, mac); });
    return same;
}

void show_registration_dialog() {}

void show_encryption_dialog() {}
} // namespace h2o
