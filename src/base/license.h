/*
 * @Author: Han
 * @Date: 2018-03-02 10:56:44
 * 完成license注册和检查
 */

#pragma once
#include <base/common.h>

namespace h2o {
std::string get_mac_address();
std::vector<uint8_t> get_mac_address_data();
std::vector<std::vector<uint8_t>> get_all_mac_addresses_data();
void save_license_data(const std::vector<uint8_t> &data, const std::string &path);
std::vector<uint8_t> load_license_data(const std::string &path);
void create_license_file(const std::string &path, int64_t days);
std::vector<uint8_t> create_license_data(const std::string &mac, int64_t days);
std::vector<uint8_t> create_license_data(const std::vector<uint8_t> &mac_data, int64_t days);
bool check_license_by_serial(const std::string &path);
void show_registration_dialog();
void show_encryption_dialog();
} // namespace h2o
