/*
 * @Author: Han
 * @Date: 2018-03-02 10:56:10
 * 采用AES进行加密
 */

#include <base/aes.h>

#define AES256 1
#include <aes/aes.hpp>

namespace h2o {
std::vector<uint8_t> aes_padding(const std::vector<uint8_t> &data) {
    if (data.size() % 16 == 0) {
        return data;
    }
    // https://en.wikipedia.org/wiki/Padding_(cryptography)#PKCS7
    int padded_length = 16 - (data.size() % 16);
    int new_length = data.size() + padded_length;
    std::vector<uint8_t> new_data(new_length);
    for (int i = 0; i < data.size(); ++i) {
        new_data[i] = data[i];
    }
    for (int i = data.size(); i < new_length; ++i) {
        new_data[i] = padded_length;
    }
    return new_data;
}

std::vector<uint8_t> aes_encrypt(const std::vector<uint8_t> &data) {
    CHECK(data.size() % 16 == 0);

    AES_ctx ctx;
    AES_init_ctx_iv(&ctx, aes_key.data(), aes_iv.data());

    std::vector<uint8_t> new_data = data;
    AES_CBC_encrypt_buffer(&ctx, new_data.data(), new_data.size());
    return new_data;
}

std::vector<uint8_t> aes_decrypt(const std::vector<uint8_t> &data) {
    CHECK(data.size() % 16 == 0);

    AES_ctx ctx;
    AES_init_ctx_iv(&ctx, aes_key.data(), aes_iv.data());

    std::vector<uint8_t> new_data = data;
    AES_CBC_decrypt_buffer(&ctx, new_data.data(), new_data.size());
    return new_data;
}

bool aes_is_same(const std::vector<uint8_t> &data1, const std::vector<uint8_t> &data2) {
    if (data1.size() != data2.size()) {
        return false;
    }
    bool all_same = true;
    for (int i = 0; i < data1.size(); ++i) {
        if (data1[i] != data2[i]) {
            return false;
        }
    }

    return all_same;
}
} // namespace h2o
