/*
 * Created Date: Wednesday, October 21st 2020, 11:29:35 am
 * Author: han
 * 类似于 MutexLocker，不过是针对文件处理，且支持不同进程、IP等
 * Copyright (c) 2020 Vrlab
 */
#pragma once
#include <string>

class QLockFile;
namespace h2o {
class FileGuard {
public:
    FileGuard(const std::string &filename);
    ~FileGuard();

private:
    QLockFile *locker_;
};
} // namespace h2o
