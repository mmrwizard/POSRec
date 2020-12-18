/*
 * Created Date: Wednesday, October 21st 2020, 11:29:35 am
 * Author: han
 * 类似于 MutexLocker，不过是针对文件处理，且支持不同进程、IP等
 * Copyright (c) 2020 Vrlab
 */
#include "file_guard.h"
#include "common.h"

#include <QLockFile>

namespace h2o {

FileGuard::FileGuard(const std::string &filename) {
    locker_ = new QLockFile(QString::fromStdString(filename + ".h2o.lock"));
    CHECK(locker_->lock()) << "FAILED to locker file" << filename;
}

FileGuard::~FileGuard() {
    locker_->unlock();
    delete locker_;
}

} // namespace h2o
