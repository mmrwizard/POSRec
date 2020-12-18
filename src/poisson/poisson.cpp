/*
 * @Author: Han
 * @Date: 2018-02-05 14:26:17
 * 外部引用头文件
 */
#include "poisson.h"

#include <viennacl/ocl/backend.hpp>
#include <viennacl/ocl/context.hpp>
#include <viennacl/ocl/platform.hpp>

namespace h2o {
void set_devices() {
    auto platforms = viennacl::ocl::get_platforms();
    int i = 0;
    bool is_first_element = true;
    for (auto &platform : platforms) {
        typedef std::vector<viennacl::ocl::device> devices_type;
        devices_type devices = platform.devices(CL_DEVICE_TYPE_GPU);

        std::string platform_info = platform.info();
        if (platform_info.find("NVIDIA") != std::string::npos && devices.size() > 0) {
            viennacl::ocl::setup_context(0, devices[0]);
            std::cout << "#" << std::endl;
            std::cout << "# Vendor and version: " << platform.info() << std::endl;
            std::cout << "#" << std::endl;
            return;
        }
        i++;
    }
}

static const char *my_compute_program =
    "__kernel void apply_opencl(__global const float *x, __global float *y, __global const int *A_index, int nnz) { \n"
    "    for (int i = get_global_id(0); i < nnz; i += get_global_size(0)) {                                         \n"
    "        int i0 = A_index[i * 5 + 0];                                                                           \n"
    "        int i1 = A_index[i * 5 + 1];                                                                           \n"
    "        int i2 = A_index[i * 5 + 2];                                                                           \n"
    "        int i3 = A_index[i * 5 + 3];                                                                           \n"
    "        int i4 = A_index[i * 5 + 4];                                                                           \n"
    "                                                                                                               \n"
    "        float v0 = i0 != -1 ?  4.0f : 0.0f;                                                                    \n"
    "        float v1 = i1 != -1 ? -1.0f : 0.0f;                                                                    \n"
    "        float v2 = i2 != -1 ? -1.0f : 0.0f;                                                                    \n"
    "        float v3 = i3 != -1 ? -1.0f : 0.0f;                                                                    \n"
    "        float v4 = i4 != -1 ? -1.0f : 0.0f;                                                                    \n"
    "                                                                                                               \n"
    "        // i0 可确保有值                                                                                         "
    "\n"
    "        // i0 = i0 != -1 ? i0 : i0;                                                                            \n"
    "        i1 = i1 != -1 ? i1 : i0;                                                                               \n"
    "        i2 = i2 != -1 ? i2 : i0;                                                                               \n"
    "        i3 = i3 != -1 ? i3 : i0;                                                                               \n"
    "        i4 = i4 != -1 ? i4 : i0;                                                                               \n"
    "                                                                                                               \n"
    "        y[i] = v0 * x[i0] + v1 * x[i1] + v2 * x[i2] + v3 * x[i3] + v4 * x[i4];                                 \n"
    "    }                                                                                                          \n"
    "}                                                                                                              \n";

void init_poisson_context() {
    set_devices();
    viennacl::ocl::context &ctx = viennacl::ocl::current_context();
    ctx.add_program(my_compute_program, "apply_opencl");
}

void deinit_poisson_context() {
    viennacl::ocl::context &ctx = viennacl::ocl::current_context();
    ctx.delete_program("apply_opencl");
    ctx.get_queue().finish();
}

} // namespace h2o
