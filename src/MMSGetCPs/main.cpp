#include <cxxopts.hpp>

//#include <QApplication>
#include <base/base.h>
#include <base/common.h>
#include <laszip/laszip_api.h>
#include <mms/lidar_matcher.h>
#include <mms/lidar_matcher_proxy.h>

//using namespace h2o;

int main(int argc, char **argv) {
    //QApplication app(argc, argv);
    h2o::initilize_glog(argv);

    cxxopts::Options options("MMSGetCPs");

    std::string path_input;
    std::string time_interval;
    std::string path_output;

    // clang-format off
    options.add_options("MMSGetCPs")
      ("i,input", "Input laz file path", cxxopts::value(path_input))
      ("t,time_interval", "get points coordinates during the interval", cxxopts::value(time_interval))
      ("o,output_pos","rectified pos file",cxxopts::value(path_output))
      ("h,help", "Print help message.");
    // clang-format on

    auto arguments = options.parse(argc, argv);

    if (arguments.count("help") || arguments.count("i") == 0) {
        std::cout << options.help({"MMSGetCPs"}) << std::endl;
        return 1;
    }

    if (!time_interval.empty()) {
        const char *s = time_interval.data();
        double time_begin, time_end;
        std::scanf(s, "%f, %f", &time_begin, &time_end);
    }

    path_input = QFileInfo(QString::fromLocal8Bit(path_input.c_str())).absoluteFilePath().toStdString();
    path_output = QFileInfo(QString::fromLocal8Bit(path_output.c_str())).absoluteFilePath().toStdString();

    //LiDARMatches matches;
    //LiDARMatcherProxy matcher(path_input);



    //PointCloudWithData time_points;
    //time_points = get_points(path_input);

    return 0;
}






























//using PointCloudPtr = std::shared_ptr<open3d::geometry::PointCloud>;
//
//struct PointCloudWithData {
//    PointCloudPtr pcd;
//    std::vector<double> gpstime;
//};
//
//PointCloudWithData get_points(std::string points_path) {
//    laszip_POINTER lasreader;
//    laszip_create(&lasreader);
//
//    laszip_BOOL compressed;
//    laszip_open_reader(lasreader, points_path.c_str(), &compressed);
//
//    laszip_header *lasheader;
//    laszip_get_header_pointer(lasreader, &lasheader);
//    laszip_I64 num_points = (lasheader->number_of_point_records ? lasheader->number_of_point_records
//                                                                : lasheader->extended_number_of_point_records);
//
//    laszip_point *point;
//    laszip_get_point_pointer(lasreader, &point);
//
//    PointCloudPtr pcd = std::make_shared<open3d::geometry::PointCloud>();
//    pcd->points_.resize(num_points);
//    pcd->colors_.resize(num_points);
//    std::vector<double> gpstime(num_points);
//
//    for (int i = 0; i < num_points; i++) {
//        if (laszip_read_point(lasreader)) {
//            CHECK(false) << "Read point error!";
//        }
//        double x = (double)point->X * lasheader->x_scale_factor + lasheader->x_offset;
//        double y = (double)point->Y * lasheader->y_scale_factor + lasheader->y_offset;
//        double z = (double)point->Z * lasheader->z_scale_factor + lasheader->z_offset;
//
//        double intensity = point->intensity / 65535.0;
//
//        pcd->points_[i] = Vector3d(x, y, z);
//        pcd->colors_[i] = Vector3d(intensity, intensity, intensity);
//        gpstime[i] = point->gps_time;      
//    }
//
//    laszip_close_reader(lasreader);
//    laszip_destroy(lasreader);
//    
//    PointCloudWithData pcdwd;
//    pcdwd.pcd = pcd;
//    pcdwd.gpstime = gpstime;
//
//    return pcdwd;
//}
//
