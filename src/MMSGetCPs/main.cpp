#include <cxxopts.hpp>

#include <QApplication>
#include <QSurfaceFormat>

#include <base/base.h>
#include <mms/ladybug_stream.h>
#include <mms/lidar_render.h>
#include <mms/hsr_file.h>
#include <iomanip>
#include <iostream>
#include <fstream>
//#include "readlas.h"

#include <mms/pos.h>

using namespace h2o;


int main(int argc, char **argv) {
        QApplication app(argc, argv);
        initilize_glog(argv);
    
        cxxopts::Options options("MMSGetCPs");
    
        std::string path_hsr;
        std::string path_truth_las;
        std::string path_warp_las;
        std::string path_warp_pos;
        std::string time_range;
        std::string path_controls;
        std::string path_checks;
        std::string time_intervals;
        std::string path_config;
    
        // clang-format off
        options.add_options("MMSGetCPs")
          ("hsr", "Input raw hsr file path", cxxopts::value(path_hsr))
          ("t,truth_las_input", "Input truth laz file path", cxxopts::value(path_truth_las))
          ("w,warp_las_input", "Input warped laz file path", cxxopts::value(path_warp_las))
          ("p,warp_pos_input", "Input warped pos file path", cxxopts::value(path_warp_pos))
          ("r,time_range", "get time range points coordinates during the interval", cxxopts::value(time_range))
          ("c,output_controls","output control pairs file",cxxopts::value(path_controls))            
          ("o,output_checks","check points file",cxxopts::value(path_checks))            
          ("d,interval","interval time",cxxopts::value(time_intervals))
          ("j,config","config json file",cxxopts::value(path_config))
          ("h,help", "Print help message.");
        // clang-format on
    
        auto arguments = options.parse(argc, argv);
    
        if (arguments.count("help") || arguments.count("t") == 0) {
            std::cout << options.help({"MMSGetCPs"}) << std::endl;
            return 1;
        }
    
        Eigen::Vector2d time_duration;
        double time_begin, time_end;
        if (!time_range.empty()) {
            const char *s = time_range.data();
            std::sscanf(s, "%lf,%lf", &time_begin, &time_end);
            time_duration << time_begin, time_end;
        }
        double time_interval;
        if (!time_range.empty()) {
            const char *s = time_intervals.data();
            std::sscanf(s, "%lf", &time_interval);
        }
        const double t = time_interval;
    
        path_truth_las = QFileInfo(QString::fromLocal8Bit(path_truth_las.c_str())).absoluteFilePath().toStdString();
        path_checks = QFileInfo(QString::fromLocal8Bit(path_checks.c_str())).absoluteFilePath().toStdString();

        //1、提取真值点云中的控制点及检查点
        std::cout << "extract truth laz controls and checks..." << std::endl;      
        LiDARRender render;
        render.load_las(path_truth_las);        
        std::vector<std::vector<double>> controls_gcps;
        std::vector<std::vector<double>> truth_checks;
        {
            int i = 0, cp_i = 15000, ck_i = 150000;
            double control_t, check_t;
            int num_points = render.gps_times_.size();
            double time_range0 = time_duration.x(), time_range1 = time_duration.y();
            double time_check = time_range0 + 1.0;
            while (time_range0 < time_range1 && time_check < time_range1 && i < num_points) {
                control_t =
                    std::fabs(time_range0 - render.gps_times_[i]) - std::fabs(time_range0 - render.gps_times_[i + 1]);
                check_t = std::fabs(time_check - render.gps_times_[i]) -
                          std::fabs(time_check - render.gps_times_[i + 1]);
                if (control_t < 0.0 && (i + 6 * cp_i) < num_points) {
                    for (int j = 0; j < 6; j++) {
                        std::vector<double> control_temp(5);
                        if (std::fabs(render.gps_times_[i + j * cp_i] - render.gps_times_[i]) < 1.0) {
                            //激光点频实际为20万点/秒，每组6个控制点保证在2秒范围内
                            control_temp[0] = render.gps_times_[i + j * cp_i]; 
                            control_temp[1] = render.points_[i + j * cp_i].x();
                            control_temp[2] = render.points_[i + j * cp_i].y();
                            control_temp[3] = render.points_[i + j * cp_i].z();
                            control_temp[4] = render.intensitys_[i + j * cp_i];
                            controls_gcps.push_back(control_temp);
                        }
                    }
                    i++, time_range0 += t;
                    continue;
                } else if (check_t < 0.0) {
                    for (int j = 0; j < 6; j++) {
                        std::vector<double> check_temp(5);
                        if (std::fabs(render.gps_times_[i + j * ck_i] - render.gps_times_[i]) < 5.0) {
                            //每组检查点设置在5秒范围内
                            check_temp[0] = render.gps_times_[i + j * ck_i];
                            check_temp[1] = render.points_[i + j * ck_i].x();
                            check_temp[2] = render.points_[i + j * ck_i].y();
                            check_temp[3] = render.points_[i + j * ck_i].z();
                            check_temp[4] = render.intensitys_[i + j * ck_i];
                            truth_checks.push_back(check_temp);
                        }
                    }
                    i++, time_check += 10.0;
                    continue;
                }
                else {
                    i++;
                }
            }
        }

        //2、提取失锁点云中对应的控制点及检查点
        std::cout << "extract warp laz controls and checks..." << std::endl;
        render.load_las(path_warp_las);
        std::vector<std::vector<double>> controls_pcds;
        std::vector<std::vector<double>> warp_checks;
        {
            int i = 0, cp_i = 15000, ck_i = 150000;
            double control_t, check_t;
            int num_points = render.gps_times_.size();
            double time_range0 = time_duration.x(), time_range1 = time_duration.y();
            double time_check = time_range0 + 1.0;
            while (time_range0 < time_range1 && time_check < time_range1 && i < num_points) {
                control_t =
                    std::fabs(time_range0 - render.gps_times_[i]) - std::fabs(time_range0 - render.gps_times_[i + 1]);
                check_t =
                    std::fabs(time_check - render.gps_times_[i]) - std::fabs(time_check - render.gps_times_[i + 1]);
                if (control_t < 0.0 && (i + 6 * cp_i) < num_points) {
                    for (int j = 0; j < 6; j++) {
                        std::vector<double> control_temp(5);
                        if (std::fabs(render.gps_times_[i + j * cp_i] - render.gps_times_[i]) < 1.0) {
                            //激光点频实际为20万点/秒，每组6个控制点保证在2秒范围内
                            control_temp[0] = render.gps_times_[i + j * cp_i];
                            control_temp[1] = render.points_[i + j * cp_i].x();
                            control_temp[2] = render.points_[i + j * cp_i].y();
                            control_temp[3] = render.points_[i + j * cp_i].z();
                            control_temp[4] = render.intensitys_[i + j * cp_i];
                            controls_pcds.push_back(control_temp);
                        }
                    }
                    i++, time_range0 += t;
                    continue;
                } else if (check_t < 0.0) {
                    for (int j = 0; j < 6; j++) {
                        std::vector<double> check_temp(5);
                        if (std::fabs(render.gps_times_[i + j * ck_i] - render.gps_times_[i]) < 5.0) {
                            //每组检查点设置在5秒范围内
                            check_temp[0] = render.gps_times_[i + j * ck_i];
                            check_temp[1] = render.points_[i + j * ck_i].x();
                            check_temp[2] = render.points_[i + j * ck_i].y();
                            check_temp[3] = render.points_[i + j * ck_i].z();
                            check_temp[4] = render.intensitys_[i + j * ck_i];
                            warp_checks.push_back(check_temp);
                        }
                    }
                    i++, time_check += 10.0;
                    continue;
                } else {
                    i++;
                }
            }
        }

        //输出控制点数据
        std::cout << "export controls and checks..." << std::endl;
        std::ofstream controls_file(path_controls);
        for (int k = 0; k < controls_gcps.size(); k++) {
            controls_file << std::setprecision(15) << controls_pcds[k][0] << " ";
            controls_file << std::setprecision(15) << controls_pcds[k][1] << " ";
            controls_file << std::setprecision(15) << controls_pcds[k][2] << " ";
            controls_file << std::setprecision(15) << controls_pcds[k][3] << " ";           

            controls_file << std::setprecision(15) << controls_gcps[k][1] << " ";
            controls_file << std::setprecision(15) << controls_gcps[k][2] << " ";
            controls_file << std::setprecision(15) << controls_gcps[k][3] << " ";

            controls_file << std::setprecision(15) << controls_pcds[k][4] << " ";
            controls_file << std::setprecision(15) << controls_gcps[k][4];
            controls_file << "\n ";
        }
        controls_file.close();

        // truth las controls
        std::ofstream controls_file_t("gcp.txt");
        for (int k = 0; k < controls_gcps.size(); k++) {
            controls_file_t << std::setprecision(15) << controls_gcps[k][0] << " ";
            controls_file_t << std::setprecision(15) << controls_gcps[k][1] << " ";
            controls_file_t << std::setprecision(15) << controls_gcps[k][2] << " ";
            controls_file_t << std::setprecision(15) << controls_gcps[k][3] << " ";
            controls_file_t << std::setprecision(15) << controls_gcps[k][4] << " ";
            controls_file_t << "\n ";
        }
        controls_file_t.close();

        std::ofstream controls_file_w("mms.txt");
        for (int k = 0; k < controls_gcps.size(); k++) {
            controls_file_w << std::setprecision(15) << controls_pcds[k][0] << " ";
            controls_file_w << std::setprecision(15) << controls_pcds[k][1] << " ";
            controls_file_w << std::setprecision(15) << controls_pcds[k][2] << " ";
            controls_file_w << std::setprecision(15) << controls_pcds[k][3] << " ";
            controls_file_w << std::setprecision(15) << controls_pcds[k][4] << " ";
            controls_file_w << "\n ";
        }
        controls_file_w.close();



        //纠正POS并输出纠正后的点云
        std::cout << "rectify the pos and warp laz..." << std::endl;
        Pos pos_warp, pos_rec;
        pos_warp.load_pos(path_warp_pos);
        std::string pos_rec_file = "pos_rec.pos";
        pos_warp.export_rectified_pos(path_controls, pos_rec_file);

        HsrFile file(path_hsr);
        file.set_config(path_config);
        file.set_body(path_config);

        std::string name = get_filename_noext(path_hsr);
        std::string dir = get_directory(path_hsr);
        name = join_paths(dir, name +std::to_string((int)time_interval)+ "s-rec.laz");
        pos_rec.load_pos(pos_rec_file);
        file.export_las_ooc(name, pos_rec);


        //加载纠正后的点云，提取检查点
        std::cout << "extract checks from the rectified laz..." << std::endl;
        render.load_las(name);
        std::vector<std::vector<double>> rec_checks;
        {
            int i = 0, ck_i = 150000;
            double min_t, mid;
            int num_points = render.gps_times_.size();
            double time_range0 = time_duration.x()+1, time_range1 = time_duration.y();
            while (time_range0 < time_range1 && i < num_points) {
                min_t = std::fabs(time_range0 - render.gps_times_[i]) -
                        std::fabs(time_range0 - render.gps_times_[i + 1]);
                if (min_t < 0.0 && (i + 6 * ck_i) < num_points) {
                    for (int j = 0; j < 6; j++) {
                        std::vector<double> control_temp(5);
                        if (std::fabs(render.gps_times_[i + j * ck_i] - render.gps_times_[i]) < 5.0) {
                            //激光点频实际为20万点/秒，每组6个控制点保证在2秒范围内
                            control_temp[0] = render.gps_times_[i + j * ck_i];
                            control_temp[1] = render.points_[i + j * ck_i].x();
                            control_temp[2] = render.points_[i + j * ck_i].y();
                            control_temp[3] = render.points_[i + j * ck_i].z();
                            control_temp[4] = render.intensitys_[i + j * ck_i];
                            rec_checks.push_back(control_temp);
                        }
                    }
                    i++, time_range0 += 10.0;
                    continue;
                } else {
                    i++;
                }
            }
        }
        
        //输出检查点数据
        int c_num = 0;
        double c_rmse_r, c_rmse_w;
        std::ofstream checks_file(path_checks);
        checks_file << "time"
                    << ","
                    << "truth_x"
                    << ","
                    << "truth_y"
                    << ","
                    << "truth_z"
                    << ","
                    << "warp_x"
                    << ","
                    << "warp_y"
                    << ","
                    << "warp_z"
                    << ","
                    << "rec_x"
                    << ","
                    << "rec_y"
                    << ","
                    << "rec_z"
                    << ","
                    << "warp_x-truth_x"
                    << ","
                    << "warp_y-truth_y"
                    << ","
                    << "warp_z-truth_z"
                    << ","
                    << "rec_x-truth_x"
                    << ","
                    << "rec_y-truth_y"
                    << ","
                    << "rec_z-truth_z"
                    << ","
                    << "Time"
                    << ","
                    << "Dev. origin"
                    << ","
                    << "Dev. Rectified"
                    << ","
                    << "rmse_wx"
                    << ","
                    << "rmse_wy"
                    << ","
                    << "rmse_wz"
                    << ","
                    << "rmse_rx"
                    << ","
                    << "rmse_ry"
                    << ","
                    << "rmse_rz"
                    << ","
                    << "rmse_wxyz"
                    << ","
                    << "rmse_rxyz"
                    << "\n";
        for (int k = 0; k < truth_checks.size(); k++) {
            
            if (truth_checks[k][0] == warp_checks[k][0] && truth_checks[k][0] == rec_checks[k][0]) {
                c_num++;
                checks_file << std::setprecision(15) << truth_checks[k][0] << ",";

                checks_file << std::setprecision(15) << truth_checks[k][1] << ",";
                checks_file << std::setprecision(15) << truth_checks[k][2] << ",";
                checks_file << std::setprecision(15) << truth_checks[k][3] << ",";

                checks_file << std::setprecision(15) << warp_checks[k][1] << ",";
                checks_file << std::setprecision(15) << warp_checks[k][2] << ",";
                checks_file << std::setprecision(15) << warp_checks[k][3] << ",";

                checks_file << std::setprecision(15) << rec_checks[k][1] << ",";
                checks_file << std::setprecision(15) << rec_checks[k][2] << ",";
                checks_file << std::setprecision(15) << rec_checks[k][3] << ",";

                checks_file << std::setprecision(15) << warp_checks[k][1] - truth_checks[k][1] << ",";
                checks_file << std::setprecision(15) << warp_checks[k][2] - truth_checks[k][2] << ",";
                checks_file << std::setprecision(15) << warp_checks[k][3] - truth_checks[k][3] << ",";

                checks_file << std::setprecision(15) << rec_checks[k][1] - truth_checks[k][1] << ",";
                checks_file << std::setprecision(15) << rec_checks[k][2] - truth_checks[k][2] << ",";
                checks_file << std::setprecision(15) << rec_checks[k][3] - truth_checks[k][3] << ",";
                double deviation_w, deviation_r, w_x = 0, w_y = 0, w_z = 0, r_x = 0, r_y = 0, r_z = 0;
                deviation_w =
                    std::sqrt((warp_checks[k][1] - truth_checks[k][1]) * (warp_checks[k][1] - truth_checks[k][1]) +
                              (warp_checks[k][2] - truth_checks[k][2]) * (warp_checks[k][2] - truth_checks[k][2]) +
                              (warp_checks[k][3] - truth_checks[k][3]) * (warp_checks[k][3] - truth_checks[k][3]));
                deviation_r =
                    std::sqrt((rec_checks[k][1] - truth_checks[k][1]) * (rec_checks[k][1] - truth_checks[k][1]) +
                              (rec_checks[k][2] - truth_checks[k][2]) * (rec_checks[k][2] - truth_checks[k][2]) +
                              (rec_checks[k][3] - truth_checks[k][3]) * (rec_checks[k][3] - truth_checks[k][3]));

                w_x += (warp_checks[k][1] - truth_checks[k][1]) * (warp_checks[k][1] - truth_checks[k][1]);
                w_y += (warp_checks[k][2] - truth_checks[k][2]) * (warp_checks[k][2] - truth_checks[k][2]);
                w_z += (warp_checks[k][3] - truth_checks[k][3]) * (warp_checks[k][3] - truth_checks[k][3]);

                r_x += (rec_checks[k][1] - truth_checks[k][1]) * (rec_checks[k][1] - truth_checks[k][1]);
                r_y += (rec_checks[k][2] - truth_checks[k][2]) * (rec_checks[k][2] - truth_checks[k][2]);
                r_z += (rec_checks[k][3] - truth_checks[k][3]) * (rec_checks[k][3] - truth_checks[k][3]);

                checks_file << std::setprecision(15) << truth_checks[k][0] << ",";
                checks_file << std::setprecision(15) << deviation_w << ",";
                checks_file << std::setprecision(15) << deviation_r << ",";

                double rmse_wx, rmse_rx;

                if (k % 6 == 0) {
                    checks_file << std::setprecision(15) << std::sqrt(w_x / 6) << ",";
                    checks_file << std::setprecision(15) << std::sqrt(w_y / 6) << ",";
                    checks_file << std::setprecision(15) << std::sqrt(w_z / 6) << ",";
                    checks_file << std::setprecision(15) << std::sqrt(r_x / 6) << ",";
                    checks_file << std::setprecision(15) << std::sqrt(r_y / 6) << ",";
                    checks_file << std::setprecision(15) << std::sqrt(r_z / 6) << ",";

                    checks_file << std::setprecision(15) << std::sqrt((w_x + w_y + w_z) / 6) << ",";
                    checks_file << std::setprecision(15) << std::sqrt((r_x + r_y + r_z) / 6) << ",";

                    w_x = w_y = w_z = r_x = r_y = r_z = 0;
                }
                checks_file << "\n ";
                

                //if (k > 1){
                //    if (truth_checks[k][0] - truth_checks[k - 1][0] > 3) {
                //        std::cout << k << std::endl;
                //        double c_rmse_r, c_rmse_w, c_mean_r_x, c_mean_r_y, c_mean_r_z, c_mean_w_x, c_mean_w_y,
                //            c_mean_w_z, c_max_xy, c_max_z, c_min_xy, c_min_z;
                //        double sum_w_x = 0, sum_w_y = 0, sum_w_z = 0, sum_r_x = 0, sum_r_y = 0, sum_r_z = 0;
                //       
                //        for (int i = 0; i < c_num; i++) {
                //            sum_r_x += rec_checks[k - i][1];
                //            sum_r_y += rec_checks[k - i][2];
                //            sum_r_z += rec_checks[k - i][3];
                //            
                //            sum_w_x += warp_checks[k - i][1];
                //            sum_w_y += warp_checks[k - i][2];
                //            sum_w_z += warp_checks[k - i][3];
                //        }
                //        c_mean_r_x = sum_r_x / c_num;
                //        c_mean_r_y = sum_r_y / c_num;
                //        c_mean_r_z = sum_r_z / c_num;
                //       
                //        c_mean_w_x = sum_w_x / c_num;
                //        c_mean_w_y = sum_w_y / c_num;
                //        c_mean_w_z = sum_w_z / c_num;

                //        double sum_sqare_r = 0, sum_sqare_w = 0;
                //        for (int i = 0; i < c_num; i++) {
                //            sum_sqare_r +=
                //                (c_mean_r_x - truth_checks[k - i][1]) * (c_mean_r_x - truth_checks[k - i][1]) +
                //                (c_mean_r_y - truth_checks[k - i][2]) * (c_mean_r_y - truth_checks[k - i][2]) +
                //                (c_mean_r_z - truth_checks[k - i][3]) * (c_mean_r_z - truth_checks[k - i][3]);
                //            sum_sqare_w +=
                //                (c_mean_w_x - truth_checks[k - i][1]) * (c_mean_w_x - truth_checks[k - i][1]) +
                //                (c_mean_w_y - truth_checks[k - i][2]) * (c_mean_w_y - truth_checks[k - i][2]) +
                //                (c_mean_w_z - truth_checks[k - i][3]) * (c_mean_w_z - truth_checks[k - i][3]);
                //        }
                //        c_rmse_r = std::sqrt(sum_sqare_r / c_num);
                //        c_rmse_w = std::sqrt(sum_sqare_w / c_num);

                //        checks_file << std::setprecision(15) << c_rmse_w << ",";
                //        checks_file << std::setprecision(15) << c_rmse_r << ",";
                //        c_num = 0;//初始化检查点计数
                //    }
                //}
            }
        }
        checks_file.close();
     
    
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
