/*
 * Created Date: Thursday, January 7th 2021, 3:59:26 pm
 * Author: Han
 * 进行两站点云的匹配
 * Copyright (c) 2021 Vrlab
 */
#include "lidar_matcher.h"
#include "lidar_matcher_proxy.h"

#include <base/base.h>
#include <base/json.h>

namespace h2o {

LiDARMatchParam lidar_match_config(const std::string &path) {
    nlohmann::json j = read_json_file(path);
    if (j.count("LiDARMatch") == 0) {
        return LiDARMatchParam();
    }
    j = j["LiDARMatch"];
    LiDARMatchParam param;
    if (j.count("VoxelSize"))
        param.voxel_size = j["VoxelSize"];
    if (j.count("IcpRadius"))
        param.icp_radius = j["IcpRadius"];
    if (j.count("IcpMaxKeypointDistance"))
        param.icp_max_keypoint_distance = j["IcpMaxKeypointDistance"];
    if (j.count("IcpMaxDistance"))
        param.icp_max_distance = j["IcpMaxDistance"];
    if (j.count("IcpMaxPoints"))
        param.icp_max_points = j["IcpMaxPoints"];
    if (j.count("IcpThresholdOverlap"))
        param.icp_t_overlap = j["IcpThresholdOverlap"];
    if (j.count("IcpThresholdMinimumPoints"))
        param.icp_t_need_points = j["IcpThresholdMinimumPoints"];
    if (j.count("IcpUseCoarse"))
        param.icp_use_initial = j["IcpUseCoarse"];
    if (j.count("NormalRadius"))
        param.ne_radius = j["NormalRadius"];
    if (j.count("NormalMaxKnn"))
        param.ne_max_knn = j["NormalMaxKnn"];
    if (j.count("KeypointRadius"))
        param.key_radius = j["KeypointRadius"];
    if (j.count("KeyGamma"))
        param.key_gamma = j["KeyGamma"];
    if (j.count("ShowMatch"))
        param.show_match = j["ShowMatch"];

    return param;
}

LiDARMatches lidar_match(const std::string &las_i, const std::string &las_j, const std::string &pos_i,
                         const std::string &pos_j, const LiDARMatchParam &param) {
    LiDARMatcherProxy matcher(las_i, las_j, pos_i, pos_j);
    matcher.set_param(param);
    return matcher.match();
}

} // namespace h2o
