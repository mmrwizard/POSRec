/*
 * Created Date: Tuesday, January 12th 2021, 10:50:01 am
 * Author: Han
 * 采用类似 ICP 的策略进行 LiDAR 的精配准
 * Copyright (c) 2021 Vrlab
 */
#pragma once

#include "lidar_calibrate.h"
#include "pos.h"

#include <base/base.h>
#include <nanoflann.hpp>

namespace nanoflann {
template <class VectorType, int DIM = -1, class Distance = nanoflann::metric_L2> struct KDTreeVectorAdaptor {
    using self_t = KDTreeVectorAdaptor<VectorType, DIM, Distance>;
    using num_t = typename VectorType::Scalar;
    using IndexType = typename VectorType::Index;
    using metric_t = typename Distance::template traits<num_t, self_t>::distance_t;
    using index_t = KDTreeSingleIndexAdaptor<metric_t, self_t, DIM, IndexType>;

    index_t *index; //! The kd-tree index for the user to call its methods as usual with any other FLANN index.

    KDTreeVectorAdaptor(const int dimensionality, const std::vector<VectorType> &vec, const int leaf_max_size = 10)
        : m_data(vec) {
        const IndexType dims = dimensionality /*mat.rows()*/;
        if (dims != dimensionality)
            throw std::runtime_error("Error: 'dimensionality' must match column count in data matrix");
        if (DIM > 0 && static_cast<int>(dims) != DIM)
            throw std::runtime_error("Data set dimensionality does not match the 'DIM' template argument");
        index = new index_t(dims, *this, nanoflann::KDTreeSingleIndexAdaptorParams(leaf_max_size));
        index->buildIndex();
    }

private:
    /** Hidden copy constructor, to disallow copying this class (Not implemented) */
    KDTreeVectorAdaptor(const self_t &);

public:
    ~KDTreeVectorAdaptor() { delete index; }

    const std::vector<VectorType> &m_data;

    /** Query for the \a num_closest closest points to a given point (entered as query_point[0:dim-1]).
     *  Note that this is a short-cut method for index->findNeighbors().
     *  The user can also call index->... methods as desired.
     * \note nChecks_IGNORED is ignored but kept for compatibility with the original FLANN interface.
     */
    inline void query(const num_t *query_point, const size_t num_closest, IndexType *out_indices,
                      num_t *out_distances_sq, const int /* nChecks_IGNORED */ = 10) const {
        nanoflann::KNNResultSet<num_t, IndexType> resultSet(num_closest);
        resultSet.init(out_indices, out_distances_sq);
        index->findNeighbors(resultSet, query_point, nanoflann::SearchParams());
    }

    inline void query(const num_t *query_point, const num_t radius, std::vector<std::pair<IndexType, num_t>> &results,
                      const int /* nChecks_IGNORED */ = 10) const {
        index->radiusSearch(query_point, radius * radius, results, nanoflann::SearchParams());
    }

    /** @name Interface expected by KDTreeSingleIndexAdaptor
     * @{ */

    const self_t &derived() const { return *this; }
    self_t &derived() { return *this; }

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return m_data.size(); }

    // Returns the L2 distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the
    // class:
    inline num_t kdtree_distance(const num_t *p1, const IndexType idx_p2, IndexType size) const {
        num_t s = 0;
        for (IndexType i = 0; i < size; i++) {
            const num_t d = p1[i] - m_data[idx_p2](i);
            s += d * d;
        }
        return s;
    }

    // Returns the dim'th component of the idx'th point in the class:
    inline num_t kdtree_get_pt(const IndexType idx, int dim) const { return m_data[idx](dim); }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it
    //   again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX> bool kdtree_get_bbox(BBOX & /*bb*/) const { return false; }
};
} // namespace nanoflann

namespace h2o {

using KDTreeVectorAdaptor = nanoflann::KDTreeVectorAdaptor<Vector3d, 3, nanoflann::metric_L2>;
using KDTreeVectorAdaptorPtr = std::shared_ptr<KDTreeVectorAdaptor>;

struct LiDARCalibrateIcpParam {
    double icp_max_distance = 0.3;
    int icp_max_iteration = 20;

    double std_point = 0.05; // 和 voxel 大小相同
    double std_body_angle = 1 * DEG2RAD;
    double std_body_trans = 0.01;

    double lambda_point = 0.9;
    double lambda_body = 0.1;

    static LiDARCalibrateIcpParam load_config(const std::string &path);
};

class LiDARCalibrateIcp {
public:
    LiDARCalibrateIcp(const std::string &path_las1, const std::string &path_las2, const std::string &path_pos1,
                      const std::string &path_pos2, const LiDARCalibrateIcpParam &param = LiDARCalibrateIcpParam());

    std::tuple<Quaterniond, Vector3d> calibrate();
    void set_body(const Quaterniond &q, const Vector3d &t) {
        q_body_ = q;
        t_body_ = t;
    }

protected:
    void solve();
    // 更新点云信息
    void evaluate();

    std::vector<std::pair<int, int>> knn_match();

protected:
    std::tuple<std::vector<Vector3d>, std::vector<double>> load_and_downsample_las(const std::string &path);

public:
    Quaterniond q_body_;
    Vector3d t_body_;

    Quaterniond dq_;
    Vector3d dt_;

    std::vector<Vector3d> points_i_;
    std::vector<double> gps_i_;

    std::vector<Vector3d> points_j_;
    std::vector<double> gps_j_;

    Pos pos1_;
    Pos pos2_;

    LiDARCalibrateIcpParam param_;

    std::vector<ceres::ResidualBlockId> blocks_;
    std::shared_ptr<ceres::Problem> problem_;
};

} // namespace h2o
