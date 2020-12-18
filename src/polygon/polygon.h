/*
 * Filename: /home/han/Documents/LinSAR/src/polygon/polygon.h
 * Path: /home/han/Documents/LinSAR/src/polygon
 * Created Date: Sunday, December 8th 2019, 7:03:07 pm
 * Author: han
 * 多边形类别，底层采用Clipper
 * Copyright (c) 2019 Vrlab
 */
#pragma once

#include <base/clipper.hpp>
#include <base/common.h>
#include <vector>

#include <QPainterPath>

namespace h2o {

struct CameraParameters;

// Editing is polygon for interactive editing
enum PolygonType { PolygonTypePolygon = 0, PolygonTypePolyline = 1 };

// 多边形点，边或中心的删除图标
enum PolygonDecoration {
    PolygonNone = 0x0000,
    PolygonVertex = 0x0001,
    PolygonEdge = 0x0002,
    PolygonCenter = 0x0004,
    PolygonAllDecoration = PolygonVertex | PolygonEdge | PolygonCenter
};

using ClipperPoint = ClipperLib::IntPoint;
using Path = ClipperLib::Path;
using Paths = ClipperLib::Paths;
using PolygonRing = std::vector<Vector2f>;
using PolygonRings = std::vector<PolygonRing>;
using PolygonHeight = std::vector<float>;
using PolygonHeights = std::vector<PolygonHeight>;

// 多边形顶点的坐标信息，对带洞多边形，first - 第 i 个多边形，second - 第 j 个顶点
using PolygonVertexIndex = std::pair<uint32_t, uint32_t>;

class Polygon;
using PolygonPtr = std::shared_ptr<Polygon>;

PolygonPtr create_polygon();
PolygonPtr create_polyline();

Vector2f cpoint_to_point(const ClipperPoint &cpoint, int64_t scale = 1000);
ClipperPoint point_to_cpoint(const Vector2f &point, int64_t scale = 1000);
PolygonRing path_to_ring(const Path &path, int64_t scale = 1000);
PolygonRings paths_to_rings(const Paths &paths, int64_t scale = 1000);

// 可以顾及多个多边形，从布尔运算获得的 paths 可能包含多个
std::vector<PolygonRings> paths_to_multirings(const ClipperLib::Paths &paths, int64_t scale = 1000);
std::vector<Paths> paths_to_multipaths(const Paths &paths);

Path ring_to_path(const PolygonRing &ring, int64_t scale = 1000);
Paths rings_to_paths(const PolygonRings &rings, int64_t scale = 1000);

QPainterPath ring_to_painter(const PolygonRing &ring, bool close_sub_path = false);
QPainterPath rings_to_painter(const PolygonRings &rings, bool close_sub_path = false);
QPainterPath path_to_painter(const Path &path, bool close_sub_path = false, int64_t scale = 1000);
QPainterPath paths_to_painter(const Paths &paths, bool close_sub_path = false, int64_t scale = 1000);

float path_area(const Path &path, int64_t scale = 1000);
float ring_aera(const PolygonRing &ring);

// 提取当前 mask 的多边形
// decimode 是 opencv 的消息，如 CHAIN_APPROXIMATE_NODE
// offset 是像素的偏移值，对 region grow 产生的区域， 可能需要进行设置
std::vector<PolygonRings> find_contours(const cv::Mat &mask, int decimode, const Vector2i &offset = Vector2i::Zero());

// 寻找点云的 2D 凸包
Path find_hull(const MatrixXf &points, int64_t scale = 1000);

struct PolygonMesh {
    // faces indices into vertices
    MatrixXu F;
    // lines indices into vertices
    MatrixXu L;
    // the boundary vertices
    MatrixXf V;
};

class Polygon {
public:
    static constexpr int64_t SCALE = 1000;

public:
    Polygon() {}
    ~Polygon() {}

public:
    /*************************************************************************************/
    // operations for object and screen space
    virtual void set_camera(CameraParameters *camera) { camera_ = camera; }

    // 屏幕一像素在现实中是几米
    virtual float get_scale() const;

    /*************************************************************************************/
    // operation for get and setters
    virtual void set_paths(const Paths &paths) { paths_ = paths; };

    virtual void set_outer_ring(const PolygonRing &ring) = 0;

    virtual void add_hole(const PolygonRing &hole) = 0;

    virtual void set_heights(const PolygonHeights &heights) { heights_ = heights; };

    virtual void init_bounds();

    /*************************************************************************************/
    // operations for edit
    // 移动多边形的某个点顶点，只改变位置
    virtual void move_point(const PolygonVertexIndex &index, const Vector2f &point) = 0;

    // 在某个多边形的末尾，增加一个点
    virtual void add_point(const PolygonVertexIndex &index, const Vector2f &point) = 0;

    // 在某个多边形的中点，增加一个分割点
    virtual void sep_point(const PolygonVertexIndex &index) = 0;

    // 删除某个端点
    virtual void del_point(const PolygonVertexIndex &index) = 0;

    /*************************************************************************************/
    // operations for selections
    // 选择顶点，如果不成功 first 是 INVALID_INDEX
    virtual PolygonVertexIndex select_vertex(const Vector2f &point2d, int threshold = 5) const = 0;

    // 选择边中心，如果不成功 first 是 INVALID_INDEX
    virtual PolygonVertexIndex select_edge(const Vector2f &point2d, int threshold = 5) const = 0;

    // select the center
    virtual PolygonVertexIndex select_center(const Vector2f &point2d, int threshold = 5) const = 0;

    // 选择任意内容，不必做多次转换，可能会快一点，选择顺序 是 顶点 -> 边界中点 -> 中心
    virtual std::tuple<PolygonDecoration, PolygonVertexIndex> select(const Vector2f &point2d, int threshold = 5) const;

    /*************************************************************************************/
    // operations for testing hit
    // 根据世界坐标系的平面坐标判断某个多边形是否选中
    virtual bool is_selected(const Vector2f &point2d, int threshold = 5) const = 0;

    // 中心点是否选择到
    virtual bool is_center_selected(const Vector2f &point2d, int threshold = 5) const = 0;

    // 当前点是否在 多边形之中
    virtual bool is_contained(const Vector2f &point2d) const = 0;

    /*************************************************************************************/
    // getters
    virtual bool has_3d() const { return !heights_.empty(); };

    virtual Paths get_paths() const { return paths_; }
    virtual Paths &get_paths() { return paths_; }

    virtual PolygonHeights get_heights() const { return heights_; }
    virtual PolygonHeights &get_heights() { return heights_; }

    virtual CameraParameters *get_camera() { return camera_; }

    // 获取是线 还是 带洞多边形
    virtual PolygonType get_polygon_type() const = 0;

    virtual PolygonRings convert_to_rings() const = 0;

    // 更新 path 信息
    virtual PolygonMesh triangulate() const = 0;
    virtual QPainterPath get_painterpath() const = 0;

    // 获取包围盒
    virtual BoundingBox2f get_bounds() const { return bounds2d_; };
    virtual BoundingBox3f get_bounds3d() const { return bounds3d_; }

    // 多边形的面积
    virtual float area() const = 0;

    // polygon - outer shell, polyline - length
    virtual float length() const = 0;

    virtual std::shared_ptr<Polygon> clone() const = 0;

public:
    // for polygon the first is outer and rest are holes
    // for polyline only one path is supported
    Paths paths_;
    PolygonHeights heights_;

    // should be initialized after set paths
    BoundingBox2f bounds2d_;
    BoundingBox3f bounds3d_;

    // used to convert from object to screen space, important for hit test
    CameraParameters *camera_;
};

} // namespace h2o
