/*
 * Created Date: Tuesday, September 8th 2020, 11:19:48 am
 * Author: han
 * 基于 poisson 的点云内插
 * Copyright (c) 2020 Vrlab
 */

#include <AdaptiveMultigrid/PreProcessor.h>

#define DEFAULT_FEM_BOUNDARY BOUNDARY_FREE // The default finite-element boundary type

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <AdaptiveMultigrid/FEMTree.h>
#include <AdaptiveMultigrid/MyMiscellany.h>
#include <AdaptiveMultigrid/PPolynomial.h>
#include <AdaptiveMultigrid/RegularGrid.h>
#include <AdaptiveMultigrid/VertexFactory.h>

#include "interpolation.h"

#include <base/base.h>
#include <polygon/polygon.h>
#include <polygon/polygon_draw.h>

namespace poisson_am_interpolate {

struct AdaptiveMultigridParam {
    int depth = 8;
    int iters = 8;
    int full_depth = 5;

    int base_vcyles = 4;

    float scale = 1.1f;

    float cg_accuracy = 1e-3f;
    float value_weight = 1000.0f;
    float gradient_weight = 0.0f;
    float lap_weigt = 0.f;
    float bilap_weight = 100.00f;
};

template <class Real, unsigned int Dim>
XForm<Real, Dim + 1> GetBoundingBoxXForm(Point<Real, Dim> min, Point<Real, Dim> max, Real scaleFactor) {
    Point<Real, Dim> center = (max + min) / 2;
    Real scale = max[0] - min[0];
    for (int d = 1; d < Dim; d++)
        scale = std::max<Real>(scale, max[d] - min[d]);
    scale *= scaleFactor;
    for (int i = 0; i < Dim; i++)
        center[i] -= scale / 2;
    XForm<Real, Dim + 1> tXForm = XForm<Real, Dim + 1>::Identity(), sXForm = XForm<Real, Dim + 1>::Identity();
    for (int i = 0; i < Dim; i++)
        sXForm(i, i) = (Real)(1. / scale), tXForm(Dim, i) = -center[i];
    return sXForm * tXForm;
}

template <class Real, unsigned int Dim>
XForm<Real, Dim + 1> GetBoundingBoxXForm(Point<Real, Dim> min, Point<Real, Dim> max, Real width, Real scaleFactor,
                                         int &depth) {
    // Get the target resolution (along the largest dimension)
    Real resolution = (max[0] - min[0]) / width;
    for (int d = 1; d < Dim; d++)
        resolution = std::max<Real>(resolution, (max[d] - min[d]) / width);
    resolution *= scaleFactor;
    depth = 0;
    while ((1 << depth) < resolution)
        depth++;

    Point<Real, Dim> center = (max + min) / 2;
    Real scale = (1 << depth) * width;

    for (int i = 0; i < Dim; i++)
        center[i] -= scale / 2;
    XForm<Real, Dim + 1> tXForm = XForm<Real, Dim + 1>::Identity(), sXForm = XForm<Real, Dim + 1>::Identity();
    for (int i = 0; i < Dim; i++)
        sXForm(i, i) = (Real)(1. / scale), tXForm(Dim, i) = -center[i];
    return sXForm * tXForm;
}

template <typename Real, unsigned int Dim, typename FunctionValueType>
using InputPointStreamInfo = typename FEMTreeInitializer<Dim, Real>::template InputPointStream<FunctionValueType>;

template <typename Real, unsigned int Dim, typename FunctionValueType>
using InputPointStream = typename InputPointStreamInfo<Real, Dim, FunctionValueType>::StreamType;

template <class Real, unsigned int Dim, typename FunctionValueType>
XForm<Real, Dim + 1> GetPointXForm(InputPointStream<Real, Dim, FunctionValueType> &stream, Real width, Real scaleFactor,
                                   int &depth) {
    Point<Real, Dim> min, max;
    InputPointStreamInfo<Real, Dim, FunctionValueType>::BoundingBox(stream, min, max);
    return GetBoundingBoxXForm(min, max, width, scaleFactor, depth);
}
template <class Real, unsigned int Dim, typename FunctionValueType>
XForm<Real, Dim + 1> GetPointXForm(InputPointStream<Real, Dim, FunctionValueType> &stream, Real scaleFactor) {
    Point<Real, Dim> min, max;
    InputPointStreamInfo<Real, Dim, FunctionValueType>::BoundingBox(stream, min, max);
    return GetBoundingBoxXForm(min, max, scaleFactor);
}

template <unsigned int Dim, typename Real, typename PointSampleData> struct ConstraintDual;

template <unsigned int Dim, typename Real> struct ConstraintDual<Dim, Real, VectorTypeUnion<Real, Real>> {
    typedef VectorTypeUnion<Real, Real> PointSampleData;
    Real vWeight;
    ConstraintDual(Real v) : vWeight(v) {}
    CumulativeDerivativeValues<Real, Dim, 0> operator()(const Point<Real, Dim> &p, const PointSampleData &data) const {
        Real value = data.template get<0>();
        CumulativeDerivativeValues<Real, Dim, 0> cdv;
        cdv[0] = value * vWeight;
        return cdv;
    }
};

template <unsigned int Dim, typename Real> struct ConstraintDual<Dim, Real, VectorTypeUnion<Real, Point<Real, Dim>>> {
    typedef VectorTypeUnion<Real, Point<Real, Dim>> PointSampleData;
    Real gWeight;
    ConstraintDual(Real g) : gWeight(g) {}
    CumulativeDerivativeValues<Real, Dim, 1> operator()(const Point<Real, Dim> &p, const PointSampleData &data) const {
        Point<Real, Dim> gradient = data.template get<0>();
        CumulativeDerivativeValues<Real, Dim, 1> cdv;
        for (int d = 0; d < Dim; d++)
            cdv[1 + d] = gradient[d] * gWeight;
        return cdv;
    }
};

template <unsigned int Dim, typename Real, typename TotalPointSampleData> struct SystemDual;

template <unsigned int Dim, typename Real> struct SystemDual<Dim, Real, VectorTypeUnion<Real, Real>> {
    CumulativeDerivativeValues<Real, Dim, 0> weight;
    SystemDual(Real v) { weight[0] = v; }
    CumulativeDerivativeValues<Real, Dim, 0> operator()(Point<Real, Dim> p, const VectorTypeUnion<Real, Real> &data,
                                                        const CumulativeDerivativeValues<Real, Dim, 0> &dValues) const {
        return dValues * weight;
    }
    CumulativeDerivativeValues<double, Dim, 0>
    operator()(Point<Real, Dim> p, const VectorTypeUnion<Real, Real> &data,
               const CumulativeDerivativeValues<double, Dim, 0> &dValues) const {
        return dValues * weight;
    };
};

template <unsigned int Dim> struct SystemDual<Dim, double, VectorTypeUnion<double, double>> {
    typedef double Real;
    CumulativeDerivativeValues<Real, Dim, 0> weight;
    SystemDual(Real v) { weight[0] = v; }
    CumulativeDerivativeValues<Real, Dim, 0> operator()(Point<Real, Dim> p, const VectorTypeUnion<double, double> &data,
                                                        const CumulativeDerivativeValues<Real, Dim, 0> &dValues) const {
        return dValues * weight;
    }
};

template <unsigned int Dim, typename Real> struct SystemDual<Dim, Real, VectorTypeUnion<Real, Point<Real, Dim>>> {
    CumulativeDerivativeValues<Real, Dim, 1> weight;
    SystemDual(Real g) {
        weight[0] = 0;
        for (int d = 0; d < Dim; d++)
            weight[d + 1] = g;
    }
    CumulativeDerivativeValues<Real, Dim, 1> operator()(Point<Real, Dim> p,
                                                        const VectorTypeUnion<Real, Point<Real, Dim>> &data,
                                                        const CumulativeDerivativeValues<Real, Dim, 1> &dValues) const {
        return dValues * weight;
    }
    CumulativeDerivativeValues<double, Dim, 1>
    operator()(Point<Real, Dim> p, const VectorTypeUnion<Real, Point<Real, Dim>> &data,
               const CumulativeDerivativeValues<double, Dim, 1> &dValues) const {
        return dValues * weight;
    };
};

template <unsigned int Dim> struct SystemDual<Dim, double, VectorTypeUnion<double, Point<double, Dim>>> {
    typedef double Real;
    CumulativeDerivativeValues<Real, Dim, 1> weight;
    SystemDual(Real g) {
        weight[0] = 0;
        for (int d = 0; d < Dim; d++)
            weight[1 + d] = g;
    }
    CumulativeDerivativeValues<Real, Dim, 1> operator()(Point<Real, Dim> p,
                                                        const VectorTypeUnion<Real, Point<Real, Dim>> &data,
                                                        const CumulativeDerivativeValues<Real, Dim, 1> &dValues) const {
        return dValues * weight;
    }
};

/**
 * \brief interpolate a raster with both value and gradients constraints
 * \param[in] values, 3 x N matrix for values, each column is (x, y, v)
 * \param[in] gradients, 4 x N matrix for gradients, each column is (x, y, dx, dy)
 * \param[in] dsize, desired output size of the image
 * \param[in] param, parameters
 */
template <class Real, unsigned int... FEMSigs>
h2o::GeoImagePtr Execute(const h2o::MatrixXf &values, const h2o::MatrixXf &gradients, double gsd,
                         const AdaptiveMultigridParam &param, UIntPack<FEMSigs...>) {
    static const int Dim = sizeof...(FEMSigs);
    typedef UIntPack<FEMSigs...> Sigs;
    typedef UIntPack<FEMSignature<FEMSigs>::Degree...> Degrees;

    typedef typename FEMTree<Dim, Real>::template InterpolationInfo<Real, 0> ValueInterpolationInfo;
    typedef typename FEMTree<Dim, Real>::template InterpolationInfo<Real, 1> GradientInterpolationInfo;

    // The factory for constructing the function's values
    typedef VertexFactory::Factory<Real, VertexFactory::ValueFactory<Real>> FunctionValueFactory;
    // The factory for constructing the function's gradients
    typedef VertexFactory::Factory<Real, VertexFactory::NormalFactory<Real, Dim>> FunctionGradientFactory;

    // The factory for constructing the value data
    typedef VertexFactory::Factory<Real, VertexFactory::PositionFactory<Real, Dim>, FunctionValueFactory>
        InputSampleValueFactory;
    // The factory for constructing the gradient data
    typedef VertexFactory::Factory<Real, VertexFactory::PositionFactory<Real, Dim>, FunctionGradientFactory>
        InputSampleGradientFactory;

    // The type of the function values
    typedef typename FunctionValueFactory::VertexType FunctionValueType;
    // The type of the function gradients
    typedef typename FunctionGradientFactory::VertexType FunctionGradientType;

    // The type of the input value
    typedef typename InputSampleValueFactory::VertexType InputSampleValueType;
    // The type of the input gradient
    typedef typename InputSampleGradientFactory::VertexType InputSampleGradientType;

    typedef InputDataStream<InputSampleValueType> InputPointValueStream;
    typedef TransformedInputDataStream<InputSampleValueType> XInputPointValueStream;
    typedef InputDataStream<InputSampleGradientType> InputPointGradientStream;
    typedef TransformedInputDataStream<InputSampleGradientType> XInputPointGradientStream;

    FunctionValueFactory functionValueFactory;
    FunctionGradientFactory functionGradientFactory;

    InputSampleValueFactory inputSampleValueFactory(VertexFactory::PositionFactory<Real, Dim>(), functionValueFactory);
    InputSampleGradientFactory inputSampleGradientFactory(VertexFactory::PositionFactory<Real, Dim>(),
                                                          functionGradientFactory);

    typedef RegularTreeNode<Dim, FEMTreeNodeData, depth_and_offset_type> FEMTreeNode;

    ThreadPool::Init(ThreadPool::OPEN_MP);

    XForm<Real, Dim + 1> modelToUnitCube, unitCubeToModel;
    modelToUnitCube = XForm<Real, Dim + 1>::Identity();

    FEMTree<Dim, Real> tree(MEMORY_ALLOCATOR_BLOCK_SIZE);

    size_t pointValueCount, pointGradientCount;

    std::vector<typename FEMTree<Dim, Real>::PointSample> *valueSamples = NULL;
    std::vector<FunctionValueType> *valueSampleData = NULL;
    std::vector<typename FEMTree<Dim, Real>::PointSample> *gradientSamples = NULL;
    std::vector<FunctionGradientType> *gradientSampleData = NULL;
    std::vector<InputSampleValueType> inCorePointsAndValues(values.cols());
    std::vector<InputSampleGradientType> inCorePointsAndGradients(gradients.cols());

    // Read in the samples
    {
        InputPointValueStream *pointValueStream = NULL;
        InputPointGradientStream *pointGradientStream = NULL;
        Point<Real, Dim> valueMin, valueMax, gradientMin, gradientMax;
        {
            // 首先读取 value
            valueSampleData = new std::vector<FunctionValueType>();
            // 读取数据
            for (int i = 0; i < inCorePointsAndValues.size(); ++i) {
                Point<Real, Dim> point(values(0, i), values(1, i));
                FunctionValueType value = values(2, i);
                InputSampleValueType sample(point, value);
                inCorePointsAndValues[i] = sample;
            }

            pointValueStream = new MemoryInputDataStream<InputSampleValueType>(inCorePointsAndValues.size(),
                                                                               &inCorePointsAndValues[0]);

            typename InputSampleValueFactory::Transform _modelToUnitCube(modelToUnitCube);
            auto XFormFunctor = [&](InputSampleValueType &p) { p = _modelToUnitCube(p); };
            XInputPointValueStream _pointStream(XFormFunctor, *pointValueStream);
            InputPointStreamInfo<Real, Dim, FunctionValueType>::BoundingBox(_pointStream, valueMin, valueMax);
        }
        if (gradients.cols() > 0) {
            gradientSampleData = new std::vector<FunctionGradientType>();
            for (int i = 0; i < inCorePointsAndGradients.size(); ++i) {
                Point<Real, Dim> point(gradients(0, i), gradients(1, i));
                Point<Real, Dim> gradient(gradients(2, i), gradients(3, i));
                InputSampleGradientType sample(point, gradient);
                inCorePointsAndGradients[i] = sample;
            }
            pointGradientStream = new MemoryInputDataStream<InputSampleGradientType>(inCorePointsAndGradients.size(),
                                                                                     &inCorePointsAndGradients[0]);
            typename InputSampleGradientFactory::Transform _modelToUnitCube(modelToUnitCube);
            auto XFormFunctor = [&](InputSampleGradientType &p) { p = _modelToUnitCube(p); };
            XInputPointGradientStream _pointStream(XFormFunctor, *pointGradientStream);
            InputPointStreamInfo<Real, Dim, FunctionGradientType>::BoundingBox(_pointStream, gradientMin, gradientMax);
        }

        {
            Point<Real, Dim> min, max;
            min = valueMin, max = valueMax;
            if (gradients.cols() > 0) {
                for (int d = 0; d < Dim; d++)
                    min[d] = std::min<Real>(min[d], gradientMin[d]), max[d] = std::max<Real>(max[d], gradientMax[d]);
            }
            modelToUnitCube =
                param.scale > 0 ? GetBoundingBoxXForm(min, max, (Real)param.scale) * modelToUnitCube : modelToUnitCube;
            unitCubeToModel = modelToUnitCube.inverse();
        }

        {
            valueSamples = new std::vector<typename FEMTree<Dim, Real>::PointSample>();
            typename InputSampleValueFactory::Transform _modelToUnitCube(modelToUnitCube);
            auto XFormFunctor = [&](InputSampleValueType &p) { p = _modelToUnitCube(p); };
            XInputPointValueStream _pointStream(XFormFunctor, *pointValueStream);
            auto ProcessData = [](const Point<Real, Dim> &p, FunctionValueType &d) { return (Real)1.; };
            pointValueCount = FEMTreeInitializer<Dim, Real>::template Initialize<FunctionValueType>(
                tree.spaceRoot(), _pointStream, param.depth, *valueSamples, *valueSampleData, true,
                tree.nodeAllocators.size() ? tree.nodeAllocators[0] : NULL, tree.initializer(), ProcessData);
            delete pointValueStream;
        }

        if (gradients.cols() > 0) {
            gradientSamples = new std::vector<typename FEMTree<Dim, Real>::PointSample>();
            typename InputSampleGradientFactory::Transform _modelToUnitCube(modelToUnitCube);
            auto XFormFunctor = [&](InputSampleGradientType &p) { p = _modelToUnitCube(p); };
            XInputPointGradientStream _pointStream(XFormFunctor, *pointGradientStream);
            auto ProcessData = [](const Point<Real, Dim> &p, FunctionGradientType &d) { return (Real)1.; };
            pointGradientCount = FEMTreeInitializer<Dim, Real>::template Initialize<FunctionGradientType>(
                tree.spaceRoot(), _pointStream, param.depth, *gradientSamples, *gradientSampleData, true,
                tree.nodeAllocators.size() ? tree.nodeAllocators[0] : NULL, tree.initializer(), ProcessData);
            delete pointGradientStream;
        } else
            pointGradientCount = 0;
    }

    DenseNodeData<Real, Sigs> solution;
    {
        DenseNodeData<Real, Sigs> constraints;
        ValueInterpolationInfo *valueInterpolationInfo = NULL;
        GradientInterpolationInfo *gradientInterpolationInfo = NULL;
        int solveDepth = param.depth;

        tree.resetNodeIndices(0);

        {
            valueInterpolationInfo =
                FEMTree<Dim, Real>::template InitializeApproximatePointAndDataInterpolationInfo<Real, FunctionValueType,
                                                                                                0>(
                    tree, *valueSamples, GetPointer(*valueSampleData),
                    ConstraintDual<Dim, Real, FunctionValueType>((Real)param.value_weight),
                    SystemDual<Dim, Real, FunctionValueType>((Real)param.value_weight), true, 1);
        }
        if (gradients.cols() > 0) {
            gradientInterpolationInfo = FEMTree<Dim, Real>::template InitializeApproximatePointAndDataInterpolationInfo<
                Real, FunctionGradientType, 1>(
                tree, *gradientSamples, GetPointer(*gradientSampleData),
                ConstraintDual<Dim, Real, FunctionGradientType>((Real)param.gradient_weight),
                SystemDual<Dim, Real, FunctionGradientType>((Real)param.gradient_weight), true, 1);
        }

        // Prepare for multigrid
        {
            if (values.cols() > 0 && gradients.cols() == 0) {
                tree.template finalizeForMultigrid<Degrees::Max(), Degrees::Max()>(
                    param.full_depth, param.full_depth, [](const FEMTreeNode *) { return true; },
                    [](const FEMTreeNode *) { return false; }, std::make_tuple(valueInterpolationInfo));
            } else if (values.cols() > 0 && gradients.cols() > 0) {
                tree.template finalizeForMultigrid<Degrees::Max(), Degrees::Max()>(
                    param.full_depth, param.full_depth, [](const FEMTreeNode *) { return true; },
                    [](const FEMTreeNode *) { return false; },
                    std::make_tuple(valueInterpolationInfo, gradientInterpolationInfo));
            }
        }

        // Add the interpolation constraints
        {
            constraints = tree.initDenseNodeData(Sigs());
            if (values.cols() > 0 && gradients.cols() > 0)
                tree.addInterpolationConstraints(constraints, solveDepth,
                                                 std::make_tuple(valueInterpolationInfo, gradientInterpolationInfo));
            else if (values.cols() > 0)
                tree.addInterpolationConstraints(constraints, solveDepth, std::make_tuple(valueInterpolationInfo));
        }

        // Solve the linear system
        {
            typename FEMTree<Dim, Real>::SolverInfo sInfo;
            sInfo.cgDepth = 0, sInfo.cascadic = true, sInfo.vCycles = 1, sInfo.iters = param.iters,
            sInfo.cgAccuracy = param.cg_accuracy, sInfo.verbose = false, sInfo.showResidual = false,
            sInfo.showGlobalResidual = SHOW_GLOBAL_RESIDUAL_NONE, sInfo.sliceBlockSize = 1;
            sInfo.baseVCycles = param.base_vcyles;
            typename FEMIntegrator::template System<Sigs, IsotropicUIntPack<Dim, 2>> F(
                {0., (double)param.lap_weigt, (double)param.bilap_weight});
            if (values.cols() > 0 && gradients.cols() > 0)
                solution = tree.solveSystem(Sigs(), F, constraints, param.depth, sInfo,
                                            std::make_tuple(valueInterpolationInfo, gradientInterpolationInfo));
            else if (values.cols() > 0)
                solution = tree.solveSystem(Sigs(), F, constraints, param.depth, sInfo,
                                            std::make_tuple(valueInterpolationInfo));
            delete valueInterpolationInfo, valueInterpolationInfo = NULL;
            delete gradientInterpolationInfo, gradientInterpolationInfo = NULL;
        }
    }

    delete valueSamples, valueSamples = NULL;
    delete gradientSamples, gradientSamples = NULL;
    delete valueSampleData, valueSampleData = NULL;
    delete gradientSampleData, gradientSampleData = NULL;

    h2o::GeoImagePtr image = std::make_shared<h2o::GeoImage>();
    {
        h2o::BoundingBox3f bounds;
        for (int i = 0; i < values.cols(); ++i) {
            bounds.extend(h2o::Vector3f(values(0, i), values(1, i), values(2, i)));
        }

        int res = 0;
        Pointer(Real) values = tree.template regularGridEvaluate<true>(solution, res, -1, false);
        XForm<Real, Dim + 1> voxelToUnitCube = XForm<Real, Dim + 1>::Identity();
        for (int d = 0; d < Dim; d++)
            voxelToUnitCube(d, d) = (Real)(1. / res), voxelToUnitCube(Dim, d) = (Real)(0.5 / res);

        XForm<Real, Dim + 1> voxelToModel = unitCubeToModel * voxelToUnitCube;
        XForm<Real, Dim + 1> rasterToModel = XForm<Real, Dim + 1>::Identity();
        rasterToModel(0, 0) = gsd;
        rasterToModel(1, 0) = 0.0;
        rasterToModel(2, 0) = bounds.min().x();
        rasterToModel(0, 1) = 0.0;
        rasterToModel(1, 1) = -gsd;
        rasterToModel(2, 1) = bounds.max().y();
        auto modelToRaster = rasterToModel.inverse();

        XForm<Real, Dim + 1> voxelToRaster = modelToRaster * voxelToModel;

        cv::Mat mat_voxel;

        if (std::is_same<Real, float>::value) {
            mat_voxel = cv::Mat(res, res, CV_32F, values);
        } else if (std::is_same<Real, double>::value) {
            mat_voxel = cv::Mat(res, res, CV_64F, values);
            mat_voxel.convertTo(mat_voxel, CV_32F);
        }

        cv::Mat M(2, 3, CV_64F);
        M.at<double>(0, 0) = voxelToRaster(0, 0);
        M.at<double>(0, 1) = voxelToRaster(1, 0);
        M.at<double>(0, 2) = voxelToRaster(2, 0);
        M.at<double>(1, 0) = voxelToRaster(0, 1);
        M.at<double>(1, 1) = voxelToRaster(1, 1);
        M.at<double>(1, 2) = voxelToRaster(2, 1);

        int rows = bounds.sizes().y() / gsd;
        int cols = bounds.sizes().x() / gsd;
        cv::Mat mat_output;
        cv::warpAffine(mat_voxel, mat_output, M, {cols, rows}, cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);

        DeletePointer(values);
        h2o::Matrix23d xform = h2o::Matrix23d::Zero();
        xform(0, 0) = rasterToModel(0, 0);
        xform(0, 1) = rasterToModel(1, 0);
        xform(0, 2) = rasterToModel(2, 0);
        xform(1, 0) = rasterToModel(0, 1);
        xform(1, 1) = rasterToModel(1, 1);
        xform(1, 2) = rasterToModel(2, 1);

        image->init_image(rows, cols, CV_32F, xform);
        image->set_image(mat_output);
    }

    ThreadPool::Terminate();

    return image;
}

} // namespace poisson_am_interpolate

namespace h2o {

GeoImagePtr interpolate_am(const MatrixXf &points, double gsd, double w_v, double w_lap, double w_bilap, int depth) {
    BoundingBox3f bounds;
    for (int i = 0; i < points.cols(); ++i) {
        bounds.extend(Vector3f(points(0, i), points(1, i), points(2, i)));
    }
    bounds.min().z() -= 100.0;
    bounds.max().z() += 100;

    double length = bounds.sizes().head(2).array().maxCoeff();
    length *= 1.2; // 四周要扩大10%

    poisson_am_interpolate::AdaptiveMultigridParam param;
    param.depth = lround(ceil(log(length) / log(2)));
    param.depth = std::min(12, param.depth);
    param.depth = std::max(6, param.depth);

    if (depth >= 8) {
        param.depth = depth;
    }

    param.value_weight = w_v;
    param.lap_weigt = w_lap;
    param.bilap_weight = w_bilap;

    constexpr int FEM_DIMENSION = 2;
    constexpr int FEM_DEGREE = 2;
    using FEMSigs = IsotropicUIntPack<FEM_DIMENSION, FEMDegreeAndBType<FEM_DEGREE, DEFAULT_FEM_BOUNDARY>::Signature>;

    MatrixXf gradients;
    GeoImagePtr image = poisson_am_interpolate::Execute<double>(points, gradients, gsd, param, FEMSigs());

    // 填充无效之区域
    Path path = find_hull(points);
    PolygonRing ring = path_to_ring(path);

    //
    cv::Mat mask = cv::Mat::zeros(image->get_size().y(), image->get_size().x(), CV_8U);
    Matrix23d world_to_pixel = reverse_affine(image->get_xform());
    draw_polygon(mask, Paths{path}, Vector4f(255, 255, 255, 255), world_to_pixel);

    for (int r = 0; r < mask.rows; ++r) {
        float *image_ptr = image->get_image().ptr<float>(r);
        uint8_t *mask_ptr = mask.ptr<uint8_t>(r);
        for (int c = 0; c < mask.cols; ++c) {
            if (image_ptr[c] < bounds.min().z() || image_ptr[c] > bounds.max().z()) {
                mask_ptr[c] = 0;
            }
        }
    }

    image->get_image().setTo(-9999, mask == 0);
    image->set_nodata(-9999);
    image->set_mask(mask);

    return image;
}

GeoImagePtr interpolate_am(const MatrixXf &points, const MatrixXf &gradients, double gsd, double w_v, double w_g,
                           double w_lap, double w_bilap, int depth) {
    BoundingBox3f bounds;
    for (int i = 0; i < points.cols(); ++i) {
        bounds.extend(Vector3f(points(0, i), points(1, i), points(2, i)));
    }
    bounds.min().z() -= 100.0;
    bounds.max().z() += 100;

    double length = bounds.sizes().head(2).array().maxCoeff();
    length *= 1.2; // 四周要扩大10%

    poisson_am_interpolate::AdaptiveMultigridParam param;
    param.depth = lround(ceil(log(length) / log(2)));
    param.depth = std::min(12, param.depth);
    param.depth = std::max(6, param.depth);

    if (depth >= 8) {
        param.depth = depth;
    }

    param.value_weight = w_v;
    param.lap_weigt = w_lap;
    param.bilap_weight = w_bilap;
    param.gradient_weight = w_g;

    constexpr int FEM_DIMENSION = 2;
    constexpr int FEM_DEGREE = 2;
    using FEMSigs = IsotropicUIntPack<FEM_DIMENSION, FEMDegreeAndBType<FEM_DEGREE, DEFAULT_FEM_BOUNDARY>::Signature>;

    GeoImagePtr image = poisson_am_interpolate::Execute<double>(points, gradients, gsd, param, FEMSigs());

    // 填充无效之区域
    Path path = find_hull(points);
    PolygonRing ring = path_to_ring(path);

    //
    cv::Mat mask = cv::Mat::zeros(image->get_size().y(), image->get_size().x(), CV_8U);
    Matrix23d world_to_pixel = reverse_affine(image->get_xform());
    draw_polygon(mask, Paths{path}, Vector4f(255, 255, 255, 255), world_to_pixel);

    for (int r = 0; r < mask.rows; ++r) {
        float *image_ptr = image->get_image().ptr<float>(r);
        uint8_t *mask_ptr = mask.ptr<uint8_t>(r);
        for (int c = 0; c < mask.cols; ++c) {
            if (image_ptr[c] < bounds.min().z() || image_ptr[c] > bounds.max().z()) {
                mask_ptr[c] = 0;
            }
        }
    }

    image->get_image().setTo(-9999, mask == 0);
    image->set_nodata(-9999);
    image->set_mask(mask);

    return image;
}

} // namespace h2o
