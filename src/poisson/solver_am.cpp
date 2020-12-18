/*
 * Created Date: Monday, August 17th 2020, 9:30:04 am
 * Author: Han
 * 采用 adaptive multigrid 内插
 * Copyright (c) 2020 Vrlab
 */
#include "solver.h"

#include <AdaptiveMultigrid/PreProcessor.h>

#define WEIGHT_DEGREE 2      // The order of the B-Spline used to splat in the weights for density estimation
#define DEFAULT_FEM_DEGREE 2 // The default finite-element degree
#define DEFAULT_FEM_BOUNDARY BOUNDARY_FREE // The default finite-element boundary type
#define DEFAULT_DIMENSION 2                // The dimension of the system

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <AdaptiveMultigrid/FEMTree.h>
#include <AdaptiveMultigrid/MyMiscellany.h>
#include <AdaptiveMultigrid/PPolynomial.h>
#include <AdaptiveMultigrid/RegularGrid.h>
#include <AdaptiveMultigrid/VertexFactory.h>

#include <base/common.h>
#include <base/geo_image.h>
#include <base/pcd.h>

namespace poisson_am_blend {

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
cv::Mat Execute(const h2o::MatrixXf &values, const h2o::MatrixXf &gradients, const h2o::Vector2i &dsize,
                const AdaptiveMultigridParam &param, UIntPack<FEMSigs...>) {
    static const int Dim = sizeof...(FEMSigs);
    typedef UIntPack<FEMSigs...> Sigs;
    typedef UIntPack<FEMSignature<FEMSigs>::Degree...> Degrees;
    typedef UIntPack<
        FEMDegreeAndBType<WEIGHT_DEGREE, DerivativeBoundary<FEMSignature<FEMSigs>::BType, 1>::BType>::Signature...>
        DataSigs;
    typedef typename FEMTree<Dim, Real>::template DensityEstimator<WEIGHT_DEGREE> DensityEstimator;

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

    cv::Mat mat_output(dsize.y(), dsize.x(), CV_32F);
    {
        bool is_primal = true;
        int res = 0;
        Pointer(Real) values = tree.template regularGridEvaluate<true>(solution, res, -1, is_primal);
        XForm<Real, Dim + 1> voxelToUnitCube = XForm<Real, Dim + 1>::Identity();
        if (is_primal) {
            for (int d = 0; d < Dim; d++)
                voxelToUnitCube(d, d) = (Real)(1. / (res - 1));
        } else {
            for (int d = 0; d < Dim; d++)
                voxelToUnitCube(d, d) = (Real)(1. / res), voxelToUnitCube(Dim, d) = (Real)(0.5 / res);
        }

        XForm<Real, Dim + 1> voxelToModel = unitCubeToModel * voxelToUnitCube;
        XForm<Real, Dim + 1> modelToVoxel = voxelToModel.inverse();

        cv::Mat mat_voxel = cv::Mat(res, res, CV_32F, values);
        cv::Mat M(2, 3, CV_64F);
        M.at<double>(0, 0) = voxelToModel(0, 0);
        M.at<double>(0, 1) = voxelToModel(1, 0);
        M.at<double>(0, 2) = voxelToModel(2, 0);
        M.at<double>(1, 0) = voxelToModel(0, 1);
        M.at<double>(1, 1) = voxelToModel(1, 1);
        M.at<double>(1, 2) = voxelToModel(2, 1);

        cv::warpAffine(mat_voxel, mat_output, M, {dsize.x(), dsize.y()}, cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);

        DeletePointer(values);
    }

    ThreadPool::Terminate();

    return mat_output;
}

} // namespace poisson_am_blend

namespace h2o {
cv::Mat poisson_interpolation_am(const cv::Mat &src, const cv::Mat &mask, PoissonSolverOptions options) {
    int num_points = cv::countNonZero(mask == BOUNDARY);
    MatrixXf points(3, num_points);

    int rows = mask.rows;
    int cols = mask.cols;

    int index = 0;
    for (int r = 0; r < rows; ++r) {
        const uint8_t *msk_ptr = mask.ptr<uint8_t>(r);
        const float *src_ptr = src.ptr<float>(r);
        for (int c = 0; c < cols; ++c) {
            if (msk_ptr[c] != BOUNDARY) {
                continue;
            }
            points(0, index) = c;
            points(1, index) = r;
            points(2, index) = src_ptr[c];
            index++;
        }
    }

    CHECK(index == num_points);
    double length = std::max(rows, cols);
    length *= 1.2;
    poisson_am_blend::AdaptiveMultigridParam param;
    param.depth = lround(ceil(log(length) / log(2)));
    param.depth = std::min(12, param.depth);

    param.value_weight = options.w_value;
    param.lap_weigt = options.w_lap;
    param.bilap_weight = options.w_bilap;

    using FEMSigs = IsotropicUIntPack<2, FEMDegreeAndBType<2, BOUNDARY_FREE>::Signature>;

    cv::Mat blend = poisson_am_blend::Execute<float>(points, MatrixXf(), Vector2i(cols, rows), param, FEMSigs());
    return blend;
}

cv::Mat poisson_interpolation_am(const cv::Mat &src, const cv::Mat &dxy, const cv::Mat &mask,
                                 PoissonSolverOptions options) {
    MatrixXf points(3, cv::countNonZero(mask != GUIDED));
    MatrixXf gradients(4, cv::countNonZero(mask == GUIDED));
    int rows = mask.rows;
    int cols = mask.cols;

    int index_value = 0;
    int index_gradient = 0;
    for (int r = 0; r < rows; ++r) {
        const uint8_t *msk_ptr = mask.ptr<uint8_t>(r);
        const float *src_ptr = src.ptr<float>(r);
        const cv::Vec2f *dxy_ptr = dxy.ptr<cv::Vec2f>(r);
        for (int c = 0; c < cols; ++c) {
            if (msk_ptr[c] == GUIDED) {
                // 加入 gradient
                gradients(0, index_gradient) = c;
                gradients(1, index_gradient) = r;
                gradients(2, index_gradient) = dxy_ptr[c][0];
                gradients(3, index_gradient) = dxy_ptr[c][1];
                index_gradient++;

                continue;
            }
            /*if (msk_ptr[c] == BOUNDARY) */ {
                points(0, index_value) = c;
                points(1, index_value) = r;
                points(2, index_value) = src_ptr[c];
                index_value++;
            }
        }
    }

    double length = std::max(rows, cols);
    length *= 1.2;
    poisson_am_blend::AdaptiveMultigridParam param;
    param.depth = lround(ceil(log(length) / log(2)));
    param.depth = std::min(12, param.depth);

    param.value_weight = options.w_value;
    param.gradient_weight = options.w_gradient;
    param.lap_weigt = options.w_lap;
    param.bilap_weight = options.w_bilap;

    using FEMSigs = IsotropicUIntPack<2, FEMDegreeAndBType<2, BOUNDARY_FREE>::Signature>;

    cv::Mat blend = poisson_am_blend::Execute<float>(points, gradients, Vector2i(cols, rows), param, FEMSigs());
    return blend;
}

} // namespace h2o
