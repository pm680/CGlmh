#pragma once
#include "util_openmesh_bind.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <chrono>
#include <omp.h>

namespace USTC_CG
{
class ARAP
{
    public:
    explicit ARAP(
        std::shared_ptr<PolyMesh> mesh,
        pxr::VtArray<pxr::GfVec2f> textureCoords)
         : mesh_(mesh),
           textureCoords_(textureCoords){}
    virtual ~ARAP() noexcept = default;
    
    void local_isometric_parameterization();

    void local_phase();
    void global_phase_decompsition();
    void global_phase_solve();

    pxr::VtArray<pxr::GfVec2f> iteration();

    float cotangent(OpenMesh::SmartHalfedgeHandle he);

    void set_max_iterations(int max_iterations);

    private:
    std::shared_ptr<PolyMesh> mesh_;
    pxr::VtArray<pxr::GfVec2f> textureCoords_;
    size_t NSize;
    size_t TSize;

    std::vector<std::unordered_map<int, Eigen::Vector2f>> vector_x;
    std::vector<Eigen::Matrix2f> vector_L;
    Eigen::MatrixXf uv;

    Eigen::SparseLU<Eigen::SparseMatrix<float>, Eigen::COLAMDOrdering<int>> solver_;

    pxr::VtArray<pxr::GfVec2f> uv_result;

    int max_iterations_ = 100;
};
}
