#pragma once 
#include "util_openmesh_bind.h"
#include <Eigen/Sparse>
#include <iostream>
#include <iomanip>

namespace USTC_CG 
{
class MinSurfCal
{
    public:
    enum WeightType
    {
        kUniform = 1,
        kCotangent = 2,
    };
    explicit MinSurfCal(
        std::shared_ptr<USTC_CG::PolyMesh> original_mesh,
        std::shared_ptr<USTC_CG::PolyMesh> mesh)
         : original_mesh_(original_mesh),
           mesh_(mesh){}
    virtual ~MinSurfCal() noexcept = default;

    void set_uniform();
    void set_cotangent();

    void decompsition();
    Eigen::MatrixXf solve();
    float weight(OpenMesh::VertexHandle vh_i, OpenMesh::SmartHalfedgeHandle he_j)  const;

    private:
    std::shared_ptr<PolyMesh> original_mesh_;
    std::shared_ptr<PolyMesh> mesh_;
    size_t NSize;
    size_t TSize;

    WeightType weight_type_;

    Eigen::SparseLU<Eigen::SparseMatrix<float>, Eigen::COLAMDOrdering<int>> solver_;
};    
}