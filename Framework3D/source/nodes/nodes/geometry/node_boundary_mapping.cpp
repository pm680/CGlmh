#include "GCore/Components/MeshOperand.h"
#include "Nodes/node.hpp"
#include "Nodes/node_declare.hpp"
#include "Nodes/node_register.h"
#include "geom_node_base.h"
#include "utils/util_openmesh_bind.h"

/*
** @brief HW4_TutteParameterization
**
** This file contains two nodes whose primary function is to map the boundary of a mesh to a plain
** convex closed curve (circle of square), setting the stage for subsequent Laplacian equation
** solution and mesh parameterization tasks.
**
** Key to this node's implementation is the adept manipulation of half-edge data structures
** to identify and modify the boundary of the mesh.
**
** Task Overview:
** - The two execution functions (node_map_boundary_to_square_exec,
** node_map_boundary_to_circle_exec) require an update to accurately map the mesh boundary to a and
** circles. This entails identifying the boundary edges, evenly distributing boundary vertices along
** the square's perimeter, and ensuring the internal vertices' positions remain unchanged.
** - A focus on half-edge data structures to efficiently traverse and modify mesh boundaries.
*/

namespace USTC_CG::node_boundary_mapping {

/*
** HW4_TODO: Node to map the mesh boundary to a circle.
*/

static void node_map_boundary_to_circle_declare(NodeDeclarationBuilder& b)
{
    // Input-1: Original 3D mesh with boundary
    b.add_input<decl::Geometry>("Input");

    // Output-1: Processed 3D mesh whose boundary is mapped to a square and the interior vertices
    // remains the same
    b.add_output<decl::Geometry>("Output");
}

static void node_map_boundary_to_circle_exec(ExeParams params)
{
    // Get the input from params
    auto input = params.get_input<GOperandBase>("Input");

    // (TO BE UPDATED) Avoid processing the node when there is no input
    if (!input.get_component<MeshComponent>()) {
        throw std::runtime_error("Boundary Mapping: Need Geometry Input.");
    }

    /* ----------------------------- Preprocess -------------------------------
    ** Create a halfedge structure (using OpenMesh) for the input mesh. The
    ** half-edge data structure is a widely used data structure in geometric
    ** processing, offering convenient operations for traversing and modifying
    ** mesh elements.
    */
    auto halfedge_mesh = operand_to_openmesh(&input);

    /* ----------- [HW4_TODO] TASK 2.1: Boundary Mapping (to circle) ------------
    ** In this task, you are required to map the boundary of the mesh to a circle
    ** shape while ensuring the internal vertices remain unaffected. This step is
    ** crucial for setting up the mesh for subsequent parameterization tasks.
    **
    ** Algorithm Pseudocode for Boundary Mapping to Circle
    ** ------------------------------------------------------------------------
    ** 1. Identify the boundary loop(s) of the mesh using the half-edge structure.
    **
    ** 2. Calculate the total length of the boundary loop to determine the spacing
    **    between vertices when mapped to a square.
    **
    ** 3. Sequentially assign each boundary vertex a new position along the square's
    **    perimeter, maintaining the calculated spacing to ensure proper distribution.
    **
    ** 4. Keep the interior vertices' positions unchanged during this process.
    **
    ** Note: How to distribute the points on the circle?
    **
    ** Note: It would be better to normalize the boundary to a unit circle in [0,1]x[0,1] for
    ** texture mapping.
    */
    std::vector<float> length;
    float TotalLength = 0.f;
    float radius = 1.f;
    for(const auto he_start : halfedge_mesh->halfedges())
    {
        if(halfedge_mesh->is_boundary(he_start))
        {
            auto he = he_start;
            do
            {
                length.push_back((halfedge_mesh->point(he.to()) - 
                                        halfedge_mesh->point(he.from())).length() / (2 * radius));
                TotalLength += length.back();
                he = he.next();
            } while(he != he_start);
            break;
        }
    }

    float r;
    float angle = 0.f;
    OpenMesh::Vec3f start_pos;
    OpenMesh::Vec3f new_pos;
    for(const auto he_start : halfedge_mesh->halfedges())
    {
        if(halfedge_mesh->is_boundary(he_start))
        {
            r = sqrtf(pow(halfedge_mesh->point(he_start.from())[0], 2)
                     + pow(halfedge_mesh->point(he_start.from())[1], 2));
            start_pos[0] = radius * halfedge_mesh->point(he_start.from())[0] / r;
            start_pos[1] = radius * halfedge_mesh->point(he_start.from())[1] / r;
            start_pos[2] = halfedge_mesh->point(he_start.from())[2];
            halfedge_mesh->set_point(he_start.from(), start_pos);
            auto he = he_start;
            auto length_it = length.begin();
            do
            {
                angle = 2.0 * M_PI * *length_it / TotalLength; 
                new_pos[0] = halfedge_mesh->point(he.from())[0] * cosf(angle)
                            - halfedge_mesh->point(he.from())[1] * sinf(angle);
                new_pos[1] = halfedge_mesh->point(he.from())[1] * cosf(angle)
                            + halfedge_mesh->point(he.from())[0] * sinf(angle);
                new_pos[2] = start_pos[2];
                halfedge_mesh->set_point(he.to(), new_pos);
                ++length_it;
                he = he.next();
            } while(he != he_start);
            break;
        }
    }

    /* ----------------------------- Postprocess ------------------------------
    ** Convert the result mesh from the halfedge structure back to GOperandBase format as the node's
    ** output.
    */
    auto operand_base = openmesh_to_operand(halfedge_mesh.get());

    auto& output = input;
    output.get_component<MeshComponent>()->vertices =
        operand_base->get_component<MeshComponent>()->vertices;

    // Set the output of the nodes
    params.set_output("Output", std::move(output));
}

/*
** HW4_TODO: Node to map the mesh boundary to a square.
*/

static void node_map_boundary_to_square_declare(NodeDeclarationBuilder& b)
{
    // Input-1: Original 3D mesh with boundary
    b.add_input<decl::Geometry>("Input");

    // Output-1: Processed 3D mesh whose boundary is mapped to a square and the interior vertices
    // remains the same
    b.add_output<decl::Geometry>("Output");
}

static void node_map_boundary_to_square_exec(ExeParams params)
{
    // Get the input from params
    auto input = params.get_input<GOperandBase>("Input");

    // (TO BE UPDATED) Avoid processing the node when there is no input
    if (!input.get_component<MeshComponent>()) {
        throw std::runtime_error("Input does not contain a mesh");
    }

    /* ----------------------------- Preprocess -------------------------------
    ** Create a halfedge structure (using OpenMesh) for the input mesh.
    */
    auto halfedge_mesh = operand_to_openmesh(&input);

    /* ----------- [HW4_TODO] TASK 2.2: Boundary Mapping (to square) ------------
    ** In this task, you are required to map the boundary of the mesh to a circle
    ** shape while ensuring the internal vertices remain unaffected.
    **
    ** Algorithm Pseudocode for Boundary Mapping to Square
    ** ------------------------------------------------------------------------
    ** (omitted)
    **
    ** Note: Can you perserve the 4 corners of the square after boundary mapping?
    **
    ** Note: It would be better to normalize the boundary to a unit circle in [0,1]x[0,1] for
    ** texture mapping.
    */
    std::vector<float> length;
    float TotalLength = 0.f;
    for(const auto he_start : halfedge_mesh->halfedges())
    {
        if(halfedge_mesh->is_boundary(he_start))
        {
            auto he = he_start;
            do
            {
                length.push_back(
                    (halfedge_mesh->point(he.to()) - halfedge_mesh->point(he.from())).length());
                TotalLength += length.back();
                he = he.next();
            } while(he != he_start);
            break;
        }
    }

    float r_former = 0.f, r_letter = 0.f;
    float a = 1.f;
    OpenMesh::Vec3f new_pos;
    for(const auto he_start : halfedge_mesh->halfedges())
    {
        if(halfedge_mesh->is_boundary(he_start))
        {
            auto he = he_start;
            auto length_it = length.begin();
            do
            {
                switch (static_cast<int>(4 * r_letter))
                { 
                case 0:
                    new_pos[0] = 4 * r_letter * a;
                    new_pos[1] = 0;
                    new_pos[2] = halfedge_mesh->point(he_start.from())[2];
                    break;
                case 1:  
                    if(r_former < 0.25)
                    {
                        new_pos[0] = a;
                        new_pos[1] = 0;
                        new_pos[2] = halfedge_mesh->point(he_start.from())[2];
                        break;
                    }
                    new_pos[0] = a;
                    new_pos[1] = (4 * r_letter - 1) * a;
                    new_pos[2] = halfedge_mesh->point(he_start.from())[2];
                    break;
                case 2:
                    if(r_former < 0.50)
                    {
                        new_pos[0] = a;
                        new_pos[1] = a;
                        new_pos[2] = halfedge_mesh->point(he_start.from())[2];
                        break;
                    }
                    new_pos[0] = (3 - 4 * r_letter) * a;
                    new_pos[1] = a;
                    new_pos[2] = halfedge_mesh->point(he_start.from())[2];
                    break;
                case 3:
                    if(r_former < 0.75)
                    {
                        new_pos[0] = 0;
                        new_pos[1] = a;
                        new_pos[2] = halfedge_mesh->point(he_start.from())[2];
                        break;
                    }
                    new_pos[0] = 0;
                    new_pos[1] = (4 - 4 * r_letter) * a;
                    new_pos[2] = halfedge_mesh->point(he_start.from())[2];
                    break;
                default: break;
                }
                if(r_letter > 0) r_former += *length_it / TotalLength;
                r_letter += *length_it / TotalLength;
                halfedge_mesh->set_point(he.to(), new_pos);
                he = he.next();
            } while(he != he_start);
            break;
        }
    }

    /* ----------------------------- Postprocess ------------------------------
    ** Convert the result mesh from the halfedge structure back to GOperandBase format as the node's
    ** output.
    */
    auto operand_base = openmesh_to_operand(halfedge_mesh.get());

    // Set the output of the nodes
    params.set_output("Output", std::move(*operand_base));
}

static void node_register()
{
    static NodeTypeInfo ntype_square, ntype_circle;

    strcpy(ntype_square.ui_name, "Map Boundary to Square");
    strcpy_s(ntype_square.id_name, "geom_map_boundary_to_square");

    geo_node_type_base(&ntype_square);
    ntype_square.node_execute = node_map_boundary_to_square_exec;
    ntype_square.declare = node_map_boundary_to_square_declare;
    nodeRegisterType(&ntype_square);

    strcpy(ntype_circle.ui_name, "Map Boundary to Circle");
    strcpy_s(ntype_circle.id_name, "geom_map_boundary_to_circle");

    geo_node_type_base(&ntype_circle);
    ntype_circle.node_execute = node_map_boundary_to_circle_exec;
    ntype_circle.declare = node_map_boundary_to_circle_declare;
    nodeRegisterType(&ntype_circle);
}

NOD_REGISTER_NODE(node_register)
}  // namespace USTC_CG::node_boundary_mapping
