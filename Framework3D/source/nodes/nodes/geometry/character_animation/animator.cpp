#include "animator.h"
#include <cassert>

namespace USTC_CG::node_character_animation {

using namespace pxr;

Joint::Joint(int idx, string name, int parent_idx, const GfMatrix4f& bind_transform) 
: idx_(idx), name_(name), parent_idx_(parent_idx), bind_transform_(bind_transform)
{
}

void Joint::compute_world_transform()
{
    // ---------- (HW TODO) Compute world space trasform of this joint -----------------
    if (!parent_) {
        world_transform_ = local_transform_;
    }
    else {
        world_transform_ = local_transform_ * parent_->world_transform_;
    }
    // --------------------------------------------------------------------------------
}

void JointTree::compute_world_transforms_for_each_joint()
{
    // ----------- (HW_TODO) Traverse all joint and compute its world space transform ---
	// Call compute_world_transform for each joint
    if (root_) {
        compute_world_transform_recursive(root_);
    }
    // ---------------------------------------------
}
void JointTree::compute_world_transform_recursive(const shared_ptr<Joint>& joint)
{
    joint->compute_world_transform();

    for (const auto& child : joint->children_) {
        compute_world_transform_recursive(child);
    }
}

void JointTree::add_joint(int idx, std::string name, int parent_idx, const GfMatrix4f& bind_transform)
{
    auto joint = make_shared<Joint>(idx, name, parent_idx, bind_transform);
    joints_.push_back(joint);
    if (parent_idx < 0) {
        root_ = joint;
    }
    else {
        joints_[parent_idx]->children_.push_back(joint);

        if (parent_idx < joints_.size())
            joint->parent_ = joints_[parent_idx];
        else {
            std::cout << "[add_joint_error] parent_idx out of range" << std::endl;
            exit(1);
        }
    }
}

void JointTree::update_joint_local_transform(const VtArray<GfMatrix4f>& new_local_transforms)
{
    assert(new_local_transforms.size() == joints_.size());

    for (int i = 0; i < joints_.size(); ++i) {
		joints_[i]->local_transform_ = new_local_transforms[i];
	}
}

void JointTree::print()
{
	for (auto joint_ptr : joints_) {
		std::cout << "Joint idx: " << joint_ptr->idx_ << " name: " << joint_ptr->name_ << " parent_idx: " << joint_ptr->parent_idx_ << std::endl;
	}
}


Animator::Animator(const shared_ptr<MeshComponent> mesh, const shared_ptr<SkelComponent> skel) 
	: mesh_(mesh),
      skel_(skel)
{
    auto joint_order = skel_->jointOrder;
    auto topology = skel_->topology;
    for (size_t i = 0; i < joint_order.size(); ++i) {
	    SdfPath jointPath(joint_order[i]);

        string joint_name = jointPath.GetName();
        int parent_idx = topology.GetParent(i);

		joint_tree_.add_joint(i, joint_name, parent_idx, GfMatrix4f(skel->bindTransforms[i]));
	}

	joint_tree_.print(); 
}

void Animator::step(const shared_ptr<SkelComponent> skel)
{
	joint_tree_.update_joint_local_transform(skel->localTransforms);

    joint_tree_.compute_world_transforms_for_each_joint(); 

	update_mesh_vertices();
}

void Animator::update_mesh_vertices()
{
    // ----------- (HW_TODO) Update mesh vertices according to the current joint transforms ----
    // 1. get skel_->jointIndices and skel_->jointWeight;
    // 2. For each vertex, compute the new position by transforming the rest position with the joint
    // transforms
    // 2. Update the vertex position in the mesh
    const auto& jointIndices = skel_->jointIndices;
    const auto& jointWeights = skel_->jointWeight;

    size_t numVertices = mesh_->vertices.size();
    size_t numInfluencingJoints = jointIndices.size() / numVertices;

    for (size_t vertexIndex = 0; vertexIndex < numVertices; ++vertexIndex) {
        GfVec3f finalPosition(0, 0, 0);

        GfVec3f initialPosition = mesh_->vertices[vertexIndex];

        for (size_t i = 0; i < numInfluencingJoints; ++i) {
            int jointIndex = jointIndices[numInfluencingJoints * vertexIndex + i];
            float weight = jointWeights[numInfluencingJoints * vertexIndex + i];

            auto joint = joint_tree_.get_joint(jointIndex);

            if (joint) {
                GfMatrix4f worldTransform = joint->get_world_transform();

                GfMatrix4f bindTransform = joint->get_bind_transform();

                GfMatrix4f inverseBindTransform = bindTransform.GetInverse();

                GfVec3f transformedPosition = worldTransform.TransformAffine(
                    inverseBindTransform.TransformAffine(initialPosition));

                finalPosition += transformedPosition * weight;
            }
        }

        mesh_->vertices[vertexIndex] = finalPosition;
    }
    // --------------------------------------------------------------------------------
}

}  // namespace USTC_CG::node_character_animation