#include "simulation/kinematics.h"

#include "Eigen/Dense"
#include <iostream>
#include "acclaim/bone.h"
#include "util/helper.h"
#include <queue>

namespace kinematics {

void forwardSolver(const acclaim::Posture& posture, acclaim::Bone* root) {
    // TODO (FK)
    // Same as HW2
    // Hint:
    //   1. If you don't use `axis` in this function, you can copy-paste your code

    root->start_position = posture.bone_translations[root->idx];
    root->end_position = posture.bone_translations[root->idx];
    Eigen::Vector4d euler = posture.bone_rotations[root->idx];
    Eigen::Quaterniond quat = util::rotateDegreeZYX(euler.x(), euler.y(), euler.z());
    root->rotation.linear() = quat.normalized().toRotationMatrix();

    std::queue<acclaim::Bone*> q;
    q.push(root);

    while (!q.empty()) {
        acclaim::Bone* bone = q.front();
        q.pop();
        if (bone != root) {
            bone->start_position = bone->parent->end_position;
            euler = posture.bone_rotations[bone->idx];
            quat = util::rotateDegreeZYX(euler.x(), euler.y(), euler.z());
            bone->rotation.linear() = quat.normalized().toRotationMatrix();
            bone->rotation = bone->parent->rotation * bone->rot_parent_current * bone->rotation;
            Eigen::Vector4d V = bone->dir * bone->length + posture.bone_translations[bone->idx];
            bone->end_position = bone->rotation * V + bone->start_position;
        }
        bone = bone->child;
        while (bone) {
            q.push(bone);
            bone = bone->sibling;
        }
    }
}

Eigen::VectorXd pseudoInverseLinearSolver(const Eigen::Matrix4Xd& Jacobian, const Eigen::Vector4d& target) {
    // TODO (find x which min(| jacobian * x - target |))
    // Hint:
    //   1. Linear algebra - least squares solution
    //   2. https://en.wikipedia.org/wiki/Moore%E2%80%93Penrose_inverse#Construction
    // Note:
    //   1. SVD or other pseudo-inverse method is useful
    //   2. Some of them have some limitation, if you use that method you should check it.

    return Jacobian.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(target);
}

/**
 * @brief Perform inverse kinematics (IK)
 *
 * @param target_pos The position where `end_bone` will move to.
 * @param start_bone This bone is the last bone you can move while doing IK
 * @param end_bone This bone will try to reach `target_pos`
 * @param posture The original AMC motion's reference, you need to modify this
 *
 * @return True if IK is stable (HW3 bonus)
 */
bool inverseJacobianIKSolver(const Eigen::Vector4d& target_pos, acclaim::Bone* start_bone, acclaim::Bone* end_bone,
                             acclaim::Posture& posture) {
    constexpr int max_iteration = 1000;
    constexpr double epsilon = 1E-3;
    constexpr double step = 0.1;
    // Since bone stores in bones[i] that i == bone->idx, we can use bone - bone->idx to find bones[0] which is the root.
    acclaim::Bone* root_bone = start_bone - start_bone->idx;
    // TODO
    // Perform inverse kinematics (IK)
    // HINTs will tell you what should do in that area.
    // Of course you can ignore it (Any code below this line) and write your own code.
    acclaim::Posture original_posture(posture);

    size_t bone_num = 0;
    std::vector<acclaim::Bone*> boneList;
    // TODO
    // Calculate number of bones need to move to perform IK, store in `bone_num` 
    // a.k.a. how may bones from end_bone to its parent then to start_bone (include both start_bone and end_bone)
    // Store the bones need to move to perform IK into boneList
    // Hint:
    //   1. Traverse from end_bone to start_bone is easier than start to end (since there is only 1 parent)
    //   2. If start bone is not reachable from end. Go to root first.
    // Note:
    //   1. Both start_bone and end_bone should be in the list
    acclaim::Bone* cur = end_bone;
    while (cur) {
        boneList.push_back(cur);
        if (cur == root_bone) {
            cur = start_bone;
            while (cur != root_bone) {
                boneList.push_back(cur);
                cur = cur->parent;
            }
        } else if (cur == start_bone) {
            break; 
        }
        cur = cur->parent;
    }
    bone_num = boneList.size();

    Eigen::Matrix4Xd Jacobian(4, 3 * bone_num);
    Jacobian.setZero();
    for (int iter = 0; iter < max_iteration; ++iter) {
        forwardSolver(posture, root_bone);
        Eigen::Vector4d desiredVector = target_pos - end_bone->end_position;
        if (desiredVector.norm() < epsilon) {
            break;
        }
        // TODO (compute jacobian)
        //   1. Compute arm vectors
        //   2. Compute jacobian columns, store in `Jacobian`
        // Hint:
        //   1. You should not put rotation in jacobian if it doesn't have that DoF.
        //   2. jacobian.col(/* some column index */) = /* jacobian column */
        for (long long i = 0; i < bone_num; i++) {
            Eigen::Vector4d V = end_bone->end_position - boneList[i]->start_position;

            Eigen::Vector3d v = Eigen::Vector3d(V[0], V[1], V[2]);
            Eigen::Affine3d R = boneList[i]->rotation;
            if (boneList[i]->dofrx) {
                Eigen::Vector3d a = R.linear().col(0).normalized();
                Eigen::Vector3d dp = a.cross(v);
                Jacobian.col(i * 3) = Eigen::Vector4d(dp[0], dp[1], dp[2], 0);
            }
            if (boneList[i]->dofry) {
                Eigen::Vector3d a = R.linear().col(1).normalized();
                Eigen::Vector3d dp = a.cross(v);
                Jacobian.col(i * 3 + 1) = Eigen::Vector4d(dp[0], dp[1], dp[2], 0);
            }
            if (boneList[i]->dofrz) {
                Eigen::Vector3d a = R.linear().col(2).normalized();
                Eigen::Vector3d dp = a.cross(v);
                Jacobian.col(i * 3 + 2) = Eigen::Vector4d(dp[0], dp[1], dp[2], 0);
            }
        }

        Eigen::VectorXd deltatheta = step * pseudoInverseLinearSolver(Jacobian, desiredVector);

        // TODO (update rotation)
        //   Update `posture.bone_rotation` (in euler angle / degrees) using deltaTheta
        // Hint:
        //   1. You can ignore rotation limit of the bone.
        // Bonus:
        //   1. You cannot ignore rotation limit of the bone.
        for (long long i = 0; i < bone_num; i++) {
            float x = deltatheta(i * 3);
            float y = deltatheta(i * 3 + 1);
            float z = deltatheta(i * 3 + 2);
            Eigen::Vector4d rotation = posture.bone_rotations[boneList[i]->idx] + util::toDegree(Eigen::Vector4d(x, y, z, 0));
            rotation[0] = std::clamp((float)rotation[0], boneList[i]->rxmin, boneList[i]->rxmax);
            rotation[1] = std::clamp((float)rotation[1], boneList[i]->rymin, boneList[i]->rymax);
            rotation[2] = std::clamp((float)rotation[2], boneList[i]->rzmin, boneList[i]->rzmax);
            posture.bone_rotations[boneList[i]->idx] = rotation;
        }

    }
    // TODO (Bonus)
    // Return whether IK is stable (i.e. whether the ball is reachable) and let the skeleton not swing its hand in the air
    Eigen::Vector4d desiredVector = target_pos - end_bone->end_position;
    if (desiredVector.norm() < epsilon) {
        return true;
    }
    posture = acclaim::Posture(original_posture);
    forwardSolver(posture, root_bone);
    return false;
}
}  // namespace kinematics
