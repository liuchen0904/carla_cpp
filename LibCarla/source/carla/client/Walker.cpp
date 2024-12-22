// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/client/Walker.h"

#include "carla/client/detail/Simulator.h"

namespace carla {
    namespace client {

        // 将控制指令应用到 Walker（行走者）对象
        void Walker::ApplyControl(const Control& control) {
            // 检查是否需要更新控制指令
            if (control != _control) { // 如果新的控制指令和当前控制状态不同
                // 调用模拟器的接口将控制应用到 Walker
                GetEpisode().Lock()->ApplyControlToWalker(*this, control);
                // 更新当前控制状态为新的控制指令
                _control = control;
            }
        }

        // 获取 Walker 的当前控制状态
        Walker::Control Walker::GetWalkerControl() const {
            // 锁定当前的 Episode，并从中获取 Walker 的快照
            // 从快照中返回当前 Walker 的控制状态
            return GetEpisode().Lock()->GetActorSnapshot(*this).state.walker_control;
        }

        // 获取 Walker 的骨骼变换信息
        Walker::BoneControlOut Walker::GetBonesTransform() {
            // 调用模拟器接口，获取 Walker 的骨骼变换
            return GetEpisode().Lock()->GetBonesTransform(*this);
        }

        // 设置 Walker 的骨骼变换
        void Walker::SetBonesTransform(const Walker::BoneControlIn& bones) {
            // 调用模拟器接口，将指定的骨骼变换应用到 Walker
            return GetEpisode().Lock()->SetBonesTransform(*this, bones);
        }

        // 混合 Walker 的当前姿势
        void Walker::BlendPose(float blend) {
            // 调用模拟器接口，将指定的混合因子应用到 Walker 的姿势
            return GetEpisode().Lock()->BlendPose(*this, blend);
        }

        // 从动画中更新 Walker 的姿势
        void Walker::GetPoseFromAnimation() {
            // 调用模拟器接口，从动画中获取 Walker 的姿势
            return GetEpisode().Lock()->GetPoseFromAnimation(*this);
        }

    } // namespace client
} // namespace carla
