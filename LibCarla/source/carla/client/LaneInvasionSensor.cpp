// Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// 压线传感器
// 
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/client/LaneInvasionSensor.h"

#include "carla/Logging.h"
#include "carla/client/Map.h"
#include "carla/client/Vehicle.h"
#include "carla/client/detail/Simulator.h"
#include "carla/geom/Location.h"
#include "carla/geom/Math.h"
#include "carla/sensor/data/LaneInvasionEvent.h"

#include <exception>

namespace carla {
namespace client {

  // ===========================================================================
  // -- 静态局部方法 ------------------------------------------------------------
  // ===========================================================================
// 静态方法，根据给定的偏航角和位置进行旋转计算
  static geom::Location Rotate(float yaw, const geom::Location &location) {
  	// 将偏航角从度转换为弧度
    yaw *= geom::Math::Pi<float>() / 180.0f;
    const float c = std::cos(yaw);
    const float s = std::sin(yaw);
    // 进行旋转计算并返回新的位置
    return {
        c * location.x - s * location.y,
        s * location.x + c * location.y,
        location.z};
  }

  // ===========================================================================
  // -- 压线回调类 LaneInvasionCallback -----------------------------------------
  // ===========================================================================

  class LaneInvasionCallback {
  public:
// 构造函数，接收车辆、地图智能指针和用户回调函数
    LaneInvasionCallback(
        const Vehicle &vehicle,
        SharedPtr<Map> &&map,
        Sensor::CallbackFunctionType &&user_callback)
      : _parent(vehicle.GetId()),
        _parent_bounding_box(vehicle.GetBoundingBox()),
        _map(std::move(map)),
        _callback(std::move(user_callback)) {
      DEBUG_ASSERT(_map != nullptr);
    }
// 处理每一帧数据的函数
    void Tick(const WorldSnapshot &snapshot) const;

  private:
// 内部结构体，用于存储一帧的边界信息
    struct Bounds {
      size_t frame;
      std::array<geom::Location, 4u> corners;
    };
 // 创建边界信息的函数
    std::shared_ptr<const Bounds> MakeBounds(
        size_t frame,
        const geom::Transform &vehicle_transform) const;

    ActorId _parent;
// 存储父类车辆的边界框信息
    geom::BoundingBox _parent_bounding_box;
// 地图的智能指针
    SharedPtr<const Map> _map;
// 用户定义的回调函数
    Sensor::CallbackFunctionType _callback;
// 可变的原子共享指针，用于存储边界信息
    mutable AtomicSharedPtr<const Bounds> _bounds;
  };
// 处理每一帧数据，检查车辆是否压线并调用用户回调函数
  void LaneInvasionCallback::Tick(const WorldSnapshot &snapshot) const {
    // 确保父类还存活。
    auto parent = snapshot.Find(_parent);
    if (!parent) {
      return;
    }
// 创建当前帧的边界信息
    auto next = MakeBounds(snapshot.GetFrame(), parent->transform);
    auto prev = _bounds.load();

    // 第一帧它将为空。
    if ((prev == nullptr) && _bounds.compare_exchange(&prev, next)) {
      return;
    }

    // 确保距离足够长。
    constexpr float distance_threshold = 10.0f * std::numeric_limits<float>::epsilon();
    for (auto i = 0u; i < 4u; ++i) {
      if ((next->corners[i] - prev->corners[i]).Length() < distance_threshold) {
        return;
      }
    }

    // 确保当前帧是最新的。
    do {
      if (prev->frame >= next->frame) {
        return;
      }
    } while (!_bounds.compare_exchange(&prev, next));

    // 最后，可以安全地计算交叉车道。
    std::vector<road::element::LaneMarking> crossed_lanes;
    for (auto i = 0u; i < 4u; ++i) {
      const auto lanes = _map->CalculateCrossedLanes(prev->corners[i], next->corners[i]);
      crossed_lanes.insert(crossed_lanes.end(), lanes.begin(), lanes.end());
    }
// 如果有交叉车道，调用用户回调函数
    if (!crossed_lanes.empty()) {
      _callback(MakeShared<sensor::data::LaneInvasionEvent>(
          snapshot.GetTimestamp().frame,
          snapshot.GetTimestamp().elapsed_seconds,
          parent->transform,
          _parent,
          std::move(crossed_lanes)));
    }
  }
// 创建边界信息的函数实现
  std::shared_ptr<const LaneInvasionCallback::Bounds> LaneInvasionCallback::MakeBounds(
      const size_t frame,
      const geom::Transform &transform) const {
    const auto &box = _parent_bounding_box;
    const auto location = transform.location + box.location;
    const auto yaw = transform.rotation.yaw;
    return std::make_shared<Bounds>(Bounds{frame, {
        location + Rotate(yaw, geom::Location( box.extent.x,  box.extent.y, 0.0f)),
        location + Rotate(yaw, geom::Location(-box.extent.x,  box.extent.y, 0.0f)),
        location + Rotate(yaw, geom::Location( box.extent.x, -box.extent.y, 0.0f)),
        location + Rotate(yaw, geom::Location(-box.extent.x, -box.extent.y, 0.0f))}});
  }

  // ===========================================================================
  // -- 压线传感器 LaneInvasionSensor -------------------------------------------
  // ===========================================================================
 // 压线传感器的析构函数，停止监听
  LaneInvasionSensor::~LaneInvasionSensor() {
    Stop();
  }
// 监听压线事件的函数
void LaneInvasionSensor::Listen(CallbackFunctionType callback) {
    // 尝试将父组件转换为车辆对象
    auto vehicle = boost::dynamic_pointer_cast<Vehicle>(GetParent());
    if (vehicle == nullptr) { // 检查父组件是否为空或者不是车辆类型
        log_error(GetDisplayId(), ": not attached to a vehicle"); // 如果不是车辆类型，记录错误日志
        return; // 提前退出函数
    }

    // 获取当前的模拟环境（Episode）
    auto episode = GetEpisode().Lock(); // 获取一个被锁定的共享指针，用于访问当前模拟的上下文

    // 创建一个 LaneInvasionCallback 对象，用于处理压线事件
    auto cb = std::make_shared<LaneInvasionCallback>(
        *vehicle,                          // 将当前的车辆对象传递给回调对象
        episode->GetCurrentMap(),          // 获取当前模拟环境中的地图对象
        std::move(callback));              // 将用户传递的回调函数绑定到回调对象中

    // 注册一个新的 Tick 事件回调，每一帧都会调用该回调
    const size_t callback_id = episode->RegisterOnTickEvent([cb=std::move(cb)](const auto &snapshot) {
        try {
            cb->Tick(snapshot); // 每帧调用 LaneInvasionCallback 对象的 Tick 方法
        } catch (const std::exception &e) {
            log_error("LaneInvasionSensor:", e.what()); // 如果回调执行过程中发生异常，记录错误日志
        }
    });

    // 更新 _callback_id 成员变量以保存当前注册的回调 ID
    const size_t previous = _callback_id.exchange(callback_id); // 使用线程安全的方式更新回调 ID

    // 如果之前有注册过的回调，则移除旧的回调
    if (previous != 0u) { // 判断是否存在旧的回调
        episode->RemoveOnTickEvent(previous); // 根据回调 ID 移除旧的 Tick 回调
    }
}

// 停止监听压线事件的函数
void LaneInvasionSensor::Stop() {
    // 将 _callback_id 设置为 0，并保存之前的回调 ID
    const size_t previous = _callback_id.exchange(0u); // 将当前回调 ID 重置为 0

    // 尝试获取模拟环境（Episode）
    auto episode = GetEpisode().TryLock(); // 尝试获取一个非阻塞的共享指针，可能返回 nullptr

    // 如果存在之前注册的回调且 Episode 有效，则移除回调
    if ((previous != 0u) && (episode != nullptr)) { // 检查是否有需要移除的回调
        episode->RemoveOnTickEvent(previous); // 根据之前的回调 ID 移除事件监听
    }
}

} // namespace client
} // namespace carla

