#ifndef SLAM_CTOR_CORE_WORLD_H_INCLUDED
#define SLAM_CTOR_CORE_WORLD_H_INCLUDED

#include <memory>

#include "robot_pose.h"

template <typename SensorData>
class SensorDataObserver {
public:
  virtual void handle_sensor_data(SensorData &) = 0;
  virtual void handle_alien_sensor_data(SensorData &) {}
  virtual void handle_serialize_callback(SensorData &) {}
  virtual void merge_map_str(const std::vector<std::string> &) {}
  // No virtual dtor, since a descendant is not suppoused to be
  // destroyed via a pointer to the class
};

class WorldPoseObserver {
public:
  virtual void on_pose_update(const RobotPose &rs) = 0;
  // No virtual dtor, since a descendant is not suppoused to be
  // destroyed via a pointer to the class
};

template<typename MapType>
class WorldMapObserver {
public:
  virtual void on_map_update(const MapType &map) = 0;
  virtual void on_serialize_update(const MapType &serialize) {}
  // No virtual dtor, since a descendant is not suppoused to be
  // destroyed via a pointer to the class
};

template<typename MapType>
class WorldObservable {
public:
  void subscribe_map(std::shared_ptr<WorldMapObserver<MapType>> obs) {
    _world_map_observers.push_back(obs);
    _world_serialize_observers.push_back(obs);
  }

  void subscribe_pose(std::shared_ptr<WorldPoseObserver> obs) {
    _world_pose_observers.push_back(obs);
  }

protected: // methods

#define NOTIFY_EACH_WOBSERVER(var)                     \
  for (auto &raw_obs : _world_##var##_observers) {     \
    auto obs_ptr = raw_obs.lock();                     \
    if (obs_ptr) {                                     \
      obs_ptr->on_##var##_update(var);                 \
    }                                                  \
  }

  void notify_with_pose(const RobotPose &pose) {
    NOTIFY_EACH_WOBSERVER(pose);
  }

  void notify_with_map(const MapType &map) {
    NOTIFY_EACH_WOBSERVER(map);
  }

  void notify_with_serialize(const MapType &serialize) {

    NOTIFY_EACH_WOBSERVER(serialize);
  }

#undef NOTIFY_EACH_WOBSERVER

private:
  std::vector<std::weak_ptr<WorldMapObserver<MapType>>> _world_map_observers;
  std::vector<std::weak_ptr<WorldMapObserver<MapType>>> _world_serialize_observers;
  std::vector<std::weak_ptr<WorldPoseObserver>> _world_pose_observers;
};

// TODO: try to simplify template params
template <typename ObservationType, typename MapT>
class World : public WorldObservable<MapT>
            , public SensorDataObserver<ObservationType> {
public: // type aliases
  using MapType = MapT;
public:
  // data-in
  virtual void update_robot_pose(const RobotPoseDelta& delta) {
    _pose += delta;
  }

  // data-out
  virtual const World<ObservationType, MapType>& world() const { return *this; }
  virtual const RobotPose& pose() const { return _pose; }
  virtual const MapType& map() const = 0;

  MapType& map() {
    return const_cast<MapType&>(
      // TODO: try to use decltype
      static_cast<const World<ObservationType, MapT>*>(this)->map()
    );
  }
protected:
  virtual void handle_observation(ObservationType&) = 0;
  virtual ~World() = default;
private:
  RobotPose _pose;
};

#endif
