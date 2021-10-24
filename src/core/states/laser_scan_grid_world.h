#ifndef SLAM_CTOR_CORE_LASER_SCAN_GRID_WORLD_H
#define SLAM_CTOR_CORE_LASER_SCAN_GRID_WORLD_H

#include "sensor_data.h"
#include "world.h"

#include "../../slams/viny_mul/viny_mul_map.h"

template <typename Map>
class LaserScanGridWorld : public World<TransformedLaserScan, Map> {
public: //types
  using MapType = typename World<TransformedLaserScan, Map>::MapType;
  using ScanType = TransformedLaserScan;
public: // methods

  void handle_sensor_data(ScanType &scan) override {
    this->update_robot_pose(scan.pose_delta);
    handle_observation(scan);

    this->notify_with_pose(this->pose());
    this->notify_with_map(this->map());
  }

  void handle_alien_sensor_data(ScanType &scan) override {
      //this->update_robot_pose(scan.pose_delta);
      handle_alien_observation(scan);

      this->notify_with_pose(this->pose());
      this->notify_with_map(this->map());
    }

  void handle_serialize_callback(ScanType& scan) override;

  void merge_map_str(const std::vector<std::string> &serialized_map) {
    merge_alien_map(serialized_map);
  }
  virtual void handle_observation(ScanType &tr_scan) = 0;
  virtual void handle_alien_observation(ScanType &tr_scan) {}
  virtual void merge_alien_map(const std::vector<std::string> &) {
  }
};

template <>
void LaserScanGridWorld<UnboundedPlainGridMapWithScan>::
       handle_serialize_callback(ScanType& scan) {
  std::cout << "handle_serialize_callback" << std::endl;
  this->map().set_scan(scan);
  this->notify_with_serialize(this->map());
}

template <typename Map>
void LaserScanGridWorld<Map>::handle_serialize_callback(ScanType& scan) {
  this->notify_with_serialize(this->map());
}

#endif
