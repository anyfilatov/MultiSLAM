#ifndef SLAM_CTOR_ROS_OCCUPANCY_GRID_PUBLISHER_H
#define SLAM_CTOR_ROS_OCCUPANCY_GRID_PUBLISHER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <rocon_std_msgs/StringArray.h>
#include "../core/states/state_data.h"
#include "../core/maps/grid_map.h"

#include "../slams/viny_mul/viny_mul_map.h"


template <typename GridMapType>
class OccupancyGridPublisher : public WorldMapObserver<GridMapType> {
public: // method
  OccupancyGridPublisher(ros::Publisher pub, ros::Publisher serialized_pub,
                         const std::string &tf_map_frame_id,
                         double publ_interval_secs = 5.0):
    _map_pub{pub}, _serialized_map_pub{serialized_pub},
    _tf_map_frame_id{tf_map_frame_id},
    _publishing_interval{publ_interval_secs} { }

  void on_map_update(const GridMapType &map) override {
    if ((ros::Time::now() - _last_pub_time) < _publishing_interval) {
      return;
    }

    nav_msgs::OccupancyGrid map_msg;
    map_msg.header.frame_id = _tf_map_frame_id;
    map_msg.info.map_load_time = ros::Time::now();
    map_msg.info.width = map.width();
    map_msg.info.height = map.height();
    map_msg.info.resolution = map.scale();
    // move map to the middle
    nav_msgs::MapMetaData &info = map_msg.info;
    DiscretePoint2D origin = map.origin();
    info.origin.position.x = -info.resolution * origin.x;
    info.origin.position.y = -info.resolution * origin.y;
    info.origin.position.z = 0;
    map_msg.data.reserve(info.height * info.width);
    DiscretePoint2D pnt;
    DiscretePoint2D end_of_map = DiscretePoint2D(info.width,
                                                 info.height) - origin;
    for (pnt.y = -origin.y; pnt.y < end_of_map.y; ++pnt.y) {
      for (pnt.x = -origin.x; pnt.x < end_of_map.x; ++pnt.x) {
        double value = (double)map[pnt];
        int cell_value = value == -1 ? -1 : value * 100;
        map_msg.data.push_back(cell_value);
      }
    }

    _map_pub.publish(map_msg);
    _last_pub_time = ros::Time::now();



  }
  void on_serialize_update(const GridMapType &map) override;
private: // fields
  ros::Publisher _map_pub;
  ros::Publisher _serialized_map_pub;
  std::string _tf_map_frame_id;
  ros::Time _last_pub_time;
  ros::Duration _publishing_interval;
};

template<>
void OccupancyGridPublisher<UnboundedPlainGridMapWithScan>::on_serialize_update(
    const UnboundedPlainGridMapWithScan &map) {
  std::stringstream ss;
  ss << map.get_scan().pose_delta.x << " " << map.get_scan().pose_delta.y
     << " " << map.get_scan().pose_delta.theta << " ";
  ss << map.get_scan().quality << " ";
  ss << map.get_scan().scan.points().size() << " ";
  int i = 0;
  for (auto point: map.get_scan().scan.points()) {
    if (i++ == 450) {
      std::cout << point.range() << " " << point.angle() << " "
          << (point.is_occupied() ? "1" : "0") << " ";
    }
    ss << point.range() << " " << point.angle() << " "
       << (point.is_occupied() ? "1" : "0") << " ";
  }
  std::string scan_str = ss.str();
  std::cout << "p_scan " << map.get_scan().scan.points().size() << std::endl;
  rocon_std_msgs::StringArray serialized_map_msg;

  auto serialized = map.save_map();

  serialized_map_msg.strings.reserve(serialized.capacity() + scan_str.capacity());
  serialized_map_msg.strings.push_back(scan_str);
  serialized_map_msg.strings.insert(serialized_map_msg.strings.end(),
                                      serialized.begin(), serialized.end());
  std::cout << serialized_map_msg.strings.size() << std::endl;
  _serialized_map_pub.publish(serialized_map_msg);
}

template <typename GridMapType>
void OccupancyGridPublisher<GridMapType>::on_serialize_update(const GridMapType &map) {

  rocon_std_msgs::StringArray serialized_map_msg;
  auto serialized = map.save_map();
  serialized_map_msg.strings.assign(serialized.begin(), serialized.end());// = map.save_map();
  std::cout << serialized_map_msg.strings.size() << std::endl;
  _serialized_map_pub.publish(serialized_map_msg);
}

#endif
