#ifndef SLAM_CTOR_ROS_LASER_SCAN_OBSERVER_H_INCLUDED
#define SLAM_CTOR_ROS_LASER_SCAN_OBSERVER_H_INCLUDED

#include <utility>
#include <memory>
#include <sensor_msgs/LaserScan.h>
#include <boost/shared_ptr.hpp>
#include <cassert>
#include <iostream>

#include "../core/states/state_data.h"
#include "../core/states/robot_pose.h"
#include "../core/states/world.h"
#include "../core/states/sensor_data.h"
#include "topic_with_transform.h"

class LaserScanObserver : public TopicObserver<sensor_msgs::LaserScan> {
  using ScanPtr = boost::shared_ptr<sensor_msgs::LaserScan>;
  using DstPtr = std::shared_ptr<SensorDataObserver<TransformedLaserScan>>;
public: //methods

  LaserScanObserver(DstPtr slam,
                    bool skip_max_vals,
                    bool use_cached_trig)
    : _slam(slam), _skip_max_vals(skip_max_vals)
    , _use_cached_trig_provider{use_cached_trig} {}

  virtual void handle_transformed_msg(
    const ScanPtr msg, const tf::StampedTransform& t) {

    RobotPose new_pose(t.getOrigin().getX(), t.getOrigin().getY(),
                       tf::getYaw(t.getRotation()));

    TransformedLaserScan transformed_scan;
    transformed_scan.scan.points().reserve(msg->ranges.size());
    transformed_scan.quality = 1.0;
    // TODO: move trig provider setup to the SLAM
    transformed_scan.scan.trig_provider = trig_provider(msg);

    double sp_angle = msg->angle_min - msg->angle_increment;
    double range_max = -1.0;
    double range_min = msg->range_max + 1.0;
    //std::cout << msg->range_max << std::endl; 60 OMG
    for (const auto &range : msg->ranges) {
      bool sp_is_occupied = true;
      double sp_range = range;
      sp_angle += msg->angle_increment;


      // filter points by range/angle
      if (sp_range < msg->range_min) {
        continue;
      } else if (msg->range_max <= sp_range) {
        sp_is_occupied = false;
        sp_range = msg->range_max;
        if (_skip_max_vals) {
          continue;
        }
      }


      if (range_max < sp_range)
        range_max = sp_range;
      if(sp_range < range_min)
        range_min = sp_range;
      transformed_scan.range_max = range_max;

      // add a scan point to a scan
      transformed_scan.scan.points().emplace_back(sp_range, sp_angle,
                                                  sp_is_occupied);

    }
    assert(are_equal(sp_angle, msg->angle_max));

    transformed_scan.pose_delta = new_pose - _prev_pose;
    _prev_pose = new_pose;

    _slam->handle_sensor_data(transformed_scan);
  }

//!----------------------------------------------------------------------!
  virtual void handle_alien_transformed_msg(
    const ScanPtr msg, const tf::StampedTransform& t) {

    RobotPose new_pose(t.getOrigin().getX(), t.getOrigin().getY(),
                       tf::getYaw(t.getRotation()));

    TransformedLaserScan transformed_scan;
    transformed_scan.scan.points().reserve(msg->ranges.size());
    transformed_scan.quality = 1.0;
    // TODO: move trig provider setup to the SLAM
    transformed_scan.scan.trig_provider = trig_provider(msg);

    double sp_angle = msg->angle_min - msg->angle_increment;
    for (const auto &range : msg->ranges) {
      bool sp_is_occupied = true;
      double sp_range = range;
      sp_angle += msg->angle_increment;

      // filter points by range/angle
      if (sp_range < msg->range_min) {
        continue;
      } else if (msg->range_max <= sp_range) {
        sp_is_occupied = false;
        sp_range = msg->range_max;
        if (_skip_max_vals) {
          continue;
        }
      }

      // add a scan point to a scan
      transformed_scan.scan.points().emplace_back(sp_range, sp_angle,
                                                  sp_is_occupied);
    }
    assert(are_equal(sp_angle, msg->angle_max));

    transformed_scan.pose_delta = RobotPose() - RobotPose();

    _slam->handle_alien_sensor_data(transformed_scan);
  }

  void handle_serialize_cb(const ScanPtr scan, const tf::StampedTransform& t){
    std::cout << "handle_serialize_cb" << std::endl;
    RobotPose cur_pose(t.getOrigin().getX(), t.getOrigin().getY(),
                           tf::getYaw(t.getRotation()));
    TransformedLaserScan transformed_scan;
        transformed_scan.scan.points().reserve(scan->ranges.size());
        transformed_scan.quality = 1.0;
        // TODO: move trig provider setup to the SLAM
        transformed_scan.scan.trig_provider = trig_provider(scan);

        double sp_angle = scan->angle_min - scan->angle_increment;
        for (const auto &range : scan->ranges) {
          bool sp_is_occupied = true;
          double sp_range = range;
          sp_angle += scan->angle_increment;

          // filter points by range/angle
          if (sp_range < scan->range_min) {
            continue;
          } else if (scan->range_max <= sp_range) {
            sp_is_occupied = false;
            sp_range = scan->range_max;
            if (_skip_max_vals) {
              continue;
            }
          }

          // add a scan point to a scan
          transformed_scan.scan.points().emplace_back(sp_range, sp_angle,
                                                      sp_is_occupied);
        }
        assert(are_equal(sp_angle, scan->angle_max));

        transformed_scan.pose_delta = cur_pose - RobotPose{0,0,0};
    _slam->handle_serialize_callback(transformed_scan);
  }

  void handle_merging_msg(const std::vector<std::string>& serialized_map){
    _slam->merge_map_str(serialized_map);
  }
//!-----------------------------------------------------------------------!

  const RobotPose &odometry_pose() const { return _prev_pose; }
  void set_odometry_pose(const RobotPose& pose) { _prev_pose = pose; }

private:

  std::shared_ptr<TrigonometryProvider> trig_provider(const ScanPtr msg) {
    if (_use_cached_trig_provider) {
      auto provider = std::make_shared<CachedTrigonometryProvider>();
      provider->update(msg->angle_min, msg->angle_max + msg->angle_increment,
                       msg->angle_increment);
      return provider;
    } else {
      return std::make_shared<RawTrigonometryProvider>();
    }
  }

private: // fields
  DstPtr _slam;
  bool _skip_max_vals;
  bool _use_cached_trig_provider;
  RobotPose _prev_pose;
};

#endif
