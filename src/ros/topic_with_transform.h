#ifndef SLAM_CTOR_ROS_TOPIC_WITH_TRANSFORM_H
#define SLAM_CTOR_ROS_TOPIC_WITH_TRANSFORM_H

#include <string>
#include <vector>
#include <memory>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>
#include <boost/shared_ptr.hpp>
#include <sensor_msgs/LaserScan.h>
#include <rocon_std_msgs/StringArray.h>
#include "std_srvs/Empty.h"

// TODO: make this class inner
template <typename MType>
class TopicObserver { // iface
public: // methods
  virtual void handle_transformed_msg(const boost::shared_ptr<MType>,
                                      const tf::StampedTransform&) = 0;
  virtual void handle_alien_transformed_msg(const boost::shared_ptr<MType>,
                                        const tf::StampedTransform&){}
  virtual void handle_serialize_cb(
                            const boost::shared_ptr<sensor_msgs::LaserScan>,
                            const tf::StampedTransform&){}
  virtual void handle_merging_msg(const std::vector<std::string>& ){}
};

// TODO: add scan drop
template <typename MsgType>
class TopicWithTransform {
  /* NB: wasn't able to implement with TF2 (ROS jade),
         probably because of deadlock
           (https://github.com/ros/geometry2/pull/144)
         Try TF2 with proposed patch.
   */
public: // methods
  // TODO: copy, move ctrs, dtor
  TopicWithTransform(ros::NodeHandle nh,
                     const std::string& topic_name,
                     const std::string& target_frame,
                     const double buffer_duration = 5.0,
                     const uint32_t tf_filter_queue_size = 1000,
                     const uint32_t subscribers_queue_size = 1000,
                     std::string robot_id = ""):
    _target_frame{target_frame},
    _subscr{nh, topic_name, subscribers_queue_size},
    _subscr_alien{nh, "alien_scan", subscribers_queue_size},
    _tf_lsnr{ros::Duration(buffer_duration)},
    _msg_flt{new tf::MessageFilter<MsgType>{
      _subscr, _tf_lsnr, _target_frame, tf_filter_queue_size}},
    _msg_flt_alien{new tf::MessageFilter<MsgType>{
            _subscr_alien, _tf_lsnr, _target_frame, tf_filter_queue_size}} {
      _msg_flt->registerCallback(&TopicWithTransform::transformed_msg_cb,
                                  this);
      _msg_flt_alien->registerCallback(&TopicWithTransform::alien_scan_cb,
                                        this);
      _sub_deserialize = nh.subscribe("/input_serialized_map",5, &TopicWithTransform::deserializator, this);
      _serialize_service = nh.advertiseService("start_serialize" + robot_id, &TopicWithTransform::start_serialize_cb, this);
      ros::spinOnce();
  }

  void subscribe(std::shared_ptr<TopicObserver<MsgType>> obs) {
    _observers.push_back(obs);
  }
private: // methods
  void transformed_msg_cb(const boost::shared_ptr<MsgType> msg) {
    tf::StampedTransform transform;
    std::string msg_frame_id =
      ros::message_traits::FrameId<MsgType>::value(*msg);
    ros::Time msg_time =
      ros::message_traits::TimeStamp<MsgType>::value(*msg);

    try{
      // NB: using last available msg (time = 0) causes issues
      //     on running with higher rate.
      _tf_lsnr.lookupTransform(_target_frame, msg_frame_id,
                               msg_time, transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("Transform retrieval failed. %s",ex.what());
      return;
    }

    for (auto obs : _observers) {
      if (auto obs_ptr = obs.lock()) {
        obs_ptr->handle_transformed_msg(msg, transform);
      }
    }
    curr_scan = msg;
    curr_pose = transform;
  }

//!-------------------------------------------------------------------------!
  void alien_scan_cb(const boost::shared_ptr<MsgType> msg) {
    tf::StampedTransform transform;
        std::string msg_frame_id =
          ros::message_traits::FrameId<MsgType>::value(*msg);
        ros::Time msg_time =
          ros::message_traits::TimeStamp<MsgType>::value(*msg);

        try{
          // NB: using last available msg (time = 0) causes issues
          //     on running with higher rate.
          _tf_lsnr.lookupTransform(_target_frame, msg_frame_id,
                                   msg_time, transform);
        }
        catch (tf::TransformException ex){
          ROS_ERROR("Transform retrieval failed. %s",ex.what());
          return;
        }
    for (auto obs : _observers) {
      if (auto obs_ptr = obs.lock()) {
        obs_ptr->handle_alien_transformed_msg(msg, transform);
      }
    }
  }

  bool start_serialize_cb(std_srvs::Empty::Request  &req,
                          std_srvs::Empty::Response &res){
    for (auto obs : _observers) {
      if (auto obs_ptr = obs.lock()) {
        obs_ptr->handle_serialize_cb(curr_scan, curr_pose);
      }
    }
    return true;
  }

  void deserializator(const rocon_std_msgs::StringArray& serialized_msg) {
    std::cout << serialized_msg.strings.size() << std::endl;

    for (auto obs : _observers) {
      if (auto obs_ptr = obs.lock()) {
        obs_ptr->handle_merging_msg(serialized_msg.strings);
      }
    }
  }

//!-------------------------------------------------------------------------!

private: // fields
  boost::shared_ptr<MsgType> curr_scan;
  tf::StampedTransform curr_pose;
  ros::Subscriber _sub_deserialize;
  ros::ServiceServer _serialize_service;
  std::string _target_frame;
  message_filters::Subscriber<MsgType> _subscr;
  message_filters::Subscriber<MsgType> _subscr_alien;
  tf::TransformListener _tf_lsnr;
  std::unique_ptr<tf::MessageFilter<MsgType>> _msg_flt;
  std::unique_ptr<tf::MessageFilter<MsgType>> _msg_flt_alien;
  std::vector<std::weak_ptr<TopicObserver<MsgType>>> _observers;
};

#endif // macro guard
