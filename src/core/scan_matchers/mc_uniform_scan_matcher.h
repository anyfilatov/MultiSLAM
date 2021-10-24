#ifndef SLAM_CTOR_CORE_MC_UNIFORM_SCAN_MATCHER_H
#define SLAM_CTOR_CORE_MC_UNIFORM_SCAN_MATCHER_H

#include <vector>

#include "../random_utils.h"
#include "pose_enumeration_scan_matcher.h"

class UniformPoseEnumerator : public PoseEnumerator {
private:
  struct WeightedRobotPose{
    RobotPose pose;
    double weight;
    WeightedRobotPose (RobotPose rp, double w) {pose = rp; weight = w;}
  };
  static bool pose_is_better(WeightedRobotPose a, WeightedRobotPose b) {
    return a.weight > b.weight;
  }
public:
  using Engine = std::mt19937;
  UniformPoseEnumerator(unsigned seed,
                        double translation_dist  /*meter*/,
                        double rotation_dist    /*radian*/,
                        unsigned samples_amount)
    : _seed(seed)
    , _probable_poses_nm(samples_amount)
    , _max_poses_amount(samples_amount)
    , _pose_shift_rv {UniformRV1D<Engine>{-translation_dist, translation_dist},
                      UniformRV1D<Engine>{-translation_dist, translation_dist},
                      UniformRV1D<Engine>{-rotation_dist, rotation_dist}}
    , _generator(seed) {
    _poses.reserve(samples_amount);
    reset();
  }

  bool has_next() const override {
    return _pose_nm < _probable_poses_nm;
  }

  RobotPose next(const RobotPose &center) override {
    assert(!_poses.empty() && "Don't forger to prepare_samples");
    return _poses[_pose_nm].pose;
  }

  void reset() override {
    _pose_nm = 0;
  }

  void feedback(bool pose_is_acceptabe, double prob) override {
    set_weight(prob);
    _pose_nm++;
  }

  void prepare_samples(RobotPose center) {
    _poses.clear();
    for (int i = 0; i < _max_poses_amount; i++) {
      _poses.emplace_back(center + _pose_shift_rv.sample(_generator), 0.0);
    }
  }

  void prepare_accurate_step(unsigned considered_samples_amount) {
    _pose_nm = 0;
    if (_max_poses_amount < considered_samples_amount)
      considered_samples_amount = _max_poses_amount;
    _probable_poses_nm = considered_samples_amount;
    sort_poses();
  }
private:
  void set_weight(double weight) {
    _poses[_pose_nm].weight = weight;
  }
  void sort_poses(){
    std::sort(_poses.begin(), _poses.end(), UniformPoseEnumerator::pose_is_better);
  }
private:
  //fields
  std::vector<WeightedRobotPose> _poses;

  unsigned _seed;
  unsigned _pose_nm, _probable_poses_nm, _max_poses_amount;
  RobotPoseDeltaRV<Engine> _pose_shift_rv;
  Engine _generator;
};

class MCUniformScanMatcher : public PoseEnumerationScanMatcher {
public:
  MCUniformScanMatcher(std::shared_ptr<ScanProbabilityEstimator> estimator,
                       unsigned seed,
                       double translation_dist,
                       double rotation_dist,
                       unsigned attempts_amount)
    : PoseEnumerationScanMatcher{
        estimator,
        std::make_shared<UniformPoseEnumerator>(
            seed, translation_dist, rotation_dist, attempts_amount
        )
      } {}

  double process_scan(const TransformedLaserScan &raw_scan,
      const RobotPose &init_pose,
      const GridMap &map,
      RobotPoseDelta &pose_delta) override {
    //auto pe = static_cast<UniformPoseEnumerator>(*pose_enumerator());
    std::shared_ptr<UniformPoseEnumerator> pe = std::dynamic_pointer_cast<UniformPoseEnumerator> (pose_enumerator());
    pe->prepare_samples(init_pose);
    auto tmp_scan = raw_scan;
    tmp_scan.scan.points().clear();
    int i = 0;
    for(auto point : raw_scan.scan.points()){
      if (i%10 == 0)
        tmp_scan.scan.points().push_back(point);
      i++;
    }
    PoseEnumerationScanMatcher::process_scan(tmp_scan, init_pose, map, pose_delta);
    pe->prepare_accurate_step(1000);
    PoseEnumerationScanMatcher::process_scan(raw_scan, init_pose, map, pose_delta);
    pe->prepare_accurate_step(1);
    RobotPose best_pose = pe->next(init_pose);
    pose_delta = best_pose - init_pose;

  }
};
#endif
