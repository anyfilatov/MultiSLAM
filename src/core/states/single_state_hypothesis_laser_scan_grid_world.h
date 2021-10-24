#ifndef SLAM_CTOR_CORE_SINGLE_STATE_HYPOTHESIS_LASER_SCAN_GRID_WORLD_H
#define SLAM_CTOR_CORE_SINGLE_STATE_HYPOTHESIS_LASER_SCAN_GRID_WORLD_H

#include <memory>
#include <utility>

#include "../maps/grid_map.h"
#include "../maps/grid_map_scan_adders.h"
#include "../scan_matchers/grid_scan_matcher.h"
#include "../scan_matchers/bf_multi_res_scan_matcher.h"
#include "../scan_matchers/mc_uniform_scan_matcher.h"
#include <chrono>
#include <ctime>


#include "laser_scan_grid_world.h"

struct SingleStateHypothesisLSGWProperties {
  double localized_scan_quality = 1.0;
  double raw_scan_quality = 1.0;
  std::size_t scan_margin = 0;

  std::shared_ptr<GridCell> cell_prototype;
  std::shared_ptr<GridScanMatcher> gsm;
  std::shared_ptr<GridMapScanAdder> gmsa;
  GridMapParams map_props;
};

template <typename MapT>
class SingleStateHypothesisLaserScanGridWorld
  : public LaserScanGridWorld<MapT> {
public:
  using MapType = typename LaserScanGridWorld<MapT>::MapType;
  using Properties = SingleStateHypothesisLSGWProperties;
public:
  SingleStateHypothesisLaserScanGridWorld(const Properties &props)
    : _props{props}
    , _map{_props.cell_prototype->clone(), _props.map_props} {}

  // scan matcher access
  auto scan_matcher() { return _props.gsm; }

  void add_sm_observer(std::shared_ptr<GridScanMatcherObserver> obs) {
    auto sm = scan_matcher();
    if (sm) { sm->subscribe(obs); }
  }

  void remove_sm_observer(std::shared_ptr<GridScanMatcherObserver> obs) {
    auto sm = scan_matcher();
    if (sm) { sm->unsubscribe(obs); }
  }

  // scan adder access
  auto scan_adder() { return _props.gmsa; }

  // state access
  const MapType& map() const override { return _map; }
  using LaserScanGridWorld<MapT>::map; // enable non-const map access

  // TODO: return scan prob
  virtual void handle_observation(TransformedLaserScan &tr_scan) {
    auto sm = scan_matcher();
    sm->reset_state();

    auto pose_delta = RobotPoseDelta{};
    auto score = sm->process_scan(tr_scan, this->pose(), this->map(), pose_delta);
    this->update_robot_pose(pose_delta);

    if (score < 0)
      return;
    tr_scan.quality = pose_delta ? _props.localized_scan_quality
                                 : _props.raw_scan_quality;

    scan_adder()->append_scan(_map, this->pose(), tr_scan.scan,
                              tr_scan.quality, _props.scan_margin);
  }

  virtual void handle_alien_observation(TransformedLaserScan &tr_scan) {

    auto msm = BruteForceMultiResolutionScanMatcher(std::make_shared<WeightedMeanPointProbabilitySPE>(std::make_shared<MaxOccupancyObservationPE>(),
        std::make_shared<EvenSPW>()), deg2rad(60), 0.5);
    msm.reset_state();
    auto pose_delta = RobotPoseDelta{};
    msm.set_lookup_ranges(7.0, 7.0, M_PI);

    RobotPose found_pose(0,0,0);
    //std::cout << this->pose().x << " " << this->pose().y << " " << this->pose().theta << std::endl;

    std::chrono::time_point<std::chrono::system_clock> t1 = std::chrono::system_clock::now();
    msm.process_scan(tr_scan, found_pose, this->map(), pose_delta);
    found_pose += pose_delta;
    std::cout << "Coarse step: " << found_pose.x << " " << found_pose.y << " " << found_pose.theta << std::endl;
    std::chrono::time_point<std::chrono::system_clock> t2 = std::chrono::system_clock::now();
    std::cout << "Time: " << std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() << std::endl;

    /*msm.set_target_accuracy(deg2rad(10), 0.1);
    msm.set_lookup_ranges(0.5, 0.5, deg2rad(60));
    msm.process_scan(tr_scan, found_pose, this->map(), pose_delta);
    found_pose += pose_delta;
    t2 = std::chrono::system_clock::now();
    std::cout << "Middle step: " << found_pose.x << " " << found_pose.y << " " << found_pose.theta << std::endl;
    std::cout << "Time: " << std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() << std::endl;*/

    msm.set_target_accuracy(deg2rad(1), 0.1);
    msm.set_lookup_ranges(0.5, 0.5, deg2rad(10));
    msm.process_scan(tr_scan, found_pose, this->map(), pose_delta);
    found_pose += pose_delta;
    t2 = std::chrono::system_clock::now();
    std::cout << "Fine step: " << found_pose.x << " " << found_pose.y << " " << found_pose.theta << std::endl;
    std::cout << "Time: " << std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() << std::endl;

    auto cur_sm = scan_matcher();
    cur_sm->reset_state();
    cur_sm->process_scan(tr_scan, this->pose() + RobotPoseDelta{0.5, 0.5, 0.5}, this->map(), pose_delta);
    std::cout << pose_delta.x << " " << pose_delta.y << " " << pose_delta.theta << std::endl;
    std::cout << this->pose().x << " " << this->pose().y << " " << this->pose().theta << std::endl << std::endl;

    }

  void merge_alien_map(const std::vector<std::string> & serialized_alien_map) {
    //std::shared_ptr<decltype(_map.new_cell())> sp = std::move(_map.new_cell());
    std::string scan_str = serialized_alien_map[0];

    TransformedLaserScan transformed_scan;
    std::stringstream ss;
    ss.str(scan_str);
    double dx, dy, dt, qual;
    int p_size;
    double p_range, p_angle;
    bool p_occ;

    ss >> dx >> dy >> dt >> qual;
    ss >> p_size;
    std::cout << "p_size" << p_size << std::endl;
    transformed_scan.quality = qual;
    transformed_scan.scan.points().reserve(p_size);
    for (int i = 0; i < p_size; i++) {

      ss >> p_range >> p_angle >> p_occ;
      if(i == 450)
        std::cout << p_range << " " << p_angle << " " << p_occ <<std::endl;
      transformed_scan.scan.points().emplace_back(p_range, p_angle,p_occ);

    }
    transformed_scan.scan.trig_provider = std::make_shared<RawTrigonometryProvider>();

    /*auto msm = BruteForceMultiResolutionScanMatcher(std::make_shared<WeightedMeanPointProbabilitySPE>(std::make_shared<MaxOccupancyObservationPE>(),
               std::make_shared<EvenSPW>()), deg2rad(20), 0.5);*/
    /*auto msm = MonteCarloScanMatcher(
               std::make_shared<WeightedMeanPointProbabilitySPE>(std::make_shared<MeanOccupancyObservationPE>(),std::make_shared<EvenSPW>()),
               12, 0.2, 0.1, 1000, 6000);*/
    auto msm = MCUniformScanMatcher(
        std::make_shared<WeightedMeanPointProbabilitySPE>(std::make_shared<MeanOccupancyObservationPE>(),std::make_shared<EvenSPW>()),
        12,2.0,3.14,10000);
    msm.reset_state();
    auto pose_delta = RobotPoseDelta{};
    msm.set_lookup_ranges(0.5, 0.5, deg2rad(10));

    RobotPose found_pose = {this->pose().x, this->pose().y, this->pose().theta};

    std::cout << "Real pose: " << dx << " " << dy << " " << dt << std::endl;
    std::cout << "My current pose: " << this->pose().x << " " << this->pose().y << " " << this->pose().theta << std::endl;
    //std::cout << this->pose().x << " " << this->pose().y << " " << this->pose().theta << std::endl;

    std::chrono::time_point<std::chrono::system_clock> t1 = std::chrono::system_clock::now();

    msm.process_scan(transformed_scan, found_pose, this->map(), pose_delta);
    found_pose += pose_delta;
    std::cout << "Coarse step: " << found_pose.x << " " << found_pose.y << " " << found_pose.theta << std::endl;
    std::cout << "Delta: " << pose_delta.x << " " << pose_delta.y << " " << pose_delta.theta << std::endl;
    std::chrono::time_point<std::chrono::system_clock> t2 = std::chrono::system_clock::now();
    std::cout << "Time: " << std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() << std::endl;


    /*msm.reset_state();
    msm.set_lookup_ranges(0.01, 0.01, deg2rad(0.1));
    msm.process_scan(transformed_scan, found_pose, this->map(), pose_delta);
    found_pose += pose_delta;

    t2 = std::chrono::system_clock::now();
    std::cout << "Middle step: " << found_pose.x << " " << found_pose.y << " " << found_pose.theta << std::endl;
    std::cout << "Delta: " << pose_delta.x << " " << pose_delta.y << " " << pose_delta.theta << std::endl;
    std::cout << "Time: " << std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() << std::endl;*/

    /*msm.set_target_accuracy(deg2rad(2), 0.1);
    msm.set_lookup_ranges(0.5, 0.5, deg2rad(20));
    msm.process_scan(transformed_scan, found_pose, this->map(), pose_delta);
    found_pose += pose_delta;
    t2 = std::chrono::system_clock::now();
    std::cout << "Fine step: " << found_pose.x << " " << found_pose.y << " " << found_pose.theta << std::endl;
    std::cout << "Delta: " << pose_delta.x << " " << pose_delta.y << " " << pose_delta.theta << std::endl;
    std::cout << "Time: " << std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count() << std::endl;*/
    //std::cout << "Real pose2: " << this->pose().x << " " << this->pose().y << " " << this->pose().theta << std::endl;



    //serialized_alien_map.erase(serialized_alien_map.begin());
    GridMapParams gmp{2,2,1};
    std::shared_ptr<GridCell> cell_proto (std::move(_map.new_cell()));
    std::shared_ptr<MapType> alien_map(new MapType{cell_proto, gmp});
    //MapType alien_map(_map);
    alien_map->load_map({serialized_alien_map.begin()+1, serialized_alien_map.end()});
    //this->map().merge(alien_map, this->pose(), found_pose);
    this->map().merge(alien_map, found_pose, {dx, dy, dt});
    this->notify_with_map(this->map());
  }

protected:
  Properties _props;
  MapType _map;
};

#endif
