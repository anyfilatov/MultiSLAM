#ifndef SLAM_CTOR_SLAM_VINY_MUL_MAP_H
#define SLAM_CTOR_SLAM_VINY_MUL_MAP_H

#include "../../core/maps/plain_grid_map.h"
#include "../../core/maps/rescalable_caching_grid_map.h"

//using MapType = RescalableCachingGridMap<UnboundedPlainGridMap>;
using MapType = UnboundedPlainGridMap;

class UnboundedPlainGridMapWithScan : public MapType {
public:
  UnboundedPlainGridMapWithScan(std::shared_ptr<GridCell> prototype,
                                const GridMapParams &params = MapValues::gmp) :
                                  MapType(prototype, params) {
  }

  void set_scan(const TransformedLaserScan& _scan) {
    scan = _scan;
  }
  TransformedLaserScan get_scan() const{
    return scan;
  }

private:
  TransformedLaserScan scan;
};

#endif
