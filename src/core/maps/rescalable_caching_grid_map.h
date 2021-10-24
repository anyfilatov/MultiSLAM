#ifndef SLAM_CTOR_CORE_RESCALABLE_CACHING_GRID_MAP_H
#define SLAM_CTOR_CORE_RESCALABLE_CACHING_GRID_MAP_H

#include <memory>
#include <cassert>
#include <limits>
#include <utility>

#include "grid_cell.h"
#include "grid_map.h"
#include "grid_rasterization.h"

template <typename BackGridMap>
class RescalableCachingGridMap : public GridMap {
private: // type aliases
  using MapCache = std::vector<std::unique_ptr<GridMap>>;
private: // consts
  static constexpr int Coarsest_Map_W = 1, Coarsest_Map_H = 1;
public: // consts
  static constexpr unsigned Map_Scale_Factor = 2;
public:
  RescalableCachingGridMap(std::shared_ptr<GridCell> prototype,
                           const GridMapParams& params = MapValues::gmp)
    : GridMap{prototype, params}
    , _map_cache{std::make_shared<MapCache>()} {

    // the finest map
    _map_cache->push_back(std::make_unique<BackGridMap>(prototype, params));
    // the coarsest map
    auto coarsest_mp = GridMapParams{Coarsest_Map_W, Coarsest_Map_H,
                                     std::numeric_limits<double>::infinity()};
    auto coarsest_map = std::make_unique<BackGridMap>(prototype, coarsest_mp);
    _map_cache->push_back(std::move(coarsest_map));
    ensure_map_cache_is_continuous();

    set_scale_id(finest_scale_id());
  }

  RescalableCachingGridMap(const RescalableCachingGridMap&) = delete;
  RescalableCachingGridMap& operator=(const RescalableCachingGridMap&) = delete;
  RescalableCachingGridMap(RescalableCachingGridMap&&) = default;
  RescalableCachingGridMap& operator=(RescalableCachingGridMap&&) = default;

  //----------------------------------------------------------------------------
  // API for manual scale management.

  unsigned scales_nm() const {
    ensure_map_cache_is_continuous();
    return _map_cache->size();
  }

  unsigned scale_id() const { return _scale_id; }

  static constexpr unsigned finest_scale_id() { return 0; }
  unsigned coarsest_scale_id() const { return scales_nm() - 1; }

  void set_scale_id(unsigned scale_id) {
    assert(scale_id < scales_nm());
    _scale_id = scale_id;
    _active_map = &map(_scale_id);
  }

  //----------------------------------------------------------------------------
  // RegularSquaresGrid overrides

  Coord origin() const override { return active_map().origin(); }
  int width() const override { return active_map().width(); }
  int height() const override { return active_map().height(); }
  double scale() const override { return active_map().scale(); }
  bool has_cell(const Coord &c) const override {
    return active_map().has_cell(c);
  }

  void rescale(double target_scale) override {
    ensure_map_cache_is_continuous();

    // TODO: replace the linear probing
    unsigned scale_id = finest_scale_id();
    while (1) {
      if (target_scale <= map(scale_id).scale()) {
        break;
      }
      ++scale_id;
    }
    set_scale_id(scale_id);
  }

  //----------------------------------------------------------------------------
  // GridMap overrides

  const GridCell& operator[](const Coord &coord) const override {
    return active_map()[coord];
  }

  void update(const Coord &area_id,
              const AreaOccupancyObservation &aoo) override {
    active_map().update(area_id, aoo);
    on_area_update(area_id);
  }

  void reset(const Coord &area_id,
             const GridCell &area) override {
    active_map().reset(area_id, area);
    on_area_update(area_id);
  }


  void merge(const std::shared_ptr<RescalableCachingGridMap<BackGridMap>> another,
               const RobotPose& own_robot, const RobotPose& alien_robot) {
    unsigned active_id = scale_id();
    rescale(finest_scale_id());
    //active_map().merge(std::make_shared<GridMap>(another->map(finest_scale_id())), own_robot, alien_robot);

    int left = -active_map().origin().x;
    int right = active_map().width() - active_map().origin().x;
    int bot = -active_map().origin().y;
    int top = active_map().height() - active_map().origin().y;

    for (int i = left; i<right; i++){
      for (int j = bot; j < top; j++){
        on_area_update(Coord{i,j});
      }
    }

    rescale(active_id);

  }

  std::vector<std::string> save_map() const {

    //first element - id of active map
    //rest elements - maps from id 0 to the end
    std::vector<std::string> res;
    res.push_back(std::to_string(_scale_id));

    for(auto& each_map : *(_map_cache)) {
      auto map_data = each_map->save_state();
      std::string map_string(map_data.begin(), map_data.end());
      res.push_back(map_string);
    }
    return res;
  }

  void load_map(const std::vector<std::string>& map_vector) {

    _scale_id = unsigned(std::stoi(map_vector[0]));
    std::cout << std::stoi(map_vector[0]) << std::endl;

    _map_cache->clear();

    for (unsigned int i = 1; i < map_vector.size(); i++){
      auto map_string = map_vector[i];
      std::vector<char> map_loader(map_string.begin(), map_string.end());

      auto each_map = std::make_unique<BackGridMap>(cell_prototype(), GridMapParams{1000,1000,0.1});
      each_map->load_state(map_loader);
      _map_cache->push_back(std::move(each_map));
    }
    set_scale_id(_scale_id);

  }

private:

  void on_area_update(const Coord &area_id) {
    // FIXME: code duplication expected_scan_point_observation
    auto aoo = AreaOccupancyObservation{true, {1.0, 1.0}, {0, 0}, 1.0};
    using GRRectangle = GridRasterizedRectangle;
    // TODO: update if a "non-finest" cell is updated?
    assert(_scale_id == finest_scale_id());
    auto modified_area = active_map()[area_id];
    auto modified_space = active_map().world_cell_bounds(area_id);

    for (unsigned coarser_scale_id = _scale_id + 1;
         coarser_scale_id <= coarsest_scale_id(); ++coarser_scale_id) {
      auto& coarser_map = map(coarser_scale_id);
      bool coarser_area_is_updated = false;
      auto cm_coords = GRRectangle{coarser_map, modified_space, false};
      while (cm_coords.has_next()) {
        auto coord = cm_coords.next();
        auto &coarser_area = coarser_map[coord];
        auto prob_md = 1.0 - modified_area.discrepancy(aoo);
        auto prob_cr = 1.0 - coarser_area.discrepancy(aoo);

        if (!coarser_area.is_unknown() && prob_md <= prob_cr) { continue; }
        coarser_map.reset(coord, modified_area);
        coarser_area_is_updated = true;
      }
      if (!coarser_area_is_updated) { break; }
    }
  }

  const GridMap& map(unsigned scale_id) const {
    return *(*_map_cache)[scale_id];
  }

  GridMap& map(unsigned scale_id) {
    return const_cast<GridMap&>(
      static_cast<const RescalableCachingGridMap&>(*this).map(scale_id));
  }

  const GridMap& active_map() const {
    return *_active_map;
  }

  GridMap& active_map() {
    return const_cast<GridMap&>(
      static_cast<const RescalableCachingGridMap&>(*this).active_map());
  }

  void ensure_map_cache_is_continuous() const {
    static const int PC_W_Target = Coarsest_Map_W * Map_Scale_Factor,
                     PC_H_Target = Coarsest_Map_H * Map_Scale_Factor;

    const GridMap& pre_coarsest_map = map(_map_cache->size() - 2);
    int pc_w = pre_coarsest_map.width(), pc_h = pre_coarsest_map.height();
    double pc_scale = pre_coarsest_map.scale();

    if (pc_w <= PC_W_Target && pc_h <= PC_H_Target) { return; }

    pc_w = ge_pow<Map_Scale_Factor>(pc_w);
    pc_h = ge_pow<Map_Scale_Factor>(pc_h);
    // more cache levels have to be added
    while (PC_W_Target < pc_w || PC_H_Target < pc_h) {
      pc_w = std::max(PC_W_Target, int(std::ceil(pc_w / Map_Scale_Factor)));
      pc_h = std::max(PC_H_Target, int(std::ceil(pc_h / Map_Scale_Factor)));
      pc_scale *= Map_Scale_Factor;

      auto map_params = GridMapParams{pc_w, pc_h, pc_scale};
      auto map = std::make_unique<BackGridMap>(cell_prototype(), map_params);
      _map_cache->insert((_map_cache->rbegin() + 1).base(), std::move(map));
    }
  }



private:
  GridMap *_active_map = nullptr;
  unsigned _scale_id = -1;
  mutable std::shared_ptr<MapCache> _map_cache;
};

// a RAII for const grid map rescaling
class SafeRescalableMap {
public:
 SafeRescalableMap(const GridMap &map)
   : _vanilla_scale{map.scale()}, _map{const_cast<GridMap&>(map)} {}
  ~SafeRescalableMap() { _map.rescale(_vanilla_scale); }
  SafeRescalableMap(const SafeRescalableMap&) = delete;
  SafeRescalableMap& operator=(const SafeRescalableMap&) = delete;
  SafeRescalableMap(SafeRescalableMap&&) = delete;
  SafeRescalableMap& operator=(SafeRescalableMap&&) = delete;

  operator GridMap&() { return _map; }
private:
  double _vanilla_scale;
  GridMap &_map;
};

#endif // header guard
