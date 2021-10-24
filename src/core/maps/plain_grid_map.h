#ifndef SLAM_CTOR_CORE_PLAIN_GRID_MAP_H_INCLUDED
#define SLAM_CTOR_CORE_PLAIN_GRID_MAP_H_INCLUDED

#include <cmath>
#include <vector>
#include <memory>
#include <cassert>
#include <algorithm>

#include "grid_map.h"


class PlainGridMap : public GridMap {
public:
  // TODO: cp, mv ctors, dtor
  PlainGridMap(std::shared_ptr<GridCell> prototype,
               const GridMapParams& params = MapValues::gmp)
    : GridMap{prototype, params}, _cells(GridMap::height()) {
    for (auto &row : _cells) {
      row.reserve(GridMap::width());
      for (int i = 0; i < GridMap::width(); i++) {
        row.push_back(prototype->clone());
      }
    }
  }

  const GridCell &operator[](const Coord& c) const override {
    auto coord = external2internal(c);
    assert(has_internal_cell(coord));
    return cell_internal(coord);
  }

protected: // fields

  const GridCell& cell_internal(const Coord& ic) const {
    return *_cells[ic.y][ic.x];
  }

  std::vector<std::vector<std::unique_ptr<GridCell>>> _cells;
};

/* Unbounded implementation */

class UnboundedPlainGridMap : public PlainGridMap {
private: // fields
  static constexpr double Expansion_Rate = 1.2;
public: // methods
  UnboundedPlainGridMap(std::shared_ptr<GridCell> prototype,
                        const GridMapParams &params = MapValues::gmp)
    : PlainGridMap{prototype, params}
    , _origin{GridMap::origin()}, _unknown_cell{prototype->clone()} {}

  void update(const Coord &area_id,
              const AreaOccupancyObservation &aoo) override {
    ensure_inside(area_id);
    PlainGridMap::update(area_id, aoo);
  }

  void reset(const Coord &area_id, const GridCell &new_area) override {
    ensure_inside(area_id);
    auto ic = external2internal(area_id);
    _cells[ic.y][ic.x].reset(new_area.clone().release());
    //PlainGridMap::reset(area_id, new_area);
  }

  void merge(std::shared_ptr<GridMap> another,
             const RobotPose& own_robot, const RobotPose& alien_robot) {

    auto const_this = static_cast<const decltype(this)>(this);
    int left = -another->origin().x;
    int right = another->width() - another->origin().x;
    int bot = -another->origin().y;
    int top = (another->height() - another->origin().y);

    /* X_in_alien_map = C_alien * X_in_alien_robot + Sh_alien
     * X_in_alien_robot = C_alien ^ (-1) * (X_in_alien_map - Sh_alien)
     * X_in_this_map = C_this * X_in_this_robot  + Sh_this
     * X_in_this_map = C_this*C_alien^(-1) * (X_in_alien_map-Sh_alien) +Sh_this
     * where C  - rotation matrix
     *       Sh - shift
     * So result rotation angle = this_angle - alien_angle
     */

    double angle = own_robot.theta - alien_robot.theta;
    double cos_a = std::cos(angle);
    double sin_a = std::sin(angle);

    for (int i = left; i < right; i++) {
      for (int j = bot; j < top; j++) {
        Coord cur_cell(i,j);
        // cur_cell will be transformed into own coordinates

        double x = cell_to_world(cur_cell).x - alien_robot.x;
        double y = cell_to_world(cur_cell).y - alien_robot.y;
        double x_in_this = (cos_a*x - sin_a*y) + own_robot.x;
        double y_in_this = (sin_a*x + cos_a*y) + own_robot.y;
        cur_cell = world_to_cell(x_in_this, y_in_this);

        ensure_inside(cur_cell);
        auto& grid_cell = const_cast<GridCell&>((*const_this)[cur_cell]);
        grid_cell.merge(another->operator [](Coord{i,j}));
      }
    }
  }

  const GridCell &operator[](const Coord& ec) const override {

    auto ic = external2internal(ec);
    if (!PlainGridMap::has_internal_cell(ic)) { return *_unknown_cell; }
    return PlainGridMap::cell_internal(ic);
  }

  Coord origin() const override { return _origin; }

  bool has_cell(const Coord &) const override { return true; }

  std::vector<char> save_state() const override {
    auto w = width(), h = height();
    size_t map_size_bytes = w * h * _unknown_cell->serialize().size();

    Serializer s(sizeof(GridMapParams) + sizeof(Coord) + map_size_bytes);
    s << h << w << scale() << origin().x << origin().y;

    Serializer ms(map_size_bytes);
    for (auto &row : _cells) {
      for (auto &cell : row) {
        ms.append(cell->serialize());
      }
    }
  #ifdef COMPRESSED_SERIALIZATION
    s.append(ms.compressed());
  #else
    s.append(ms.result());
  #endif
    return s.result();
  }

  void load_state(const std::vector<char>& data) override {
    decltype(width()) w, h;
    decltype(scale()) s;

    Deserializer d(data);
    d >> h >> w >> s >> _origin.x >> _origin.y;

    set_width(w);
    set_height(h);
    set_scale(s);
  #ifdef COMPRESSED_SERIALIZATION
    std::vector<char> map_data = Deserializer::decompress(
        data.data() + d.pos(), data.size() - d.pos(),
        w * h * _unknown_cell->serialize().size());
    size_t pos = 0;
  #else
    const std::vector<char> &map_data = data;
    size_t pos = d.pos();
  #endif
    _cells.clear();
    _cells.resize(h);
    for (auto &row : _cells) {
      row.reserve(w);
      for (int i = 0; i < w; ++i) {
        auto cell = new_cell();
        pos = cell->deserialize(map_data, pos);
        row.push_back(std::move(cell));
      }
    }
  }

  std::vector<std::string> save_map() const {
    std::vector<std::string> res;
    auto map_data = this->save_state();
    std::string map_string(map_data.begin(), map_data.end());
    res.push_back(map_string);
    return res;
  }

  void load_map(const std::vector<std::string>& map_vector) {
    auto map_string = map_vector[0];
    std::vector<char> map_loader(map_string.begin(), map_string.end());
    this->load_state(map_loader);
  }

protected: // methods

  bool ensure_inside(const Coord &c) {
    auto coord = external2internal(c);
    if (PlainGridMap::has_internal_cell(coord)) return false;

    unsigned w = width(), h = height();
    unsigned prep_x = 0, app_x = 0, prep_y = 0, app_y = 0;
    std::tie(prep_x, app_x) = determine_cells_nm(0, coord.x, w);
    std::tie(prep_y, app_y) = determine_cells_nm(0, coord.y, h);

    unsigned new_w = prep_x + w + app_x, new_h = prep_y + h + app_y;
    #define UPDATE_DIM(dim, elem)                                    \
      if (dim < new_##dim && new_##dim < Expansion_Rate * dim) {     \
        double scale = prep_##elem / (new_##dim - dim);              \
        prep_##elem += (Expansion_Rate * dim - new_##dim) * scale;   \
        new_##dim = Expansion_Rate * dim;                            \
        app_##elem = new_##dim - (prep_##elem + dim);                \
      }

    UPDATE_DIM(w, x);
    UPDATE_DIM(h, y);
    #undef UPDATE_DIM

    // PERFORMANCE: _cells can be reused
    std::vector<std::vector<std::unique_ptr<GridCell>>> new_cells{new_h};
    for (size_t y = 0; y != new_h; ++y) {
      std::generate_n(std::back_inserter(new_cells[y]), new_w,
                      [this](){ return this->_unknown_cell->clone(); });
      if (y < prep_y || prep_y + h <= y) { continue; }

      std::move(_cells[y - prep_y].begin(), _cells[y - prep_y].end(),
                &new_cells[y][prep_x]);
    }

    std::swap(_cells, new_cells);
    set_height(new_h);
    set_width(new_w);
    _origin += Coord(prep_x, prep_y);

    assert(PlainGridMap::has_cell(c));
    return true;
  }

  std::tuple<unsigned, unsigned> determine_cells_nm(
    int min, int val, int max) const {
    assert(min <= max);
    unsigned prepend_nm = 0, append_nm = 0;
    if (val < min) {
      prepend_nm = min - val;
    } else if (max <= val) {
      append_nm = val - max + 1;
    }
    return std::make_tuple(prepend_nm, append_nm);
  }

private: // fields
  Coord _origin;
  std::shared_ptr<GridCell> _unknown_cell;
};

#endif
