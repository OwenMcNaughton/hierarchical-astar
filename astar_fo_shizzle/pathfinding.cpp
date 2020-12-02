#include "pathfinding.h"

#include <algorithm>
#include <random>

array<Point2D, 8> SquareGrid::DIRS = {
    Point2D(1, 0), Point2D(-1, 0), Point2D(0, -1),  Point2D(0, 1),
    Point2D(1, 1), Point2D(-1, 1), Point2D(-1, -1), Point2D(1, -1)};

SquareGrid::SquareGrid(int iter, const vector<int>& lowres_sizes) {
  iter_ = iter;
  lowres_sizes_ = lowres_sizes;
  bmp_ = new BMP("town.bmp");
  width_ = height_ = X;
  walls = (bool**)malloc(X * sizeof(bool*));
  weights = (float**)malloc(X * sizeof(float*));
  for (int i = 0; i < X; i++) {
    walls[i] = (bool*)malloc(X * sizeof(bool));
    weights[i] = (float*)malloc(X * sizeof(float));
  }
  for (int x = 0; x < height_; x++) {
    for (int y = 0; y < width_; y++) {
      weights[x][y] = 1;
      walls[x][y] =
          bmp_->is_pixel_black(x, y) || bmp_->is_pixel_building(x, y) ? 1 : 0;
      if (bmp_->is_pixel_green(x, y)) {
        weights[x][y] = 0.2;
        // weights[x][y] = 1;
      }
      if (bmp_->is_pixel_yellow(x, y)) {
        weights[x][y] = 30;
        // weights[x][y] = 1;
      }
      if (bmp_->is_pixel_blue(x, y)) {
        weights[x][y] = 0.5;
        // weights[x][y] = 1;
      }

      avg_weight += weights[x][y];
    }
  }
  avg_weight /= X * X;
  find_poi();
  for (int i : lowres_sizes_) {
    gen_lowres_map(i);
  }
  gen_meta_territories();
}

void SquareGrid::find_poi() {
  unordered_set<Point2D> floodfilled_buildings;

  vector<Point2D> v;
  for (int x = 0; x < height_; x++) {
    for (int y = 0; y < width_; y++) {
      v.push_back(Point2D(x, y));
    }
  }
  random_shuffle(v.begin(), v.end());

  for (const auto& p : v) {
    if (passable(p)) {
      auto ns = neighbors(p, 0, 0, width_, true);
      for (const auto& n : ns) {
        if (in_bounds(n, 0, 0, width_) && bmp_->is_pixel_building(n.x_, n.y_) &&
            floodfilled_buildings.count(Point2D(n.x_, n.y_)) == 0) {
          floodfill_building(n.x_, n.y_, floodfilled_buildings);

          if (bmp_->is_residential(n.x_, n.y_)) {
            residential.push_back(p);
          } else if (bmp_->is_commercial(n.x_, n.y_)) {
            commercial.push_back(p);
          }
        }
      }
    }
  }
}

void SquareGrid::gen_lowres_map(int size) {
  vector<LowresTile*> lowres_map;
  for (int i = 0; i != width_; i += size) {
    for (int j = 0; j != height_; j += size) {
      auto tiles = gen_lowres_tiles(size, i, j);
      for (LowresTile* t : tiles) {
        lowres_map.push_back(t);
      }
    }
  }
  lowres_maps_[size] = lowres_map;
  unordered_map<LowresTile*, unordered_set<LowresTile*>> territory_adj;
  for (int i = 0; i != width_; i++) {
    for (int j = 0; j != height_; j++) {
      if (territories_[size].count(Point2D(i, j)) == 0) {
        continue;
      }
      auto ns = neighbors(Point2D(i, j), 0, 0, width_);
      for (const auto& n : ns) {
        LowresTile* lrt = territories_[size][n];
        territory_adj[territories_[size][Point2D(i, j)]].insert(lrt);
      }
    }
  }
  for (auto& p : territory_adj) {
    auto it = p.second.find(p.first);
    if (it != p.second.end()) {
      p.second.erase(it);
    }
  }
  territory_adj_[size] = territory_adj;
}

vector<LowresTile*> SquareGrid::gen_lowres_tiles(int size, int x, int y) {
  unordered_set<Point2D> territory;
  vector<LowresTile*> lrts;
  for (int i = x; i != x + size; i++) {
    for (int j = y; j != y + size; j++) {
      Point2D p(i, j);
      if (!passable(Point2D(i, j))) {
        continue;
      }
      if (territory.count(p) == 0) {
        unordered_set<Point2D> new_territory;
        auto lrt = gen_lowres_tile(size, i, j, new_territory);
        lrts.push_back(lrt);
        for (const auto& p : lrt->territory_) {
          territory.insert(p);
        }
      }
    }
  }
  return lrts;
}

LowresTile* SquareGrid::gen_lowres_tile(int size, int x, int y,
                                        unordered_set<Point2D>& territory) {
  floodfill(size, x, y, territory);
  LowresTile* lrt = new LowresTile();
  lrt->territory_ = territory;
  lrt->resolution_ = size;
  float total_weight = 0;
  int64_t cx = 0;
  int64_t cy = 0;
  bool has_roads = false;
  for (const auto& p : lrt->territory_) {
    float w = weights[p.x_][p.y_];
    total_weight += w < 1 ? w / 4 : w;
    territories_[size].emplace(p, lrt);
    cx += p.x_;
    cy += p.y_;
  }
  lrt->centroid_.x_ = cx / lrt->territory_.size();
  lrt->centroid_.y_ = cy / lrt->territory_.size();
  lrt->avg_weight_ = total_weight / lrt->territory_.size();
  if (lrt->avg_weight_ < 1) {
    lrt->avg_weight_ /= 2;
  }
  return lrt;
}

void SquareGrid::floodfill(int size, int x, int y,
                           unordered_set<Point2D>& territory) {
  if (!passable(Point2D(x, y)) || territory.count(Point2D(x, y)) != 0) {
    return;
  }
  territory.insert(Point2D(x, y));
  auto ns = neighbors(Point2D(x, y), (x / size) * size, (y / size) * size, size,
                      false);
  for (const auto& n : ns) {
    floodfill(size, n.x_, n.y_, territory);
  }
}

void SquareGrid::floodfill_building(int x, int y,
                                    unordered_set<Point2D>& territory) {
  if (passable(Point2D(x, y)) || territory.count(Point2D(x, y)) != 0) {
    return;
  }
  territory.insert(Point2D(x, y));
  auto ns = neighbors(Point2D(x, y), 0, 0, width_, true);
  for (const auto& n : ns) {
    floodfill_building(n.x_, n.y_, territory);
  }
}

void SquareGrid::gen_meta_territories() {
  unordered_map<Point2D, unordered_set<LowresTile*>> flat_territories;
  for (const auto& lowres_map : territories_) {
    for (const auto& point : lowres_map.second) {
      flat_territories[point.first].insert(point.second);
    }
  }
  for (const auto& point : flat_territories) {
    for (auto a : point.second) {
      for (auto b : point.second) {
        // dont point at urself
        if (a == b) {
          continue;
        }
        // a must be a map with greater res than b
        if (a->resolution_ > b->resolution_) {
          continue;
        }
        meta_territories_[a].emplace(b->resolution_, b);
      }
    }
  }
}

SquareGrid::~SquareGrid() {
  for (int i = 0; i < X; i++) {
    delete walls[i];
  }
  for (int i = 0; i < X; i++) {
    delete weights[i];
  }
  delete walls;
  delete weights;
  delete bmp_;
  for (auto& p : lowres_maps_) {
    for (LowresTile* lrt : p.second) {
      delete lrt;
    }
  }
}

pair<Point2D, Point2D> SquareGrid::get_rand_startend() const {
  random_device rd;
  mt19937 gen(rd());
  uniform_int_distribution<> _0_5(0, 0);
  if (_0_5(gen) == 0) {
    return make_pair(get_rand_passable(), get_rand_passable());
  } else {
    uniform_int_distribution<> _1_100(1, 100);
    int r = _1_100(gen);
    uniform_int_distribution<> com(0, commercial.size() - 1);
    uniform_int_distribution<> res(0, residential.size() - 1);
    if (r < 56) {
      return make_pair(commercial[com(gen)], commercial[com(gen)]);
    } else if (r < 73) {
      return make_pair(residential[res(gen)], commercial[com(gen)]);
    } else if (r < 90) {
      return make_pair(commercial[com(gen)], residential[res(gen)]);
    } else {
      return make_pair(residential[res(gen)], residential[res(gen)]);
    }
  }
}

Point2D SquareGrid::get_rand_passable() const {
  random_device rd;
  mt19937 gen(rd());
  uniform_int_distribution<> distrib(0, X - 1);
  for (;;) {
    int x = distrib(gen);
    int y = distrib(gen);
    if (!walls[x][y]) {
      return Point2D(x, y);
    }
  }
}

bool SquareGrid::in_bounds(Point2D id, int x, int y, int step) const {
  return x <= id.x_ && id.x_ < (x + step) && y <= id.y_ && id.y_ < (y + step);
}

float SquareGrid::cost(Point2D a, Point2D b, bool is_near) const {
  float weight = is_near ? 1 : weights[b.x_][b.y_];
  return ((abs(a.x_ - b.x_) + abs(a.y_ - b.y_)) == 2 ? 1.414 : 1) * weight;
}

float SquareGrid::cost_no_weight(FPoint2D a, Point2D b) const {
  return sqrt(pow(a.x_ - b.x_, 2) + pow(a.y_ - b.y_, 2));
}

float SquareGrid::cost_no_weight(Point2D a, Point2D b) const {
  return sqrt(pow(a.x_ - b.x_, 2) + pow(a.y_ - b.y_, 2));
}


float SquareGrid::cost(LowresTile* a, LowresTile* b, bool is_near) const {
  return sqrt(pow(a->centroid_.x_ - b->centroid_.x_, 2) +
              pow(a->centroid_.y_ - b->centroid_.y_, 2)) *
         b->avg_weight_;
}

bool SquareGrid::passable(Point2D id) const { return !walls[id.x_][id.y_]; }

vector<Point2D> SquareGrid::neighbors(Point2D id, int x, int y, int step,
                                      bool skip_pass) const {
  vector<Point2D> results;
  for (Point2D dir : DIRS) {
    Point2D next(id.x_ + dir.x_, id.y_ + dir.y_);
    if (in_bounds(next, x, y, step) && (skip_pass || passable(next))) {
      results.push_back(next);
    }
  }
  return results;
}

unordered_set<LowresTile*> SquareGrid::neighbors(LowresTile* id,
                                                 int lowres_size) const {
  return territory_adj_.at(lowres_size).at(id);
}

void SquareGrid::record_path_traffic(const vector<Point2D>& path,
                                     float path_cost) {
  mu_.lock();
  for (const auto& s : path) {
    if (traffic_.count(s) == 0) {
      traffic_.emplace(s, 1);
    } else {
      traffic_[s] += 1;
    }
  }
  mu_.unlock();
}

inline float heuristic(Point2D a, Point2D b) {
  float h = (abs(a.x_ - b.x_) + abs(a.y_ - b.y_));
  return h;
  return pow(h, 0.9);
}

inline float heuristic(LowresTile* a, LowresTile* b) {
  float h = (abs(a->centroid_.x_ - b->centroid_.x_) +
             abs(a->centroid_.y_ - b->centroid_.y_));
  return h;
  return pow(h, 0.9);
}

void a_star_search(
    SquareGrid& graph, const Point2D& start, const Point2D& goal,
    ThreadGrid& came_from, unordered_map<Point2D, float>& cost_so_far,
    const unordered_set<LowresTile*>& lowres_path,
    const unordered_map<Point2D, LowresTile*>& lowres_territory) {
  PriorityQueue<Point2D, float> frontier;
  frontier.put(start, 0);

  came_from.set(start, Point2D(-1, -1));
  cost_so_far[start] = 0;

  bool is_near = false;
  // heuristic(start, goal) < 400;

  while (!frontier.empty()) {
    Point2D current = frontier.get();

    if (current == goal) {
      came_from.mark_completed();
      return;
    }

    // this_thread::sleep_for(5ms);

    for (Point2D next : graph.neighbors(current, 0, 0, graph.width_)) {
      if (!lowres_path.empty()) {
        LowresTile* next_territory = lowres_territory.at(next);
        if (lowres_path.count(next_territory) == 0) {
          continue;
        }
      }

      float new_cost =
          cost_so_far[current] + graph.cost(current, next, is_near);
      if (cost_so_far.find(next) == cost_so_far.end() ||
          new_cost < cost_so_far[next]) {
        cost_so_far[next] = new_cost;
        float priority = new_cost;  // + heuristic(next, goal);
        frontier.put(next, priority);
        came_from.set(next, current);
      }
    }
  }
}

void a_star_search(SquareGrid& graph, LowresTile* start, LowresTile* goal,
                   shared_ptr<ThreadGrid>& came_from,
                   unordered_map<LowresTile*, float>& cost_so_far,
                   const unordered_set<LowresTile*>& lowres_path,
                   const unordered_map<LowresTile*, unordered_set<LowresTile*>>&
                       territory_adj) {
  PriorityQueue<LowresTile*, float> frontier;
  frontier.put(start, 0);

  came_from->set(start, nullptr);
  cost_so_far[start] = 0;

  bool is_near = false;
  // heuristic(start, goal) < 400;

  while (!frontier.empty()) {
    LowresTile* current = frontier.get();

    // this_thread::sleep_for(1ms);
    if (current == goal) {
      came_from->mark_completed();
      return;
    }

    for (LowresTile* next : territory_adj.at(current)) {
      if (!lowres_path.empty()) {
        LowresTile* next_territory = graph.meta_territories_.at(next).at(
            (*lowres_path.begin())->resolution_);
        if (lowres_path.count(next_territory) == 0) {
          continue;
        }
      }

      float new_cost =
          cost_so_far[current] + graph.cost(current, next, is_near);
      if (cost_so_far.find(next) == cost_so_far.end() ||
          new_cost < cost_so_far[next]) {
        cost_so_far[next] = new_cost;
        float priority = new_cost;
        //+ heuristic(next, goal);
        frontier.put(next, priority);
        came_from->set(next, current);
      }
    }
  }
}