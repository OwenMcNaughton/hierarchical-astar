#ifndef PATHFINDING_H
#define PATHFINDING_H

#include <unordered_map>

#include "bmp.h"
#include "util.h"

struct SquareGrid {
  static array<Point2D, 8> DIRS;

  int width_, height_, iter_ = 0;
  vector<int> lowres_sizes_;
  Point2D start_, end_;
  bool** walls;
  float** weights;
  float avg_weight = 0;
  int16_t** near_weights;
  vector<Point2D> residential;
  vector<Point2D> commercial;
  vector<Point2D> preds = {{60, 350}, {170, 310}};
  BMP* bmp_;
  unordered_map<int, vector<LowresTile*>>
      lowres_maps_;  // I'm the owner of all LowresTile pointers
  unordered_map<int, unordered_map<Point2D, LowresTile*>> territories_;
  unordered_map<LowresTile*, unordered_map<int, LowresTile*>> meta_territories_;
  unordered_map<int, unordered_map<LowresTile*, unordered_set<LowresTile*>>>
      territory_adj_;
  unordered_map<Point2D, int> traffic_;
  mutex mu_;

  SquareGrid(int iter, const vector<int>& lowres_sizes);
  void find_poi();
  void gen_lowres_map(int size);
  vector<LowresTile*> gen_lowres_tiles(int size, int x, int y);
  LowresTile* gen_lowres_tile(int size, int x, int y,
                              unordered_set<Point2D>& territory);
  void floodfill(int size, int x, int y, unordered_set<Point2D>& territory);
  void floodfill_building(int x, int y, unordered_set<Point2D>& territory);
  void gen_meta_territories();

  ~SquareGrid();

  pair<Point2D, Point2D> get_rand_startend() const;

  Point2D get_rand_passable() const;

  bool in_bounds(Point2D id, int x, int y, int step) const;

  float cost(Point2D a, Point2D b, bool is_near) const;
  float cost_no_weight(FPoint2D a, Point2D b) const;
  float cost_no_weight(Point2D a, Point2D b) const;
  float cost(LowresTile* a, LowresTile* b, bool is_near) const;

  bool passable(Point2D id) const;

  vector<Point2D> neighbors(Point2D id, int x, int y, int step,
                            bool skip_pass = false) const;
  unordered_set<LowresTile*> neighbors(LowresTile* id, int lowres_size) const;

  void record_path_traffic(const vector<Point2D>& path, float path_cost = 1);
};

inline float heuristic(Point2D a, Point2D b);

void a_star_search(SquareGrid& graph, const Point2D& start, const Point2D& goal,
                   ThreadGrid& came_from,
                   unordered_map<Point2D, float>& cost_so_far,
                   const unordered_set<LowresTile*>& lowres_path,
                   const unordered_map<Point2D, LowresTile*>& lowres_territory);

void a_star_search(SquareGrid& graph, LowresTile* start, LowresTile* goal,
                   shared_ptr<ThreadGrid>& came_from,
                   unordered_map<LowresTile*, float>& cost_so_far,
                   const unordered_set<LowresTile*>& lowres_path,
                   const unordered_map<LowresTile*, unordered_set<LowresTile*>>&
                       territory_adj);

#endif