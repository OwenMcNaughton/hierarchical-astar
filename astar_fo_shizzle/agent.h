#ifndef AGENT_H
#define AGENT_H

#include <SDL.h>
#include <SDL_image.h>

#include "pathfinding.h"
#include "util.h"

class Agent {
 public:
  Agent(SquareGrid* grid);

  void update_pos(float dt);

  void find_goal();

  bool claim_tile(int tries = 1);

  void find_path();

  void render(SDL_Renderer* gRenderer, float scale);

 private:
  Point2D last_, next_, goal_;
  FPoint2D curr_;
  SquareGrid* grid_;
  ThreadGrid came_from_;
  unordered_map<Point2D, float> cost_so_far_;
  unordered_map<int, shared_ptr<ThreadGrid>> came_from_lowres_;
  unordered_map<int, unordered_map<LowresTile*, float>> cost_so_far_lowres_;
  unordered_map<int, unordered_set<LowresTile*>> lowres_paths_;
  vector<Point2D> path_;
};

#endif