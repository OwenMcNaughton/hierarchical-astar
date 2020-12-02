#include "agent.h"

#include <algorithm>

Agent::Agent(SquareGrid* grid) : grid_(grid) {
  for (auto r : grid_->lowres_sizes_) {
    came_from_lowres_[r] = make_shared<ThreadGrid>();
    cost_so_far_lowres_[r] = unordered_map<LowresTile*, float>();
  }
}

void Agent::find_goal() { goal_ = grid_->get_rand_startend().second; }
//void Agent::find_start() {
//  last_ = grid_->get_rand_startend().first;
//  curr_.x_ = last_.x_;
//  curr_.y_ = last_.y_;
//}

void Agent::find_path() {
  int r;
  LowresTile *lowres_s, *lowres_e;
  unordered_set<LowresTile*> lrp = {};
  for (int j = 0; j != grid_->lowres_sizes_.size(); j++) {
    Stopwatch sw;
    r = grid_->lowres_sizes_[j];
    lowres_s = grid_->territories_[r][last_];
    lowres_e = grid_->territories_[r][goal_];
    a_star_search(*grid_, lowres_s, lowres_e, came_from_lowres_[r],
                  cost_so_far_lowres_[r], lrp, grid_->territory_adj_[r]);
    lrp = came_from_lowres_[r]->reconstruct_path(lowres_s, lowres_e);
    lowres_paths_[r] = lrp;
  }
  a_star_search(*grid_, last_, goal_, came_from_, cost_so_far_, lrp,
                grid_->territories_[r]);
  auto path = came_from_.reconstruct_path(last_, goal_);
  path_ = path;
  reverse(path_.begin(), path_.end());
  path_.pop_back();
  next_ = path_.back();
  // grid_->record_path_traffic(path,
  //                           came_from_.path_cost(curr_, goal_,
  //                           cost_so_far_));
}

bool Agent::claim_tile(int tries) {
  LowresTile* lrtc = grid_->territories_.at(2).at(last_);
  LowresTile* lrtn = grid_->territories_.at(2).at(next_);
  tries--;
  if (lrtn->mu_.try_lock()) {
    if (lrtc->mu_.try_lock()) {
      if (!lrtn->occupied_) {
        lrtn->occupied_ = true;
        lrtc->occupied_ = false;
      }
      lrtc->mu_.unlock();
    }
    lrtn->mu_.unlock();
    return lrtn->occupied_ && !lrtc->occupied_;
  }
  if (tries == 0) {
    find_path();
  } else {
    this_thread::sleep_for(500ms);
    claim_tile(tries);
  }
  return false;
}

void Agent::update_pos(float dt) {
  float xdir = next_.x_ - last_.x_;
  float ydir = next_.y_ - last_.y_;
  curr_.x_ += xdir * dt;
  curr_.y_ += ydir * dt;
  float exp_dist = grid_->cost_no_weight(next_, last_);
  float curr_dist = grid_->cost_no_weight(curr_ + FPoint2D(xdir, ydir), last_);

  if (curr_dist > exp_dist) {
    curr_.x_ = next_.x_;
    curr_.y_ = next_.y_;
    last_.x_ = next_.x_;
    last_.y_ = next_.y_;
    if (path_.empty()) {
      find_goal();
    }
    next_ = path_.back();
    path_.pop_back();
    claim_tile();
  }
}

void Agent::render(SDL_Renderer* gRenderer, float scale) {
  SDL_SetRenderDrawColor(gRenderer, 255, 0, 255, 255);
  SDL_Rect r = {curr_.x_ * scale, (X - curr_.y_) * scale, scale * 2, scale * 2};
  SDL_RenderFillRect(gRenderer, &r);
}