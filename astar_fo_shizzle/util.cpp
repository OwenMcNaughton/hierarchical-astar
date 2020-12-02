#include "util.h"

#include <tuple>

uint64_t getUnixTimeStamp() {
  auto secs = static_cast<chrono::seconds>(time(nullptr)).count();
  return static_cast<uint64_t>(secs);
}

bool Stopwatch::is_running() const {
  return stop_time_ == chrono::high_resolution_clock::time_point::min();
}

chrono::high_resolution_clock::time_point Stopwatch::end_time() const {
  return is_running() ? chrono::high_resolution_clock::now() : stop_time_;
}

chrono::high_resolution_clock::time_point begin_time_{
    chrono::high_resolution_clock::now() },
    stop_time_{ chrono::high_resolution_clock::time_point::min() };

void Stopwatch::stop() {
  if (is_running()) stop_time_ = chrono::high_resolution_clock::now();
}

double Stopwatch::elapsed() const {
  return (end_time() - begin_time_).count() / 1000000.0;
}

void ThreadGrid::set(const Point2D& k, const Point2D& v) {
  mu_.lock();
  grid_[k] = v;
  newest_ = make_pair(k, v);
  mu_.unlock();
}

Point2D ThreadGrid::get(const Point2D& k) {
  mu_.lock();
  auto a = grid_.at(k);
  mu_.unlock();
  return a;
}

bool ThreadGrid::empty() {
  mu_.lock();
  auto a = grid_.empty();
  mu_.unlock();
  return a;
}

bool ThreadGrid::has(const Point2D& k) const {
  //mu_.lock();
  auto a = grid_.find(k) == grid_.end();
  //mu_.unlock();
  return !a;
}

void ThreadGrid::mark_completed() {
  mu_.lock();
  completed_ = true;
  mu_.unlock();
}

bool ThreadGrid::is_completed() {
  mu_.lock();
  auto a = completed_;
  mu_.unlock();
  return a;
}

void ThreadGrid::mark_drawn() {
  mu_.lock();
  drawn_ = true;
  // grid_.clear();
  mu_.unlock();
}

bool ThreadGrid::is_drawn() {
  mu_.lock();
  auto a = drawn_;
  mu_.unlock();
  return a;
}

void ThreadGrid::clear() {
  mu_.lock();
  grid_.clear();
  drawn_ = false;
  completed_ = false;
  mu_.unlock();
}

vector<Point2D> ThreadGrid::reconstruct_path(Point2D start, Point2D goal) {
  if (!has(goal)) {
    goal = newest_.second;
    if (goal.x_ == -1) {
      return {};
    }
  }

  vector<Point2D> path;
  Point2D current = goal;
  mu_.lock();
  while (current != start) {
    path.push_back(current);
    current = grid_.at(current);
  }
  path.push_back(start);
  reverse(path.begin(), path.end());
  mu_.unlock();
  return path;
}

int ThreadGrid::path_cost(Point2D start, Point2D goal,
  unordered_map<Point2D, float>& cost_so_far) {
  auto path = reconstruct_path(start, goal);
  int cost = 0;
  for (const auto& s : path) {
    cost += cost_so_far.at(s);
  }
  return cost;
}

size_t ThreadGrid::size() { return grid_.size(); }

void ThreadGrid::set(LowresTile* k, LowresTile* v) {
  mu_.lock();
  lowres_map_[k] = v;
  newest_lr_ = make_pair(k, v);
  mu_.unlock();
}

LowresTile* ThreadGrid::get(LowresTile* k) {
  mu_.lock();
  auto a = lowres_map_.at(k);
  mu_.unlock();
  return a;
}

bool ThreadGrid::has(LowresTile* k) const {
  //mu_.lock();
  auto a = lowres_map_.find(k) == lowres_map_.end();
  //mu_.unlock();
  return !a;
}

unordered_set<LowresTile*> ThreadGrid::reconstruct_path(LowresTile* start,
  LowresTile* goal) {
  if (!has(goal)) {
    goal = newest_lr_.second;
    if (goal == nullptr) {
      return {};
    }
  }

  unordered_set<LowresTile*> path;
  LowresTile* current = goal;
  mu_.lock();
  while (current != start) {
    path.insert(current);
    current = lowres_map_.at(current);
  }
  path.insert(start);
  mu_.unlock();
  return path;
}

int ThreadGrid::path_cost(LowresTile* start, LowresTile* goal,
  unordered_map<LowresTile*, float>& cost_so_far) {
  auto path = reconstruct_path(start, goal);
  int cost = 0;
  for (const auto& s : path) {
    cost += cost_so_far.at(s);
  }
  return cost;
}

// size_t ThreadedGrid::size() const { return grid_.size(); }