
#ifndef UTIL_H
#define UTIL_H

#include <array>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <queue>
#include <shared_mutex>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace std;

const uint64_t X = 512;

uint64_t getUnixTimeStamp();

class Stopwatch {
  bool is_running() const;
  chrono::high_resolution_clock::time_point end_time() const;

  chrono::high_resolution_clock::time_point begin_time_{
      chrono::high_resolution_clock::now() };
  chrono::high_resolution_clock::time_point stop_time_{
      chrono::high_resolution_clock::time_point::min() };

public:
  void stop();
  double elapsed() const;
};

struct Point2D {
  Point2D() {}
  Point2D(int16_t x, int16_t y) : x_(x), y_(y) {}

  int16_t x_ = -1, y_ = -1, arb_;
};

struct FPoint2D {
  FPoint2D() {}
  FPoint2D(float x, float y) : x_(x), y_(y) {}

  float x_ = -1, y_ = -1, arb_;
};

FPoint2D operator+(FPoint2D a, FPoint2D b) {
  try {
    return FPoint2D(a.x_ + b.x_, a.y_ + b.y_);
  } catch (const std::exception& ex) {
    cout << ex.what() << endl;
  }
}

namespace std {
  template <>
  struct hash<Point2D> {
    typedef Point2D argument_type;
    typedef size_t result_type;
    size_t operator()(const Point2D& id) const noexcept {
      try {
        return hash<int>()(id.x_ ^ (id.y_ << 4));
      }
      catch (const std::exception& ex) {
        cout << ex.what() << endl;
      }
    }
  };
}  // namespace std

bool operator==(Point2D a, Point2D b) {
  try {
    return a.x_ == b.x_ && a.y_ == b.y_;
  }
  catch (const std::exception& ex) {
    cout << ex.what() << endl;
  }
}

bool operator!=(Point2D a, Point2D b) {
  try {
    return !(a == b);
  }
  catch (const std::exception& ex) {
    cout << ex.what() << endl;
  }
}

bool operator<(Point2D a, Point2D b) {
  return tie(a.x_, a.y_) < tie(b.x_, b.y_);
}

basic_iostream<char>::basic_ostream& operator<<(
  basic_iostream<char>::basic_ostream& out, const Point2D& loc) {
  out << '(' << loc.x_ << ',' << loc.y_ << ')';
  return out;
}

template <typename T, typename priority_t>
struct PriorityQueue {
  typedef pair<priority_t, T> PQElement;
  priority_queue<PQElement, vector<PQElement>, greater<PQElement>> elements;

  inline bool empty() const { return elements.empty(); }

  inline void put(T item, priority_t priority) {
    elements.emplace(priority, item);
  }

  T get() {
    T best_item = elements.top().second;
    elements.pop();
    return best_item;
  }
};

struct LowresTile {
  LowresTile() {
    centroid_.x_ = 0;
    centroid_.y_ = 0;
  }

  float avg_weight_;
  int resolution_;
  Point2D centroid_;
  unordered_set<Point2D> territory_;
  mutex mu_;
  bool occupied_ = false;
};

class ThreadGrid {
public:
  void set(const Point2D& k, const Point2D& v);

  Point2D get(const Point2D& k);

  bool has(const Point2D& k) const;

  vector<Point2D> reconstruct_path(Point2D start, Point2D goal);

  int path_cost(Point2D start, Point2D goal,
    unordered_map<Point2D, float>& cost_so_far);

  void set(LowresTile* k, LowresTile* v);

  LowresTile* get(LowresTile* k);

  bool has(LowresTile* k) const;

  unordered_set<LowresTile*> reconstruct_path(LowresTile* start,
    LowresTile* goal);

  int path_cost(LowresTile* start, LowresTile* goal,
    unordered_map<LowresTile*, float>& cost_so_far);

  bool empty();

  void mark_completed();

  bool is_completed();

  void mark_drawn();

  bool is_drawn();

  void clear();

  size_t size();

private:
  shared_mutex mu_;
  bool completed_ = false;
  bool drawn_ = false;
  pair<Point2D, Point2D> newest_;
  unordered_map<Point2D, Point2D> grid_;
  pair<LowresTile*, LowresTile*> newest_lr_;
  unordered_map<LowresTile*, LowresTile*> lowres_map_;
};

#endif