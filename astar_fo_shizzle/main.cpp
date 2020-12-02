
#include <numeric>
#include <string>

#include "agent.h"
#include "bmp.h"
#include "pathfinding.h"
#include "sdl_util.h"
#include "threadpool.h"
#include "util.h"

using namespace std;

namespace std {
template <>
struct hash<pair<Point2D, Point2D>> {
  typedef Point2D argument_type;
  typedef size_t result_type;
  size_t operator()(const pair<Point2D, Point2D>& id) const noexcept {
    try {
      return hash<int>()(id.first.x_ ^ (id.first.y_ << 4) + id.first.x_ ^
                         (id.first.y_ << 4));
    } catch (const std::exception& ex) {
      cout << ex.what() << endl;
    }
  }
};
}  // namespace std

// this is the only comment in this repo.
// this is a comment to tell you that this the code in this repo is awful and
// contains no comments that can explain the awfulness.
int main(int argc, char* argv[]) {
  float scale = 1;

  if (!init(scale)) {
    printf("failed to initialize!\n");
  } else {
    if (!loadMedia()) {
      printf("failed to load media!\n");
    }
  }

  int j = getUnixTimeStamp();
  for (int iter = 0; iter != 1; iter++) {
    const int N = 1000;
    vector<ThreadGrid> came_from(N);
    vector<unordered_map<Point2D, float>> cost_so_far(N);
    vector<Point2D> ends(N);
    vector<Point2D> starts(N);
    vector<unordered_map<int, double>> es(N);
    vector<int> reses = {16, 8, 4, 2};
    SquareGrid grid(1, reses);

    // Agent a(&grid);
    // vector<Agent> agents;

    mutex* mu = new mutex();

    auto t1 = thread([&]() {
      blit(grid, cost_so_far, came_from, starts, ends, /*agents,*/ mu, scale);
    });

    // a.find_start();
    // a.find_goal();
    // a.find_path();
    // float dt = 0;
    // while (true) {
    //  Stopwatch msw;
    //  a.update_pos(10);
    //  a.render(gRenderer, scale);
    //}

    Stopwatch msw;
    {
      ThreadPool pool(10);

      for (uint64_t i = 0; i != N; i++) {
        // cout << i << endl;

        pool.enqueue([&, i]() {
          auto p = grid.get_rand_startend();
          Point2D s = p.first;
          Point2D e = p.second;

          starts[i].x_ = s.x_;
          starts[i].y_ = s.y_;
          ends[i].x_ = e.x_;
          ends[i].y_ = e.y_;

          int r;
          LowresTile *lowres_s, *lowres_e;
          unordered_set<LowresTile*> lrp = {};

          Stopwatch msw;
          for (int j = 0; j != reses.size(); j++) {
            Stopwatch sw;
            r = reses[j];
            lowres_s = grid.territories_[r][s];
            lowres_e = grid.territories_[r][e];
            shared_ptr<ThreadGrid> cf = make_shared<ThreadGrid>();
            unordered_map<LowresTile*, float> csf;
            a_star_search(grid, lowres_s, lowres_e, cf, csf, lrp,
                          grid.territory_adj_[r]);
            lrp = cf->reconstruct_path(lowres_s, lowres_e);
            es[i].emplace(r, sw.elapsed());
          }

          Stopwatch sw;
          a_star_search(grid, s, e, came_from[i], cost_so_far[i], lrp,
                        grid.territories_[r]);
          auto path = came_from[i].reconstruct_path(s, e);
          // cout << path.size() << endl;
          es[i].emplace(1, sw.elapsed());
          es[i].emplace(0, msw.elapsed());
          grid.record_path_traffic(
              path, came_from[i].path_cost(s, e, cost_so_far[i]));
        });

        while (true) {
          if (pool.all_workers_busy()) {
            this_thread::sleep_for(1ms);
          } else {
            break;
          }
        }
      }
    }

    cout << msw.elapsed() << endl;
    cout << "summarize([";
    // cout << "[";
    for (auto e : es) {
      cout << e[0] << ",";
    }
    cout << "])" << endl;
    t1.join();
    mu->lock();
    // came_from.clear();
    // cost_so_far.clear();
    mu->unlock();
    delete mu;
  }
  this_thread::sleep_for(100s);

  return 1;
}