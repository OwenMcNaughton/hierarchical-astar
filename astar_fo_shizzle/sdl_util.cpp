#include "sdl_util.h"

#include <string>

bool init(float scale) {
  bool success = true;
  if (SDL_Init(SDL_INIT_VIDEO) < 0) {
    printf("SDL could not initialize! SDL_Error: %s\n", SDL_GetError());
    success = false;
  } else {
    gWindow = SDL_CreateWindow("SDL Tutorial", 400, 0, SCREEN_WIDTH * scale,
                               SCREEN_HEIGHT * scale, SDL_WINDOW_SHOWN);
    if (gWindow == NULL) {
      printf("Window could not be created! SDL_Error: %s\n", SDL_GetError());
      success = false;
    } else {
      gScreenSurface = SDL_GetWindowSurface(gWindow);
      gRenderer = SDL_CreateRenderer(gWindow, -1, SDL_RENDERER_ACCELERATED);
      if (gRenderer == NULL) {
        printf("Renderer could not be created! SDL Error: %s\n",
               SDL_GetError());
        success = false;
      }
      SDL_SetRenderDrawColor(gRenderer, 0xFF, 0xFF, 0xFF, 0xFF);
      int imgFlags = IMG_INIT_PNG;
      if (!(IMG_Init(imgFlags) & imgFlags)) {
        printf("SDL_image could not initialize! SDL_image Error: %s\n",
               IMG_GetError());
        success = false;
      }
    }
  }

  return success;
}

SDL_Texture* loadTexture(string path) {
  SDL_Texture* newTexture = NULL;
  SDL_Surface* loadedSurface = IMG_Load(path.c_str());
  if (loadedSurface == NULL) {
    printf("Unable to load image %s! SDL_image Error: %s\n", path.c_str(),
           IMG_GetError());
  } else {
    newTexture = SDL_CreateTextureFromSurface(gRenderer, loadedSurface);
    if (newTexture == NULL) {
      printf("Unable to create texture from %s! SDL Error: %s\n", path.c_str(),
             SDL_GetError());
    }
    SDL_FreeSurface(loadedSurface);
  }

  return newTexture;
}

bool loadMedia() {
  bool success = true;
  bg = loadTexture("town.bmp");
  if (bg == NULL) {
    printf("Unable to load image %s! SDL Error: %s\n", "maze1024.bmp",
           SDL_GetError());
    success = false;
  }

  return success;
}

void blit(unordered_map<Point2D, float>& cost_so_far, ThreadGrid& came_from,
          Point2D start, Point2D end, float scale) {
  auto path = came_from.reconstruct_path(start, end);
  auto p = end;

  // if (path.empty()) {
  //  return;
  //}
  // if (path[path.size() - 1] != end) {
  //  return;
  //}

  // float max = -1999999999;
  // float min = 1999999999;
  // for (const auto& p : cost_so_far) {
  //  if (p.second > max) max = p.second;
  //  if (p.second < min) min = p.second;
  //}
  // for (const auto& p : cost_so_far) {
  //  float norm = (p.second - min) / (max - min) * 254;
  //  float norm2 = (p.second - min) / (max - min) * 100;
  //  SDL_SetRenderDrawColor(gRenderer, 0, 255 - norm, 140 - norm2, 1);
  //  SDL_Rect r = { p.first.x_ * scale, (X - p.first.y_) * scale - scale * 1,
  //                scale * 1, scale * 1 };
  //  SDL_RenderFillRect(gRenderer, &r);
  //}

  SDL_SetRenderDrawColor(gRenderer, 255, 0, 0, 255);
  for (const auto& step : path) {
    SDL_Rect r = {(step.x_ + 1) * scale, (X - step.y_ + 1) * scale - scale * 1,
                  scale * 1, scale * 1};
    SDL_RenderFillRect(gRenderer, &r);
  }

  // SDL_SetRenderDrawColor(gRenderer, 255, 0, 255, 255);
  // SDL_Rect r = {end.x_ * scale, (X - end.y_) * scale, scale * 2, scale * 2};
  // SDL_RenderFillRect(gRenderer, &r);

  if (came_from.is_completed()) {
    came_from.mark_drawn();
  }
  SDL_RenderPresent(gRenderer);
}

int log_normal(float v, float min, float max) {
  float a = (log10((v - min)) / (log10(max - min)));
  float b = a * (255 - 0);
  float c = b + 0;
  return c;
}

void render_traffic(const SquareGrid& graph, float scale) {
  float max = -1999999999;
  float min = 1999999999;
  for (const auto& p : graph.traffic_) {
    if (p.second > max) max = p.second;
    if (p.second < min) min = p.second;
  }
  for (const auto& p : graph.traffic_) {
    float norm = log_normal(p.second, min, max);
    SDL_SetRenderDrawColor(gRenderer, norm, norm, norm, 1);
    SDL_Rect r = {p.first.x_ * scale, (X - p.first.y_) * scale - scale * 1,
                  scale * 1, scale * 1};
    SDL_RenderFillRect(gRenderer, &r);
  }
  SDL_RenderPresent(gRenderer);
}

void blit(const SquareGrid& graph,
          vector<unordered_map<Point2D, float>>& cost_so_fars,
          vector<ThreadGrid>& came_froms, vector<Point2D>& starts,
          vector<Point2D>& ends, /*vector<Agent> agents,*/ mutex* mu,
          float scale) {
  SDL_Rect r = {1, 1, (X - 1) * scale, (X - 1) * scale};
  SDL_RenderCopy(gRenderer, bg, NULL, &r);
  SDL_RenderPresent(gRenderer);
  while (true) {
    Stopwatch sw;

    // render_traffic(graph, scale);

    int j = 0;
    mu->lock();
    for (int i = 0; i != cost_so_fars.size(); i++) {
      if (!came_froms[i].is_drawn()) {
        blit(cost_so_fars[i], came_froms[i], starts[i], ends[i], scale);
      }
      if (came_froms[i].is_drawn()) {
        cost_so_fars[i].clear();
        j++;
      }
    }
    if (j == cost_so_fars.size()) {
      mu->unlock();
      return;
    }

    mu->unlock();

    cout << "FRAME" << endl;

    double e = sw.elapsed();
    double sleep_for = 10 - e;
    if (e > 0) {
      this_thread::sleep_for(chrono::microseconds(int(sleep_for * 1000)));
    }
  }
}
