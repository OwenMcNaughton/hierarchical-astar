#ifndef SDL_UTIL_H
#define SDL_UTIL_H

#include <SDL.h>
#include <SDL_image.h>

#include "agent.h"
#include "pathfinding.h"
#include "util.h"

const int SCREEN_WIDTH = X;
const int SCREEN_HEIGHT = X;
SDL_Window* gWindow = NULL;
SDL_Surface* gScreenSurface = NULL;
SDL_Texture* bg = NULL;
SDL_Renderer* gRenderer = NULL;

bool init(float scale = 1);

SDL_Texture* loadTexture(string path);

bool loadMedia();

void blit(unordered_map<Point2D, float>& cost_so_far, ThreadGrid& came_from,
          Point2D start, Point2D end, float scale = 1);

void blit(const SquareGrid& graph,
          vector<unordered_map<Point2D, float>>& cost_so_fars,
          vector<ThreadGrid>& came_froms, vector<Point2D>& starts,
          vector<Point2D>& ends, /*vector<Agent> agents,*/ mutex* mu,
          float scale = 1);

#endif