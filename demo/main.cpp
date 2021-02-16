#include <vector>
#include <cstdlib>
#include <array>

#define _SDL_main_h
#include <SDL.h>

#include "../bvh/bvh.h"


SDL_Surface *screen = nullptr;
bvh::bvh_t uut;

static inline float randf() {
  return float(rand()) / float(RAND_MAX);
}

static inline float randf(float val) {
  return val * float(rand()) / float(RAND_MAX);
}

struct bounce_t {
  float x, y;
  float dx, dy;
  float size;

  void make() {
    x = randf(512);
    y = randf(512);
    dx = (1.f - randf() * 2.f) * 0.1f;
    dy = (1.f - randf() * 2.f) * 0.1f;
    size = randf(32);
  }

  void tick() {
    if (x < 0   && dx < 0) dx *= -1.f;
    if (x > 512 && dx > 0) dx *= -1.f;
    if (y < 0   && dy < 0) dy *= -1.f;
    if (y > 512 && dy > 0) dy *= -1.f;
    x += dx;
    y += dy;
  }

  const bvh::aabb_t aabb() const {
    return bvh::aabb_t{
      x - size, y - size,
      x + size, y + size
    };
  }
};

void plot(int x, int y, uint32_t rgb) {
  if (x < 0 || y < 0 || x >= 512 || y >= 512) {
    return;
  }
  uint32_t *pix = (uint32_t *)screen->pixels;
  pix[x + y * 512] = rgb;
}

void rect(const bvh::aabb_t &a, uint32_t rgb) {
  const int xl = (int)a.minx;
  const int xh = (int)a.maxx;
  const int yl = (int)a.miny;
  const int yh = (int)a.maxy;
  for (int x = xl; x < xh; ++x) {
    plot(x, yl, rgb);
    plot(x, yh, rgb);
  }
  for (int y = yl; y < yh; ++y) {
    plot(xl, y, rgb);
    plot(xh, y, rgb);
  }
}

void draw_node(const bvh::node_t &node) {
  if (node.user_data) {
    const bounce_t *b = (bounce_t*)node.user_data;
    const bvh::aabb_t c = b->aabb();
    rect(c, 0xff0000);
  }
  if (node.child[0] == bvh::invalid_index) {
    rect(node.aabb, 0x00ff00);
  }
  else {
    rect(node.aabb, 0x808080);
  }
  if (node.child[0] != bvh::invalid_index) {
    const auto &a = uut.get(node.child[0]);
    draw_node(a);
  }
  if (node.child[1] != bvh::invalid_index) {
    const auto &b = uut.get(node.child[1]);
    draw_node(b);
  }
}

void move_volume(bvh::index_t vol) {
  const auto &obj = uut.get(vol);
  bounce_t *b = (bounce_t*)uut.user_data(vol);
  b->tick();
  uut.move(vol, b->aabb());
}

int main(int argc, char **args) {

  if (SDL_Init(SDL_INIT_VIDEO) != 0) {
    return 1;
  }

  screen = SDL_SetVideoMode(512, 512, 32, 0);
  if (!screen) {
    return 1;
  }

  std::array<bvh::index_t, 16> indices;
  std::array<bounce_t, 16> bounce;
  for (int i = 0; i < 16; ++i) {
    bounce[i].make();
    indices[i] = uut.insert(bounce[i].aabb(), &bounce[i]);
  }

  bool active = true;
  while (active) {

    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      if (event.type == SDL_QUIT) {
        active = false;
      }
    }

    SDL_FillRect(screen, nullptr, 0x101010);

    for (const bvh::index_t i : indices) {
      move_volume(i);
    }

    // draw the bvh
    if (!uut.empty()) {
      const auto &node = uut.root();
      draw_node(node);
    }

    SDL_Flip(screen);
    SDL_Delay(1);
  }

  return 0;
}