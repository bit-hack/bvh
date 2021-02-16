#include <vector>
#include <cstdlib>

#include "bvh.h"

float randf() {
  return float(rand()) / float(RAND_MAX);
}

bvh::aabb_t rand_aabb() {
  bvh::aabb_t out;
  out.minx = randf() * 1024.f;
  out.miny = randf() * 1024.f;
  out.maxx = out.minx + randf() * 256.f;
  out.maxy = out.miny + randf() * 256.f;
  return out;
}

bvh::aabb_t move_aabb(const bvh::aabb_t &in) {
  bvh::aabb_t out = in;
  const float dx = randf() * 64 - 32;
  const float dy = randf() * 64 - 32;
  out.minx += dx;
  out.miny += dy;
  out.maxx += dx;
  out.maxy += dy;
  return out;
}

bvh::bvh_t uut;

int main(int argc, char **args) {

  std::vector<bvh::index_t> indices;

  for (int i = 0; i < 1000000; ++i) {

    switch (rand() % 4) {
    case 0:
      if (indices.size() < 256) {
        bvh::aabb_t o = rand_aabb();
        bvh::index_t index = uut.insert(o, nullptr);
        indices.push_back(index);
        break;
      }
    case 1:
      if (!indices.empty() && indices.size() > 64) {
        auto itt = indices.begin() + (rand() % indices.size());
        uut.remove(*itt);
        indices.erase(itt);
      }
      break;
    case 2:
    case 3:
#if 1
      if (!indices.empty()) {
        auto itt = indices.begin() + (rand() % indices.size());
        auto obj = uut.get(*itt);
        uut.move(*itt, move_aabb(obj.aabb));
      }
#endif
      break;
    }
  }

  return 0;
}
