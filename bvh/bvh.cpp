#include <assert.h>

#include "bvh.h"

// todo:
// - use vector instead of array to allow resizing

// enable to validate the tree after every operation
#define VALIDATE 1


namespace {

// line segment aabb intersection test
bool raycast(float ax, float ay, float bx, float by, const bvh::aabb_t &aabb) {
  static const float EPSILON = 0.0001f;
  const float dx = (bx - ax) * .5f;
  const float dy = (by - ay) * .5f;
  const float ex = (aabb.maxx - aabb.minx) * .5f;
  const float ey = (aabb.maxy - aabb.miny) * .5f;
  const float cx = ax + dx - (aabb.minx + aabb.maxx) * .5f;
  const float cy = ay + dy - (aabb.miny + aabb.maxy) * .5f;
  const float adx = fabsf(dx);
  const float ady = fabsf(dy);
  if (fabsf(cx) > (ex + adx)) return false;
  if (fabsf(cy) > (ey + ady)) return false;
  return (fabsf(dx * cy - dy * cx) <= (ex * ady + ey * adx + EPSILON));
}

}  // namespace {}

namespace bvh {

bool aabb_t::raycast(float x0, float y0, float x1, float y1) const {
  return ::raycast(x0, y0, x1, y1, *this);
}

bvh_t::bvh_t()
  : growth(16.f)
  , _free_list(invalid_index)
  , _root(invalid_index)
{
  clear();
}

void bvh_t::clear() {
  _free_all();
  _root = invalid_index;
}

bool bvh_t::_is_leaf(index_t index) const {
  return get(index).is_leaf();
}

// return a quality metric for this subtree
float bvh_t::_quality(index_t n) const {
  if (n == invalid_index) {
    return 0.f;
  }
  const node_t &node = _get(n);
  if (node.is_leaf()) {
    return 0.f;
  }
  // cost metric is the sum of the surface area of all interior nodes (non root or leaf)
  return ((n == _root) ? 0.f : node.aabb.area()) +
    _quality(node.child[0]) +
    _quality(node.child[1]);
}

index_t bvh_t::_insert_into_leaf(index_t leaf, index_t node) {
  assert(_is_leaf(leaf));
  // create new internal node
  index_t inter = _new_node();
  // insert children
  _get(inter).child[0] = leaf;
  _get(inter).child[1] = node;
  // keep track of the parents
  _get(inter).parent = invalid_index;  // fixed up by callee
  _get(leaf).parent = inter;
  _get(node).parent = inter;
  // recalculate the aabb on way up
  _get(inter).aabb = aabb_t::find_union(_get(leaf).aabb, _get(node).aabb);
  // new child is the intermediate node
  return inter;
}

index_t bvh_t::insert(const aabb_t &aabb, void *user_data) {
  // create the new node
  index_t index = _new_node();
  assert(index != invalid_index);
  auto &node = _get(index);
  // grow the aabb by a factor
  node.aabb = aabb_t::grow(aabb, growth);
  // 
  node.user_data = user_data;
  node.parent = invalid_index;
  // mark that this is a leaf
  node.child[0] = invalid_index;
  node.child[1] = invalid_index;
  // insert into the tree
  if (_root == invalid_index) {
    _root = index;
    return index;
  }
  if (_is_leaf(_root)) {
    _root = _insert_into_leaf(_root, index);
    _validate(_root);
    return index;
  }
  _insert(index);
#if VALIDATE
  _validate(_root);
#endif
  // return this index
  return index;
}

void bvh_t::_recalc_aabbs(index_t i) {
  // walk from leaf to root recalculating aabbs
  while (i != invalid_index) {
    node_t &y = _get(i);
    y.aabb = aabb_t::find_union(_get(y.child[0]).aabb, _get(y.child[1]).aabb);
    _optimize(y);
    i = y.parent;
  }
}

index_t bvh_t::_find_best_sibling(const aabb_t &aabb) const {
  struct search_t { index_t index; float cost; };
  std::vector<search_t> stack;

  // XXX: can be optimized with a priority queue

  // XXX: could also be optimized by node replacement rather then poping
  // leading to less resizing

  if (_root != invalid_index) {
    stack.push_back(search_t{ _root, 0.f });
  }

  float best_cost = INFINITY;
  index_t best_index = invalid_index;

  while (!stack.empty()) {
    // pop one node
    const search_t s = stack.back();
    const node_t &n = _get(s.index);
    stack.resize(stack.size() - 1);
    // find the cost for inserting into this node
    const aabb_t uni = aabb_t::find_union(aabb, n.aabb);
    const float growth = uni.area() - n.aabb.area();
    assert(growth >= 0.f);
    const float cost = s.cost + growth;
    // skip the entire subtree if we know we have a better choice
    if (cost >= best_cost) {
      continue;
    }
    if (n.is_leaf()) {
      best_cost = cost;
      best_index = s.index;
    }
    else {
      assert(n.child[0] != invalid_index);
      assert(n.child[1] != invalid_index);
      stack.push_back(search_t{ n.child[0], cost });
      stack.push_back(search_t{ n.child[1], cost });
    }
  }
  // return the best leaf we cound find
  return best_index;
}

void bvh_t::_insert(index_t node) {

  // note: this insert phase assumes that there are at least two nodes already
  //       in the tree and thus _root is a non leaf node.

  // find the best sibling for node
  const aabb_t &aabb = _get(node).aabb;
  index_t sibi = _find_best_sibling(aabb);
  assert(sibi != invalid_index);
  // once the best leaf has been found we come to the insertion phase
  const node_t &sib = _get(sibi);
  assert(sib.is_leaf());
  const index_t parent = sib.parent;
  const index_t inter = _insert_into_leaf(sibi, node);
  _get(inter).parent = parent;
  // fix up parent child relationship
  if (parent != invalid_index) {
    node_t &p = _get(parent);
    p.replace_child(sibi, inter);
  }
  // recalculate aabb and optimize on the way up
  _recalc_aabbs(parent);
}

void bvh_t::remove(index_t index) {
  assert(index != invalid_index);
  assert(_is_leaf(index));
  auto &node = _get(index);
  _unlink(index);
  node.child[0] = _free_list;
  node.child[1] = invalid_index;
  _free_list = index;
#if VALIDATE
  _validate(_root);
#endif
}

void bvh_t::move(index_t index, const aabb_t &aabb) {
  assert(index != invalid_index);
  assert(_is_leaf(index));
  auto &node = _get(index);
  // check fat aabb against slim new aabb for hysteresis on our updates
  if (node.aabb.contains(aabb)) {
    // this is okay and we can early exit
    return;
  }
  // effectively remove this node from the tree
  _unlink(index);
  // save the fat version of this aabb
  node.aabb = aabb_t::grow(aabb, growth);
  // insert into the tree
  if (_root == invalid_index) {
    _root = index;
    return;
  }
  if (_is_leaf(_root)) {
    _root = _insert_into_leaf(_root, index);
  }
  _insert(index);
#if VALIDATE
  _validate(_root);
#endif
}

void bvh_t::_free_all() {
  _free_list = 0;
  for (index_t i = 0; i < _max_nodes; ++i) {
    _get(i).child[0] = i + 1;
    _get(i).child[1] = invalid_index;
    _get(i).parent = invalid_index;
  }
  // mark the end of the list of free nodes
  _get(_max_nodes - 1).child[0] = invalid_index;
  _get(_max_nodes - 1).child[1] = invalid_index;
  _get(_max_nodes - 1).parent = invalid_index;
  _root = invalid_index;
}

index_t bvh_t::_new_node() {
  index_t out = _free_list;
  assert(out != invalid_index);
  _free_list = get(_free_list).child[0];
  return out;
}

void bvh_t::_unlink(index_t index) {
  // assume we only ever unlink a leaf
  assert(_is_leaf(index));
  auto &node = _get(index);

  // special case root node
  if (index == _root) {
    assert(node.parent == invalid_index);
    // case 1 (node was root)
    _root = invalid_index;
    return;
  }

  auto &p0 = _get(node.parent);

  if (p0.parent == invalid_index) {

    // case 2 (p0 is root)
    //       P0
    //  node     ?

    // find the sibling node
    index_t sib = (p0.child[0] == index) ? 1 : 0;
    // promote as the new root
    _root = p0.child[sib];
    _get(_root).parent = invalid_index;
    // free p0 node
    _free_node(node.parent);
    node.parent = invalid_index;
    return;
  }

  auto &p1 = _get(p0.parent);

  // case 3 (p0 is not root)
  //              p1
  //       p0
  //  node     ?

  // find the configuration of p1 children
  // find child index of p0 for p1
  index_t p1c = (p1.child[0] == node.parent) ? 0 : 1;
  assert(p1.child[p1c] == node.parent);

  // find the configuration of p0 children
  // find child index of 'node' for p0
  index_t p0c = (p0.child[0] == index) ? 0 : 1;
  assert(p0.child[p0c] == index);

  // promote sibling of 'node' into place of p0
  // find sibling of 'node'
  index_t sib = p0.child[p0c ^ 1];
  assert(sib != invalid_index);
  // promote sibling of 'node' into place of p0
  p1.child[p1c] = sib;
  _get(sib).parent = p0.parent;

  // recalculate the remaining tree aabbs, p1 up
  _touched_aabb(p0.parent);

  // free p0
  _free_node(node.parent);
  // node is now unlinked
  node.parent = invalid_index;
}

void bvh_t::_touched_aabb(index_t i) {
  while (i != invalid_index) {
    node_t &node = _get(i);
    assert(!node.is_leaf());
    assert(node.child[0] != invalid_index);
    assert(node.child[1] != invalid_index);
    node.aabb = aabb_t::find_union(_child(i, 0).aabb, _child(i, 1).aabb);
    i = node.parent;
  }
}

void bvh_t::_free_node(index_t index) {
  assert(index != invalid_index);
  auto &node = _get(index);
  node.child[0] = _free_list;
  _free_list = index;
}

void bvh_t::_validate(index_t index) {
  if (index == invalid_index) {
    return;
  }

  auto &node = _get(index);

  if (index == _root) {
    assert(_get(index).parent == invalid_index);
  }
  // check parents children
  if (node.parent != invalid_index) {
    auto &parent = _get(node.parent);
    assert(parent.child[0] != parent.child[1]);
    assert(parent.child[0] == index ||
           parent.child[1] == index);
  }
  if (_is_leaf(index)) {
    // leaves should have no children
    assert(node.child[0] == invalid_index);
    assert(node.child[1] == invalid_index);
  }
  else {
    // interior nodes should have two children
    assert(node.child[0] != invalid_index);
    assert(node.child[1] != invalid_index);
    // validate childrens parent indices
    assert(_child(index, 0).parent == index);
    assert(_child(index, 1).parent == index);
    // validate aabbs
    assert(node.aabb.contains(_child(index, 0).aabb));
    assert(node.aabb.contains(_child(index, 1).aabb));
    // validate child nodes
    _validate(node.child[0]);
    _validate(node.child[1]);

    // XXX: check area of aabb is minimal?
  }
}

void bvh_t::optimize() {
  _optimize(_root);
#if VALIDATE
  _validate(_root);
#endif
}

void bvh_t::_optimize(index_t i) {
  if (i == invalid_index) {
    return;
  }
  node_t &n = _get(i);
  // random walk down the tree
  if (rand() & 0x80) {
    _optimize(n.child[0]);
  }
  else {
    _optimize(n.child[1]);
  }
  _optimize(n);
}

void bvh_t::_optimize(node_t &node) {

#if VALIDATE
  const float before = quality();
#endif

  // four possible rotations
  //
  //        n                   n
  //   c0       c1         c0      *x0
  // x0  x1             *c1  x1
  //
  //        n                   n
  //   c0       c1         c0      *x1
  // x0  x1              x0 *c1
  //
  // same as above but with c0 and c1 roles swapped

  for (int i = 0; i < 2; ++i) {

    // gen 1
    const auto c0i = node.child[0];
    const auto c1i = node.child[1];
    if (c0i == invalid_index || c1i == invalid_index) {
      return;
    }
    auto &c0 = _get(c0i);
    auto &c1 = _get(c1i);

    // gen 2
    const auto x0i = c0.child[0];
    const auto x1i = c0.child[1];
    if (x0i == invalid_index || x1i == invalid_index) {
      // swap c0 and c1 and try again (rotations 3 and 4)
      std::swap(node.child[0], node.child[1]);
      continue;
    }
    auto &x0 = _get(x0i);
    auto &x1 = _get(x1i);

    // note: this heuristic only looks at minimizing the area of c0
    //       which is all that is affected by these rotations

    // current
    const float h0 = c0.aabb.area();
    // rotation 1
    const aabb_t a1 = aabb_t::find_union(c1.aabb, x1.aabb);
    const float h1 = a1.area();
    // rotation 2
    const aabb_t a2 = aabb_t::find_union(x0.aabb, c1.aabb);
    const float h2 = a2.area();

    if (h1 < h2) {
      if (h1 < h0) {
        // do rotation 1 (swap x0/c1)
        //        n                   n
        //   c0       c1  ->     c0      *x0
        // x0  x1             *c1  x1
        //
        c1.parent = c0i;
        x0.parent = c0.parent;
        std::swap(c0.child[0], node.child[1]); // x0 and c1
        c0.aabb = a1;
        assert(c0.aabb.contains(c1.aabb));
        assert(c0.aabb.contains(x1.aabb));
      }
    } else {
      if (h2 < h0) {
        // do rotation 2 (swap x1/c1)
        //        n                   n
        //   c0       c1  ->     c0      *x1
        // x0  x1              x0 *c1
        //
        c1.parent = c0i;
        x1.parent = c0.parent;
        std::swap(c0.child[1], node.child[1]); // x1 and c1
        c0.aabb = a2;
        assert(c0.aabb.contains(x0.aabb));
        assert(c0.aabb.contains(c1.aabb));
      }
    }
    // swap c0 and c1 and try again (rotations 3 and 4)
    std::swap(node.child[0], node.child[1]);
  }

#if VALIDATE
  const float after = quality();
  assert(after - 1.f <= before);
#endif
}

void bvh_t::find_overlaps(const aabb_t &bb, std::vector<index_t> &overlaps) {
  std::vector<index_t> stack;
  if (_root != invalid_index) {
    stack.push_back(_root);
  }
  while (!stack.empty()) {
    // pop one node
    const index_t ni = stack.back();
    assert(ni != invalid_index);
    const node_t &n = _get(ni);

    // XXX: we can not resize here, and instead replace the poped node later instead
    // of a push_back as a little optimisation

    stack.resize(stack.size() - 1);
    // if these aabbs overlap
    if (aabb_t::overlaps(bb, n.aabb)) {
      if (n.is_leaf()) {
        overlaps.push_back(ni);
      }
      else {
        assert(n.child[0] != invalid_index);
        assert(n.child[1] != invalid_index);
        stack.push_back(n.child[0]);
        stack.push_back(n.child[1]);
      }
    }
  }
}

void bvh_t::find_overlaps(index_t node, std::vector<index_t> &overlaps) {
  const node_t &n = _get(node);
  find_overlaps(n.aabb, overlaps);
}

void bvh_t::raycast(float x0, float y0, float x1, float y1,
                    std::vector<index_t> &overlaps) {
  std::vector<index_t> stack;
  if (_root != invalid_index) {
    stack.push_back(_root);
  }
  while (!stack.empty()) {
    // pop one node
    const index_t ni = stack.back();
    assert(ni != invalid_index);
    const node_t &n = _get(ni);
    stack.resize(stack.size() - 1);
    // if the ray and aabb overlap
    if (::raycast(x0, y0, x1, y1, n.aabb)) {
      if (n.is_leaf()) {
        overlaps.push_back(ni);
      }
      else {
        assert(n.child[0] != invalid_index);
        assert(n.child[1] != invalid_index);
        stack.push_back(n.child[0]);
        stack.push_back(n.child[1]);
      }
    }
  }
}

} // namespace bvh
