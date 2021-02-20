#include <assert.h>

#include "bvh.h"

// todo:
// - use vector instead of array to allow resizing
// - add node rotations
// - resolve all overlapping pairs

#define VALIDATE 1

namespace bvh {

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

index_t bvh_t::insert(const aabb_t &aabb, void *user_data) {

  // 1. create a node
  //    . calc fat aabb
  // 2. start at root node
  //    . bubble down to leaf minimising node area

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
  _root = (_root == invalid_index) ? index : _insert(_root, index);
  _get(_root).parent = invalid_index;
#if VALIDATE
  _validate(_root);
#endif
  // return this index
  return index;
}

// insert 'node' into 'into'
index_t bvh_t::_insert(index_t into, index_t node) {
  assert(into != invalid_index);
  assert(node != invalid_index);
  // if we are inserting into a leaf
  if (_is_leaf(into)) {
    // create new internal node
    index_t inter = _new_node();
    // insert children
    _get(inter).child[0] = into;
    _get(inter).child[1] = node;
    // keep track of the parents
    _get(inter).parent = invalid_index;  // fixed up by callee
    _get(into).parent = inter;
    _get(node).parent = inter;
    // recalculate the aabb on way up
    _get(inter).aabb = aabb_t::find_union(_get(into).aabb, _get(node).aabb);
    // new child is the intermediate node
    return inter;
  }
  else {
    // this node should have two children already
    assert(_get(into).child[0] != invalid_index &&
           _get(into).child[1] != invalid_index);
    {
      // calculate new area if inserted into each child
      // child 0 + node
      const auto &aabb_e0 = aabb_t::find_union(_child(into, 0).aabb, _get(node).aabb);
      // child 1 + node
      const auto &aabb_e1 = aabb_t::find_union(_child(into, 1).aabb, _get(node).aabb);
      // insert to minimise overall surface area
      // (child 0 + node) + (child 1)
      const float sah_0 = aabb_e0.area() + _child(into, 1).aabb.area();
      // (child 0) + (node + child 1)
      const float sah_1 = aabb_e1.area() + _child(into, 0).aabb.area();
      if (sah_0 <= sah_1) {
        _get(into).child[0] = _insert(_get(into).child[0], node);
        _child(into, 0).parent = into;
      }
      else {
        _get(into).child[1] = _insert(_get(into).child[1], node);
        _child(into, 1).parent = into;
      }
    }
    // recalculate the parent aabb on way up
    _get(into).aabb = aabb_t::find_union(_child(into, 0).aabb,
                                         _child(into, 1).aabb);
    // no intermediate (return original)
    return into;
  }
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
  _root = (_root == invalid_index) ? index : _insert(_root, index);
  _get(_root).parent = invalid_index;
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

  // free p0
  _free_node(node.parent);
  // node is now unlinked
  node.parent = invalid_index;
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
  }
}

void bvh_t::optimize() {
  _optimize(_root);
}

void bvh_t::_optimize(index_t i) {
  if (i == invalid_index) {
    return;
  }
  node_t &n = _get(i);
#if 0
  // optimize all nodes in the tree
  if (n.child[0] != invalid_index) {
    _optimize(n.child[0]);
  }
  if (n.child[1] != invalid_index) {
    _optimize(n.child[1]);
  }
#else
  // random walk down the tree
  if (rand() & 0x80) {
    _optimize(n.child[0]);
  }
  else {
    _optimize(n.child[1]);
  }
#endif
  _optimize(n);
}

void bvh_t::_optimize(node_t &node) {

  // config a (start):
  //
  //        n
  //   c0       c1
  // x0  x1   x2  x3
  //
  // config b:                config c:
  //
  //        n                        n
  //   c0       c1              c0       c1
  // x0  x2   x1  x3          x0  x3   x2  x1

  // gen 1
  const auto c0i = node.child[0];
  const auto c1i = node.child[1];
  if (c0i == invalid_index ||
      c1i == invalid_index) {
    return;
  }
  auto &c0 = _get(node.child[0]);
  auto &c1 = _get(node.child[1]);

  // gen 2
  const auto x0i = c0.child[0];
  const auto x1i = c0.child[1];
  const auto x2i = c1.child[0];
  const auto x3i = c1.child[1];
  if (x0i == invalid_index ||
      x1i == invalid_index ||
      x2i == invalid_index ||
      x3i == invalid_index) {
    return;
  }
  auto &x0 = _get(c0.child[0]);
  auto &x1 = _get(c0.child[1]);
  auto &x2 = _get(c1.child[0]);
  auto &x3 = _get(c1.child[1]);

  // original
  const float a = aabb_t::find_union(x0.aabb, x1.aabb).area() +
                  aabb_t::find_union(x2.aabb, x3.aabb).area();
  // configuration b
  const float b = aabb_t::find_union(x0.aabb, x2.aabb).area() +
                  aabb_t::find_union(x1.aabb, x3.aabb).area();
  // configuration c
  const float c = aabb_t::find_union(x0.aabb, x3.aabb).area() + 
                  aabb_t::find_union(x2.aabb, x1.aabb).area();

  bool recalc = false;

  if (b < c && b < a) {
    // rotate to config b
    _get(c0.child[1]).parent = c1i;
    _get(c1.child[0]).parent = c0i;
    std::swap(c0.child[1], c1.child[0]);
    // recalculate aabb
    c0.aabb = aabb_t::find_union(x0.aabb, x2.aabb);
    c1.aabb = aabb_t::find_union(x1.aabb, x3.aabb);
    recalc = true;
  }
  if (c < b && c < a) {
    // rotate to config c
    _get(c0.child[1]).parent = c1i;
    _get(c1.child[1]).parent = c0i;
    std::swap(c0.child[1], c1.child[1]);
    // recalculate aabb
    c0.aabb = aabb_t::find_union(x0.aabb, x3.aabb);
    c1.aabb = aabb_t::find_union(x2.aabb, x1.aabb);
    recalc = true;
  }

  // if we transformed the nodes, work back up the tree recomputing the aabbs
  // until we hit the root node
  if (recalc) {
    index_t n = c0.parent;
    while (n != invalid_index) {
      node_t &x = _get(n);
      // recalculate aabb
      x.aabb = aabb_t::find_union(
        _get(x.child[0]).aabb,
        _get(x.child[1]).aabb);
      // move up the tree
      n = x.parent;
    }
  }
}

// find all overlaps with a given bounding-box
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

// find all overlaps with a given node
void bvh_t::find_overlaps(index_t node, std::vector<index_t> &overlaps) {
}

} // namespace bvh
