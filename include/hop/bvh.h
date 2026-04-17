#pragma once

#include <hop/math/aa_box.h>
#include <hop/math/intersect.h>
#include <hop/scalar_traits.h>

#include <algorithm>
#include <utility>
#include <vector>

namespace hop {

// A BVH (Bounding Volume Hierarchy) for spatial acceleration of AABB queries.
//
// Template parameters:
//   T    — scalar type (float or fixed16)
//   Item — payload stored at leaf nodes (e.g., int index, pointer)
//
// Usage:
//   bvh<float, int> tree;
//   std::vector<std::pair<aa_box<float>, int>> entries = ...;
//   tree.build(entries);
//   tree.query_aabb(box, [](int item) { ... });
//   tree.query_ray(origin, direction, [](int item, float &best_t) { ... });

template <typename T, typename Item> class bvh {
public:
	using tr = scalar_traits<T>;

	struct node {
		aa_box<T> box;
		int left = -1;
		int right = -1;
		Item item {};
		bool is_leaf() const { return left == -1 && right == -1; }
	};

	// Build from a list of (AABB, item) pairs. The input vector may be reordered.
	void build(std::vector<std::pair<aa_box<T>, Item>> & entries) {
		nodes_.clear();
		if (entries.empty())
			return;
		nodes_.reserve(entries.size() * 2);
		build_recursive(entries, 0, static_cast<int>(entries.size()));
	}

	// Find all items whose AABBs overlap the given box.
	template <typename Callback> void query_aabb(const aa_box<T> & box, Callback && cb) const {
		if (nodes_.empty())
			return;
		query_aabb_recursive(0, box, cb);
	}

	// Find items along a ray (segment). The callback receives (item, best_t)
	// and should update best_t if it finds a closer hit, enabling early pruning.
	template <typename Callback>
	void query_ray(const vec3<T> & origin, const vec3<T> & direction, Callback && cb) const {
		if (nodes_.empty())
			return;
		T best_t = tr::one();
		vec3<T> inv_dir = safe_inv_dir(direction);
		query_ray_recursive(0, origin, inv_dir, best_t, cb);
	}

	const std::vector<node> & get_nodes() const { return nodes_; }
	bool empty() const { return nodes_.empty(); }
	int size() const { return static_cast<int>(nodes_.size()); }

private:
	std::vector<node> nodes_;

	int build_recursive(std::vector<std::pair<aa_box<T>, Item>> & entries, int start, int end) {
		int idx = static_cast<int>(nodes_.size());
		nodes_.push_back({});

		if (end - start == 1) {
			// Leaf node — nodes_ won't reallocate here since we reserved 2x
			nodes_[idx].box = entries[start].first;
			nodes_[idx].item = entries[start].second;
			return idx;
		}

		// Compute encompassing AABB
		aa_box<T> total = entries[start].first;
		for (int i = start + 1; i < end; i++) {
			total.merge(entries[i].first);
		}
		nodes_[idx].box = total;

		// Split axis: longest extent
		T dx = total.maxs.x - total.mins.x;
		T dy = total.maxs.y - total.mins.y;
		T dz = total.maxs.z - total.mins.z;
		int axis = (dx >= dy && dx >= dz) ? 0 : (dy >= dz) ? 1 : 2;

		// Sort by centroid on split axis
		std::sort(entries.begin() + start,
		          entries.begin() + end,
		          [axis](const std::pair<aa_box<T>, Item> & a, const std::pair<aa_box<T>, Item> & b) {
			          T ca, cb;
			          if (axis == 0) {
				          ca = a.first.mins.x + a.first.maxs.x;
				          cb = b.first.mins.x + b.first.maxs.x;
			          } else if (axis == 1) {
				          ca = a.first.mins.y + a.first.maxs.y;
				          cb = b.first.mins.y + b.first.maxs.y;
			          } else {
				          ca = a.first.mins.z + a.first.maxs.z;
				          cb = b.first.mins.z + b.first.maxs.z;
			          }
			          return ca < cb;
		          });

		int mid = (start + end) / 2;

		// Note: recursive calls may grow nodes_, but we access nodes_[idx]
		// by integer index after, so reallocation is safe.
		int left = build_recursive(entries, start, mid);
		int right = build_recursive(entries, mid, end);
		nodes_[idx].left = left;
		nodes_[idx].right = right;

		return idx;
	}

	template <typename Callback> void query_aabb_recursive(int idx, const aa_box<T> & box, Callback && cb) const {
		const auto & n = nodes_[idx];
		if (!test_intersection(n.box, box))
			return;

		if (n.is_leaf()) {
			cb(n.item);
			return;
		}
		if (n.left >= 0)
			query_aabb_recursive(n.left, box, cb);
		if (n.right >= 0)
			query_aabb_recursive(n.right, box, cb);
	}

	// Ray-AABB slab test. Returns true if ray hits box before best_t.
	// We use a dedicated slab test rather than hop's find_intersection(segment, aa_box)
	// because we only need a boolean (no hit point/normal), making this faster for
	// BVH traversal where we test many nodes but only care about pruning.
	static bool ray_hits_aabb(const vec3<T> & origin, const vec3<T> & inv_dir, const aa_box<T> & box, T best_t) {
		T zero_val {};

		T t1 = (box.mins.x - origin.x) * inv_dir.x;
		T t2 = (box.maxs.x - origin.x) * inv_dir.x;
		T tmin = tr::min_val(t1, t2);
		T tmax = tr::max_val(t1, t2);

		t1 = (box.mins.y - origin.y) * inv_dir.y;
		t2 = (box.maxs.y - origin.y) * inv_dir.y;
		tmin = tr::max_val(tmin, tr::min_val(t1, t2));
		tmax = tr::min_val(tmax, tr::max_val(t1, t2));

		t1 = (box.mins.z - origin.z) * inv_dir.z;
		t2 = (box.maxs.z - origin.z) * inv_dir.z;
		tmin = tr::max_val(tmin, tr::min_val(t1, t2));
		tmax = tr::min_val(tmax, tr::max_val(t1, t2));

		return tmax >= tr::max_val(tmin, zero_val) && tmin < best_t;
	}

	static vec3<T> safe_inv_dir(const vec3<T> & direction) {
		T zero_val {};
		// Use a large-but-safe reciprocal for zero components.
		// For fixed16, from_int(10000) = 10000 << 16 which fits int32.
		// For float, 1e7 is fine.
		T big;
		if constexpr (std::is_same_v<T, float>) {
			big = 1e7f;
		} else {
			big = tr::from_int(10000);
		}
		return vec3<T>(direction.x != zero_val ? tr::one() / direction.x : big,
		               direction.y != zero_val ? tr::one() / direction.y : big,
		               direction.z != zero_val ? tr::one() / direction.z : big);
	}

	template <typename Callback>
	void query_ray_recursive(
	    int idx, const vec3<T> & origin, const vec3<T> & inv_dir, T & best_t, Callback && cb) const {
		const auto & n = nodes_[idx];

		if (!ray_hits_aabb(origin, inv_dir, n.box, best_t))
			return;

		if (n.is_leaf()) {
			cb(n.item, best_t);
			return;
		}
		if (n.left >= 0)
			query_ray_recursive(n.left, origin, inv_dir, best_t, cb);
		if (n.right >= 0)
			query_ray_recursive(n.right, origin, inv_dir, best_t, cb);
	}
};

} // namespace hop
