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
//
// Built top-down with a binned surface-area heuristic, one item per leaf, and stored
// depth-first: an internal node's left child is the node after it, so a node is a box
// and two ints, and refit is one reverse pass. Queries walk an explicit stack.

template <typename T, typename Item> class bvh {
public:
	using tr = scalar_traits<T>;

	struct node {
		aa_box<T> box;
		int index = 0;     // a leaf's slot in items_; an internal node's right child
		bool leaf = false;
		bool is_leaf() const { return leaf; }
	};

	// Build from a list of (AABB, item) pairs. The input vector is not modified.
	void build(std::vector<std::pair<aa_box<T>, Item>> & entries) {
		nodes_.clear();
		items_.clear();
		if (entries.empty())
			return;
		const int n = static_cast<int>(entries.size());
		refs_.resize(n);
		for (int i = 0; i < n; ++i) {
			const aa_box<T> & b = entries[i].first;
			refs_[i].index = i;
			for (int a = 0; a < 3; ++a)
				refs_[i].c[a] = (tr::to_float(b.mins[a]) + tr::to_float(b.maxs[a])) * 0.5f;
		}
		nodes_.reserve(2 * n - 1);
		items_.reserve(n);
		build_node(entries, 0, n, 0);
	}

	template <typename Callback> void query_aabb(const aa_box<T> & box, Callback && cb) const {
		if (nodes_.empty() || !test_intersection(nodes_[0].box, box))
			return;
		int stack[max_stack];
		int top = 0;
		int idx = 0;
		for (;;) {
			const node & n = nodes_[idx];
			if (n.leaf) {
				cb(items_[n.index]);
			} else {
				// Children are tested from the parent, so each box is tested once.
				const int l = idx + 1, r = n.index;
				const bool hl = test_intersection(nodes_[l].box, box);
				const bool hr = test_intersection(nodes_[r].box, box);
				if (hl) {
					if (hr)
						stack[top++] = r;
					idx = l;
					continue;
				}
				if (hr) {
					idx = r;
					continue;
				}
			}
			if (top == 0)
				return;
			idx = stack[--top];
		}
	}

	// Recompute every node's box from get_box(item) without changing the topology.
	// Children follow their parent, so one reverse pass is bottom-up.
	template <typename GetBox> void refit(GetBox && get_box) {
		for (int idx = static_cast<int>(nodes_.size()) - 1; idx >= 0; --idx) {
			node & n = nodes_[idx];
			if (n.leaf) {
				n.box = get_box(items_[n.index]);
			} else {
				n.box = nodes_[idx + 1].box;
				n.box.merge(nodes_[n.index].box);
			}
		}
	}

	// The callback may shrink best_t (the hit fraction along direction) to cull the rest.
	template <typename Callback>
	void query_ray(const vec3<T> & origin, const vec3<T> & direction, Callback && cb) const {
		if (nodes_.empty())
			return;
		T best_t = tr::one();
		int stack[max_stack];
		int top = 0;
		stack[top++] = 0;
		while (top > 0) {
			const int idx = stack[--top];
			const node & n = nodes_[idx];
			if (!ray_hits_aabb(origin, direction, n.box, best_t))
				continue;
			if (n.leaf) {
				cb(items_[n.index], best_t);
				continue;
			}
			stack[top++] = n.index;
			stack[top++] = idx + 1;
		}
	}

	// Every item once, in leaf order (spatially clustered).
	template <typename Out> void collect_leaves(Out & out) const {
		for (const auto & it : items_)
			out.push_back(it);
	}

	const std::vector<node> & get_nodes() const { return nodes_; }
	bool empty() const { return nodes_.empty(); }
	int size() const { return static_cast<int>(nodes_.size()); }

private:
	// Past this depth the build splits at the median, which halves the count, so no
	// path is longer than sah_depth + log2(count) and the query stacks hold that.
	static constexpr int sah_depth = 32;
	static constexpr int max_stack = sah_depth + 40;
	static constexpr int bins = 12;

	struct ref {
		float c[3];   // centroid; the build bins in float whatever T is
		int index;    // into entries
	};

	std::vector<node> nodes_;
	std::vector<Item> items_;
	std::vector<ref> refs_; // build scratch

	static float half_area(const float mn[3], const float mx[3]) {
		const float dx = mx[0] - mn[0], dy = mx[1] - mn[1], dz = mx[2] - mn[2];
		return dx * dy + dy * dz + dz * dx;
	}

	void build_node(const std::vector<std::pair<aa_box<T>, Item>> & entries, int start, int end, int depth) {
		const int idx = static_cast<int>(nodes_.size());
		nodes_.push_back({});
		aa_box<T> total = entries[refs_[start].index].first;
		float cmin[3], cmax[3];
		for (int a = 0; a < 3; ++a)
			cmin[a] = cmax[a] = refs_[start].c[a];
		for (int i = start + 1; i < end; ++i) {
			total.merge(entries[refs_[i].index].first);
			for (int a = 0; a < 3; ++a) {
				cmin[a] = std::min(cmin[a], refs_[i].c[a]);
				cmax[a] = std::max(cmax[a], refs_[i].c[a]);
			}
		}
		nodes_[idx].box = total;
		if (end - start == 1) {
			nodes_[idx].leaf = true;
			nodes_[idx].index = static_cast<int>(items_.size());
			items_.push_back(entries[refs_[start].index].second);
			return;
		}
		int axis = 0;
		for (int a = 1; a < 3; ++a)
			if (cmax[a] - cmin[a] > cmax[axis] - cmin[axis])
				axis = a;
		const float extent = cmax[axis] - cmin[axis];
		int mid = -1;
		if (depth < sah_depth && extent > 0.0f)
			mid = sah_split(entries, start, end, axis, cmin[axis], extent);
		if (mid <= start || mid >= end) {
			mid = (start + end) / 2;
			std::nth_element(refs_.begin() + start, refs_.begin() + mid, refs_.begin() + end,
			                 [axis](const ref & a, const ref & b) { return a.c[axis] < b.c[axis]; });
		}
		build_node(entries, start, mid, depth + 1);
		nodes_[idx].index = static_cast<int>(nodes_.size());
		build_node(entries, mid, end, depth + 1);
	}

	// Binned SAH along `axis`: partition at the bin boundary with the least
	// area-weighted count and return the split, or -1 if every item fell in one bin.
	int sah_split(const std::vector<std::pair<aa_box<T>, Item>> & entries, int start, int end, int axis, float lo,
	              float extent) {
		struct bin {
			float mn[3] = { 1e30f, 1e30f, 1e30f };
			float mx[3] = { -1e30f, -1e30f, -1e30f };
			int n = 0;
			void grow(const bin & o) {
				for (int a = 0; a < 3; ++a) {
					mn[a] = std::min(mn[a], o.mn[a]);
					mx[a] = std::max(mx[a], o.mx[a]);
				}
				n += o.n;
			}
		} b[bins];
		const float scale = bins / extent;
		auto bin_of = [&](const ref & r) {
			const int k = static_cast<int>((r.c[axis] - lo) * scale);
			return k < 0 ? 0 : (k >= bins ? bins - 1 : k);
		};
		for (int i = start; i < end; ++i) {
			bin & bb = b[bin_of(refs_[i])];
			const aa_box<T> & box = entries[refs_[i].index].first;
			for (int a = 0; a < 3; ++a) {
				bb.mn[a] = std::min(bb.mn[a], tr::to_float(box.mins[a]));
				bb.mx[a] = std::max(bb.mx[a], tr::to_float(box.maxs[a]));
			}
			++bb.n;
		}
		// right[k]: bins k.. merged.
		bin right[bins];
		right[bins - 1] = b[bins - 1];
		for (int k = bins - 2; k > 0; --k) {
			right[k] = right[k + 1];
			right[k].grow(b[k]);
		}
		float best = 1e30f;
		int best_k = -1;
		bin left;
		for (int k = 1; k < bins; ++k) {
			left.grow(b[k - 1]);
			if (left.n == 0 || right[k].n == 0)
				continue;
			const float cost = left.n * half_area(left.mn, left.mx) + right[k].n * half_area(right[k].mn, right[k].mx);
			if (cost < best) {
				best = cost;
				best_k = k;
			}
		}
		if (best_k < 0)
			return -1;
		auto it = std::partition(refs_.begin() + start, refs_.begin() + end,
		                         [&](const ref & r) { return bin_of(r) < best_k; });
		return static_cast<int>(it - refs_.begin());
	}

	static bool ray_hits_aabb(const vec3<T> & origin, const vec3<T> & direction, const aa_box<T> & box, T best_t) {
		T zero_val {};
		T tmin = zero_val;
		T tmax = best_t;
		for (int a = 0; a < 3; ++a) {
			if (direction[a] != zero_val) {
				T inv = tr::one() / direction[a];
				T t1 = (box.mins[a] - origin[a]) * inv;
				T t2 = (box.maxs[a] - origin[a]) * inv;
				tmin = tr::max_val(tmin, tr::min_val(t1, t2));
				tmax = tr::min_val(tmax, tr::max_val(t1, t2));
			} else if (origin[a] < box.mins[a] || origin[a] > box.maxs[a]) {
				return false;
			}
		}
		return tmax >= tmin && tmin < best_t;
	}
};

} // namespace hop
