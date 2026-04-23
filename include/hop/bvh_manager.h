#pragma once

#include <hop/bvh.h>
#include <hop/manager.h>
#include <hop/math/intersect.h>
#include <hop/solid.h>

#include <algorithm>
#include <utility>
#include <vector>

namespace hop {

// A concrete hop::manager implementation that uses a BVH for spatial
// acceleration. Static solids are indexed in the BVH; dynamic solids
// are tracked in a flat list (typically small).
//
// Usage:
//   bvh_manager<float> mgr;
//   mgr.add_solid(static_wall, true);
//   mgr.add_solid(player, false);
//   simulator.set_manager(&mgr);
//
// Call rebuild() after adding/removing static solids, or it will
// rebuild lazily on the next query.
//
// If a "static" solid's world_bound changes — e.g., the caller moves it
// or reshapes it — call mark_dirty() to schedule a rebuild. Moving static
// solids is supported but the BVH will only re-index on the next query.

template <typename T> class bvh_manager : public manager<T> {
public:
	void add_solid(solid<T> * s, bool is_static) {
		if (is_static) {
			static_solids_.push_back(s);
			dirty_ = true;
		} else {
			dynamic_solids_.push_back(s);
		}
	}

	void remove_solid(solid<T> * s) {
		auto it_s = std::find(static_solids_.begin(), static_solids_.end(), s);
		if (it_s != static_solids_.end()) {
			static_solids_.erase(it_s);
			dirty_ = true;
			return;
		}
		auto it_d = std::find(dynamic_solids_.begin(), dynamic_solids_.end(), s);
		if (it_d != dynamic_solids_.end()) {
			dynamic_solids_.erase(it_d);
		}
	}

	// Mark the BVH stale. Call after a static solid's position or shape
	// changes; the next query will rebuild.
	void mark_dirty() { dirty_ = true; }

	void rebuild() {
		std::vector<std::pair<aa_box<T>, solid<T> *>> entries;
		entries.reserve(static_solids_.size());

		for (auto * s : static_solids_) {
			if (s->get_num_shapes() == 0)
				continue;
			entries.push_back({ s->get_world_bound(), s });
		}

		bvh_.build(entries);
		dirty_ = false;
	}

	int get_static_count() const { return static_cast<int>(static_solids_.size()); }
	int get_dynamic_count() const { return static_cast<int>(dynamic_solids_.size()); }

	// ---- manager<T> interface ----

	int find_solids_in_aa_box(const aa_box<T> & box, solid<T> * solids[], int max_solids) override {
		if (dirty_)
			rebuild();

		int count = 0;

		// Query BVH for static solids
		bvh_.query_aabb(box, [&](solid<T> * s) {
			if (count < max_solids) {
				solids[count] = s;
				count++;
			}
		});

		// Linear scan for dynamic solids
		for (auto * s : dynamic_solids_) {
			if (count >= max_solids)
				break;
			if (test_intersection(box, s->get_world_bound())) {
				solids[count] = s;
				count++;
			}
		}

		return count;
	}

	// The simulator already traces against solids from find_solids_in_aa_box.
	// These are for additional geometry not in the solid list (unused here).
	void trace_segment(collision<T> & result, const segment<T> & seg, int collide_with_bits) override {}
	void trace_solid(collision<T> & result, solid<T> * s, const segment<T> & seg, int collide_with_bits) override {}

	// Update callbacks — no-ops for basic spatial acceleration.
	void pre_update(int dt, T fdt) override {}
	void post_update(int dt, T fdt) override {}
	void pre_update(solid<T> * s, int dt, T fdt) override {}
	void intra_update(solid<T> * s, int dt, T fdt) override {}
	bool collision_response(solid<T> * s, vec3<T> & position, vec3<T> & remainder, collision<T> & col) override {
		return false;
	}
	void post_update(solid<T> * s, int dt, T fdt) override {}

private:
	std::vector<solid<T> *> static_solids_;
	std::vector<solid<T> *> dynamic_solids_;
	bvh<T, solid<T> *> bvh_;
	bool dirty_ = false;
};

} // namespace hop
