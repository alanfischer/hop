#pragma once

#include <hop/bvh.h>
#include <hop/manager.h>
#include <hop/math/intersect.h>
#include <hop/solid.h>

#include <algorithm>
#include <utility>
#include <vector>

namespace hop {

// A concrete hop::manager implementation that uses BVHs for spatial
// acceleration. Static solids live in one BVH; dynamic solids live in a
// second BVH that is refit each tick from the solids' current world bounds
// (topology fixed; rebuilt only when solids are added/removed). Below
// linear_scan_threshold solids in either category, queries fall back to
// a linear scan — for small N, the tree traversal loses to a tight loop.
//
// Usage:
//   bvh_manager<float> mgr;
//   mgr.add_solid(static_wall, true);
//   mgr.add_solid(player, false);
//   simulator.set_manager(&mgr);
//
// Static BVH: call rebuild() after adding/removing static solids, or it
// rebuilds lazily on the next query. If a "static" solid's world_bound
// changes — e.g., the caller moves it or reshapes it — call mark_dirty().
//
// Dynamic BVH: refit happens automatically in pre_update(dt), so as
// long as the simulator drives the tick the dynamic tree is always fresh
// at the start of a tick. After many ticks the topology drifts (objects
// leave their initial clusters), inflating internal-node AABBs and
// degrading query precision; rebuild_dynamic() is exposed for callers
// who want to recover quality periodically.

template <typename T> class bvh_manager : public manager<T> {
public:
	void add_solid(solid<T> * s, bool is_static) {
		if (is_static) {
			static_solids_.push_back(s);
			dirty_ = true;
		} else {
			dynamic_solids_.push_back(s);
			dynamic_dirty_ = true;
		}
		// iteration_order_ contains both buckets, so any add invalidates it —
		// including static adds, which leave dynamic_dirty_ untouched.
		order_dirty_ = true;
	}

	void remove_solid(solid<T> * s) {
		auto it_s = std::find(static_solids_.begin(), static_solids_.end(), s);
		if (it_s != static_solids_.end()) {
			static_solids_.erase(it_s);
			dirty_ = true;
			// Drop the now-dangling pointer from the cached order immediately;
			// it may be read again before the next rebuild.
			order_dirty_ = true;
			return;
		}
		auto it_d = std::find(dynamic_solids_.begin(), dynamic_solids_.end(), s);
		if (it_d != dynamic_solids_.end()) {
			dynamic_solids_.erase(it_d);
			dynamic_dirty_ = true;
			order_dirty_ = true;
		}
	}

	// Mark the static BVH stale. Call after a static solid's position or
	// shape changes; the next query will rebuild.
	void mark_dirty() { dirty_ = true; }

	// Mark the dynamic BVH's leaf bounds stale without invalidating topology.
	// For callers that move dynamic solids outside the simulator tick (an engine
	// integration moving trigger volumes, say) and still need the next query to
	// see current bounds. Unlike mark_dirty() this costs an O(n) refit rather
	// than a sort-and-rebuild; topology is still refreshed periodically so
	// drift cannot degrade query precision indefinitely.
	void mark_dynamic_moved() { dynamic_moved_ = true; }

	void rebuild() {
		std::vector<std::pair<aa_box<T>, solid<T> *>> entries;
		entries.reserve(static_solids_.size());

		for (auto * s : static_solids_) {
			if (s->get_shapes().empty())
				continue;
			entries.push_back({ s->get_world_bound(), s });
		}

		bvh_.build(entries);
		dirty_ = false;
	}

	// Refit the dynamic BVH in place from current world bounds, keeping topology.
	// Periodically falls back to a full rebuild so accumulated drift (solids
	// wandering out of their original clusters, inflating internal AABBs) cannot
	// degrade query precision without bound.
	void refit_dynamic() {
		if (++refits_since_dynamic_rebuild_ >= dynamic_rebuild_period) {
			rebuild_dynamic();
			return;
		}
		dynamic_bvh_.refit([](solid<T> * s) { return s->get_world_bound(); });
		dynamic_moved_ = false;
	}

	// Rebuild dynamic BVH topology from current world bounds. Cheap refits
	// suffice between rebuilds, but call this if dynamics have drifted far
	// from their initial spatial clustering and queries are returning too
	// many false positives.
	void rebuild_dynamic() {
		std::vector<std::pair<aa_box<T>, solid<T> *>> entries;
		entries.reserve(dynamic_solids_.size());

		for (auto * s : dynamic_solids_) {
			if (s->get_shapes().empty())
				continue;
			entries.push_back({ s->get_world_bound(), s });
		}

		dynamic_bvh_.build(entries);
		dynamic_dirty_ = false;
		dynamic_moved_ = false;
		ticks_since_dynamic_rebuild_ = 0;
		refits_since_dynamic_rebuild_ = 0;

		// Not built from this tree any more (see rebuild_iteration_order) — but an
		// add or a remove is what usually brings us here, so keep it in step.
		if (order_dirty_)
			rebuild_iteration_order();
	}

	// Rebuild the simulator's update order: dynamics in insertion order, statics
	// after them. Invariant the simulator relies on: the order must contain
	// *every* solid this manager owns exactly once, so its size equals
	// dynamic_solids_.size() + static_solids_.size().
	//
	// CANONICAL, and that is the whole point. Both vectors are insertion-ordered
	// (remove_solid erases, which preserves the relative order of the rest), so
	// this order is the solids' solve_id_ order — a pure function of WHICH
	// solids exist, never of when this manager last touched its trees.
	//
	// It used to be the dynamic BVH's DFS leaf order, for cache locality on the
	// update pass. That made the order a function of the tree's TOPOLOGY, which
	// is rebuilt on an add/remove or every dynamic_rebuild_period ticks and only
	// refit in between — so the same solids in the same places iterated in a
	// different order depending on when they were added relative to the last
	// rebuild. A Gauss-Seidel solve is order-dependent, so that is a different
	// simulation: two peers running one jointed ragdoll from byte-identical
	// inputs came out 0.3m apart in a single tick, because one built its bones
	// inside a physics tick (rebuild, then throw) and the other between two
	// (throw, then rebuild). Order is the one thing a solver must not take from
	// history. The BVH still accelerates every broad-phase query — it just no
	// longer decides who is solved first.
	void rebuild_iteration_order() {
		iteration_order_.clear();
		iteration_order_.reserve(dynamic_solids_.size() + static_solids_.size());
		for (auto * s : dynamic_solids_)
			iteration_order_.push_back(s);
		for (auto * s : static_solids_)
			iteration_order_.push_back(s);
		order_dirty_ = false;
	}

	int get_static_count() const { return static_cast<int>(static_solids_.size()); }
	int get_dynamic_count() const { return static_cast<int>(dynamic_solids_.size()); }

	// ---- manager<T> interface ----

	// Below this static-count, skip the BVH and scan inline — a tree traversal
	// of 10-15 nodes loses to iterating ~20 AABBs in a flat array (virtual
	// dispatch + pointer chasing costs more than the saved comparisons).
	// Measured break-even is around 10-15 statics; 20 leaves headroom.
	static constexpr int linear_scan_threshold = 20;

	// Force a full rebuild of the dynamic BVH every N ticks even when no
	// solids have been added or removed. Refit-only preserves topology, so
	// after objects rearrange the internal-node AABBs grow loose and queries
	// return more false positives. Rebuilding restores tight clustering.
	// 16 ticks was the sweet spot in the bounce_stress 10000-sphere scene
	// (sweep over 16/32/64/128/256 showed flatter curve past 16; rebuild
	// cost is ~1 ms/build at N=10000, well below the savings).
	static constexpr int dynamic_rebuild_period = 16;

	int find_solids_in_aa_box(const aa_box<T> & box, solid<T> * solids[], int max_solids,
	                          int collide_with_bits = -1) override {
		// -1 means "no filter", not "all bits": it must keep reporting scope-0 solids,
		// which a bitwise test against -1 would drop.
		const bool filter = collide_with_bits != -1;
		int count = 0;
		auto accepts = [&](solid<T> * s) {
			return !filter || (collide_with_bits & s->get_collision_scope()) != 0;
		};

		if (static_cast<int>(static_solids_.size()) >= linear_scan_threshold) {
			if (dirty_)
				rebuild();
			bvh_.query_aabb(box, [&](solid<T> * s) {
				if (count < max_solids && accepts(s)) {
					solids[count] = s;
					count++;
				}
			});
		} else {
			for (auto * s : static_solids_) {
				if (count >= max_solids)
					break;
				if (s->get_shapes().empty())
					continue;
				if (!accepts(s))
					continue;
				if (test_intersection(box, s->get_world_bound())) {
					solids[count] = s;
					count++;
				}
			}
		}

		// Dynamics: BVH above threshold, linear scan below. Refit normally
		// happens in pre_update; rebuild lazily here only if we're stale and
		// no tick has yet driven the refit (e.g., direct caller queries).
		if (static_cast<int>(dynamic_solids_.size()) >= linear_scan_threshold) {
			if (dynamic_dirty_) {
				rebuild_dynamic();
			} else if (dynamic_moved_) {
				refit_dynamic();
			}
			dynamic_bvh_.query_aabb(box, [&](solid<T> * s) {
				if (count < max_solids && accepts(s)) {
					solids[count] = s;
					count++;
				}
			});
		} else {
			for (auto * s : dynamic_solids_) {
				if (count >= max_solids)
					break;
				if (!accepts(s))
					continue;
				if (test_intersection(box, s->get_world_bound())) {
					solids[count] = s;
					count++;
				}
			}
		}

		return count;
	}

	// bvh_manager is broad-phase only — find_solids_in_aa_box accelerates the
	// simulator's scan, but this manager contributes no geometry of its own.
	// trace_segment / trace_solid stay no-ops; engine integrations that want
	// to merge in external geometry (level BSP, heightfield, static trimesh)
	// should subclass manager<T> directly and override these. See manager.h.
	void trace_segment(collision<T> & result, const segment<T> & seg, int collide_with_bits) override {}
	void trace_solid(collision<T> & result, solid<T> * s, const segment<T> & seg, int collide_with_bits, T margin) override {}

	// Refresh the dynamic BVH at the start of every tick. Rebuild when
	// topology is stale (adds/removes since last rebuild) or every
	// dynamic_rebuild_period ticks to recover from drift; otherwise refit.
	// Below threshold the linear scan is faster, so we skip the BVH work.
	void pre_update(T dt) override {
		if (static_cast<int>(dynamic_solids_.size()) < linear_scan_threshold)
			return;
		// The order follows adds and removes and nothing else now, so it is
		// refreshed on its own dirty flag rather than off the back of a rebuild
		// (see rebuild_iteration_order).
		if (order_dirty_)
			rebuild_iteration_order();
		if (dynamic_dirty_ || ++ticks_since_dynamic_rebuild_ >= dynamic_rebuild_period) {
			rebuild_dynamic();
			ticks_since_dynamic_rebuild_ = 0;
		} else {
			dynamic_bvh_.refit([](solid<T> * s) { return s->get_world_bound(); });
			dynamic_moved_ = false;
		}
	}
	void post_update(T dt) override {}
	void pre_update(solid<T> * s, T dt) override {}
	void intra_update(solid<T> * s, T dt) override {}
	bool collision_response(solid<T> * s, vec3<T> & position, vec3<T> & remainder, collision<T> & col) override {
		return false;
	}
	void post_update(solid<T> * s, T dt) override {}

	// Spatial-locality iteration order. Returned when the dynamic BVH is in
	// active use — below that threshold the linear-scan fallback wouldn't
	// have a meaningful order to suggest.
	const std::vector<solid<T> *> * get_iteration_order() const override {
		if (static_cast<int>(dynamic_solids_.size()) < linear_scan_threshold)
			return nullptr;
		return &iteration_order_;
	}

private:
	std::vector<solid<T> *> static_solids_;
	std::vector<solid<T> *> dynamic_solids_;
	std::vector<solid<T> *> iteration_order_;
	bvh<T, solid<T> *> bvh_;
	bvh<T, solid<T> *> dynamic_bvh_;
	bool dirty_ = false;
	bool dynamic_dirty_ = false;
	bool order_dirty_ = false;
	int ticks_since_dynamic_rebuild_ = 0;
	bool dynamic_moved_ = false;
	int refits_since_dynamic_rebuild_ = 0;
};

} // namespace hop
