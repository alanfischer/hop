#pragma once

#include <algorithm>
#include <functional>
#include <hop/collision.h>
#include <hop/math/support.h>
#include <hop/shape.h>
#include <memory>
#include <vector>

namespace hop {

template <typename T> class constraint;
template <typename T> class simulator;
template <typename T> class manager;

template <typename T> class solid : public std::enable_shared_from_this<solid<T>> {
public:
	using ptr = std::shared_ptr<solid<T>>;
	using tr = scalar_traits<T>;

	// A single persistent contact slot. Populated by the simulator's TOI loop
	// (one slot per partner this solid has hit recently) and consumed by the
	// post-integration contact solver, which iterates Gauss–Seidel sweeps over
	// every active body's slots. accum_n / accum_t carry the previous tick's
	// accumulated impulses for warm-starting; impact_speed captures the
	// approach velocity at TOI for restitution. last_tick is the refresh
	// marker — slots whose last_tick falls behind the current tick are
	// considered stale and dropped by the solver.
	struct touch {
		solid<T> * partner = nullptr;
		vec3<T>    normal;            // points from partner toward this solid (the separating direction for self)
		T          accum_n {};        // accumulated normal impulse magnitude (>= 0)
		vec3<T>    accum_t;           // accumulated friction impulse (this-side convention: the impulse applied to self)
		T          impact_speed {};   // approach speed at TOI; drives restitution target
		int        last_tick = -1;    // refresh marker; stale slots are skipped by the solver
		int        pair_built_tick = -1; // bumped to current_tick when the solver has already built a pair via this slot's twin (dedup)
	};
	static constexpr int max_touches = 12;

	static T infinite_mass() { return -tr::one(); }

	solid() { reset(); }

	void destroy() {
		while (!constraints_.empty()) {
			auto * c = constraints_[0];
			if (c->start_solid_.get() == this) {
				c->destroy();
			} else {
				c->set_end_solid(nullptr);
				internal_remove_constraint(c);
			}
		}
		shapes_.clear();
	}

	void reset() {
		destroy();
		scope_ = -1;
		collision_scope_ = -1;
		collide_with_scope_ = -1;
		trigger_scope_ = 0;
		mass_ = tr::one();
		inv_mass_ = tr::one();
		position_.reset();
		velocity_.reset();
		force_.reset();
		coefficient_of_gravity_ = tr::one();
		coefficient_of_restitution_ = tr::half();
		coefficient_of_restitution_override_ = false;
		coefficient_of_static_friction_ = tr::half();
		coefficient_of_dynamic_friction_ = tr::half();
		coefficient_of_effective_drag_ = T {};
		shape_types_ = 0;
		local_bound_.reset();
		world_bound_.reset();
		collision_callback_ = nullptr;
		user_data_ = nullptr;
		active_ = true;
		deactivate_count_ = 0;
		touch_count_ = 0;
		for (int i = 0; i < max_touches; ++i) {
			touches_[i].partner = nullptr;
			touches_[i].normal.reset();
			touches_[i].accum_n = T{};
			touches_[i].accum_t.reset();
			touches_[i].impact_speed = T{};
			touches_[i].last_tick = -1;
			touches_[i].pair_built_tick = -1;
		}
		simulator_ = nullptr;
	}

	// Scope bitmasks. Four independent ints with different roles:
	//
	//   scope_              — per-tick activation mask. Matched against the
	//                         `scope` argument of simulator::update(dt, scope):
	//                         the solid is updated only if (scope_ & scope) != 0
	//                         (or `scope` is 0, the "tick everything" default).
	//                         Default `-1` (all bits = always tick).
	//
	//   collision_scope_    — channels this solid broadcasts on. When something
	//                         else traces against this solid, the trace's
	//                         collide_with bits must intersect this mask, or
	//                         the pair is filtered out. Default `-1`.
	//
	//   collide_with_scope_ — channels this solid listens to. Set to 0 to
	//                         disable broad-phase entirely for this solid
	//                         (useful for sensors that only emit callbacks).
	//                         Used as the `collide_with_bits` argument when
	//                         the solid is the moving side of a sweep.
	//                         Default `-1`.
	//
	//   trigger_scope_      — trigger / zone identification bits. When another
	//                         solid is found to be statically overlapping this
	//                         one (collision time t == 0), these bits are OR'd
	//                         into the resulting collision::scope field. Lets
	//                         callers tag a solid as a "damage zone" or "water
	//                         volume" and read which zones their player is
	//                         currently inside off the trace result. Works for
	//                         primitive shapes AND traceable meshes. Default 0
	//                         (no trigger bits — the solid is just geometry).
	void set_scope(int s) { scope_ = s; }
	int get_scope() const { return scope_; }
	void set_collision_scope(int s) { collision_scope_ = s; }
	int get_collision_scope() const { return collision_scope_; }
	void set_collide_with_scope(int s) { collide_with_scope_ = s; }
	int get_collide_with_scope() const { return collide_with_scope_; }
	void set_trigger_scope(int s) { trigger_scope_ = s; }
	int get_trigger_scope() const { return trigger_scope_; }

	// Mass
	void set_mass(T mass) {
		mass_ = mass;
		if (mass > T {}) {
			inv_mass_ = tr::one() / mass;
		} else {
			inv_mass_ = T {};
		}
	}
	T get_mass() const { return mass_; }
	void set_infinite_mass() {
		mass_ = infinite_mass();
		inv_mass_ = T {};
	}
	bool has_infinite_mass() const { return mass_ == infinite_mass(); }

	// Position / Velocity / Force
	void set_position(const vec3<T> & pos);
	const vec3<T> & get_position() const { return position_; }
	void set_velocity(const vec3<T> & vel) {
		velocity_.set(vel);
		activate();
	}
	const vec3<T> & get_velocity() const { return velocity_; }
	void add_force(const vec3<T> & f) {
		add(force_, f);
		activate();
	}
	const vec3<T> & get_force() const { return force_; }
	void clear_force() { force_.reset(); }

	// Coefficients
	void set_coefficient_of_gravity(T c) { coefficient_of_gravity_ = c; }
	T get_coefficient_of_gravity() const { return coefficient_of_gravity_; }
	void set_coefficient_of_restitution(T c) { coefficient_of_restitution_ = c; }
	T get_coefficient_of_restitution() const { return coefficient_of_restitution_; }
	void set_coefficient_of_restitution_override(bool o) { coefficient_of_restitution_override_ = o; }
	bool get_coefficient_of_restitution_override() const { return coefficient_of_restitution_override_; }
	void set_coefficient_of_static_friction(T c) { coefficient_of_static_friction_ = c; }
	T get_coefficient_of_static_friction() const { return coefficient_of_static_friction_; }
	void set_coefficient_of_dynamic_friction(T c) { coefficient_of_dynamic_friction_ = c; }
	T get_coefficient_of_dynamic_friction() const { return coefficient_of_dynamic_friction_; }
	void set_coefficient_of_effective_drag(T c) { coefficient_of_effective_drag_ = c; }
	T get_coefficient_of_effective_drag() const { return coefficient_of_effective_drag_; }

	// Shapes
	void add_shape(typename shape<T>::ptr s) {
		shapes_.push_back(s);
		s->solid_ = this;
		update_local_bound();
		activate();
	}

	void remove_shape(typename shape<T>::ptr s) {
		shapes_.erase(std::remove(shapes_.begin(), shapes_.end(), s), shapes_.end());
		s->solid_ = nullptr;
		update_local_bound();
		activate();
	}

	void remove_all_shapes() {
		shapes_.clear();
		update_local_bound();
		activate();
	}
	const std::vector<typename shape<T>::ptr> & get_shapes() const { return shapes_; }
	int get_shape_types() const { return shape_types_; }

	const aa_box<T> & get_local_bound() const { return local_bound_; }
	const aa_box<T> & get_world_bound() const { return world_bound_; }

	// Diagnostic: read the persistent contact cache. Slots whose last_tick
	// matches the simulator's current tick are live contacts this step; older
	// slots are stale (will be reaped on the next solver pass).
	int get_touch_count() const { return touch_count_; }
	const touch & get_touch(int i) const { return touches_[i]; }

	using collision_fn = std::function<void(const collision<T> &)>;
	void set_collision_callback(collision_fn fn) { collision_callback_ = std::move(fn); }
	const collision_fn & get_collision_callback() const { return collision_callback_; }

	using collision_filter_fn = std::function<bool(solid<T> *)>;
	void set_collision_filter(collision_filter_fn fn) { collision_filter_ = std::move(fn); }
	bool should_collide(solid<T> * other) const { return !collision_filter_ || (other && collision_filter_(other)); }

	void set_user_data(void * d) { user_data_ = d; }
	void * get_user_data() const { return user_data_; }

	void activate() {
		if (deactivate_count_ > 0)
			deactivate_count_ = 0;
		if (!active_) {
			active_ = true;
			for (auto * c : constraints_) {
				if (c->start_solid_.get() != this && c->start_solid_)
					c->start_solid_->activate();
				else if (c->end_solid_.get() != this && c->end_solid_)
					c->end_solid_->activate();
			}
		}
	}

	void set_stay_active(bool a) {
		deactivate_count_ = a ? -1 : 0;
		activate();
	}
	void deactivate() {
		active_ = false;
		deactivate_count_ = 0;
		// A sleeping body is at rest by definition. The deactivation test gates on
		// positional stillness, but the swept-collision snap can leave a small
		// residual velocity in velocity_ each tick (the body jitters in place
		// without its position drifting). Freezing that residual would (a) make a
		// "resting" pile carry phantom kinetic energy and (b) inject it back the
		// instant a neighbour wakes the body. Zero it so sleep means rest.
		velocity_.reset();
	}
	bool active() const { return active_ && simulator_ != nullptr; }

	void update_local_bound() {
		shape_types_ = 0;
		if (shapes_.empty()) {
			local_bound_.reset();
		} else {
			shape_types_ |= static_cast<int>(shapes_[0]->get_type());
			shapes_[0]->get_bound(local_bound_);
			add(local_bound_, shapes_[0]->get_local_position());
			aa_box<T> box;
			for (int i = 1; i < static_cast<int>(shapes_.size()); ++i) {
				shape_types_ |= static_cast<int>(shapes_[i]->get_type());
				shapes_[i]->get_bound(box);
				add(box, shapes_[i]->get_local_position());
				local_bound_.merge(box);
			}
		}
		add(world_bound_, local_bound_, position_);
	}

private:
	void internal_set_simulator(simulator<T> * s) { simulator_ = s; }
	void internal_add_constraint(constraint<T> * c) { constraints_.push_back(c); }
	void internal_remove_constraint(constraint<T> * c) {
		constraints_.erase(std::remove(constraints_.begin(), constraints_.end(), c), constraints_.end());
	}

	// Bypass activate(). The simulator calls this from update_solid every tick
	// to commit the integrated position; activating from there would re-arm the
	// deactivation counter on every step and prevent solids from ever sleeping.
	void set_position_direct(const vec3<T> & pos) {
		position_.set(pos);
		add(world_bound_, local_bound_, position_);
	}

	// -- Hot: every-tick gates and integration math --
	bool active_ = true;
	int scope_ = -1;
	vec3<T> position_;
	vec3<T> velocity_;
	vec3<T> force_;
	aa_box<T> world_bound_;       // broad phase reads this
	aa_box<T> local_bound_;
	std::vector<typename shape<T>::ptr> shapes_;
	int shape_types_ = 0;
	T mass_ {};
	T inv_mass_ {};
	T coefficient_of_gravity_ {};
	T coefficient_of_effective_drag_ {};

	// -- Warm: collision response (read on actual hits, not per pair) --
	T coefficient_of_restitution_ {};
	bool coefficient_of_restitution_override_ = false;
	T coefficient_of_static_friction_ {};
	T coefficient_of_dynamic_friction_ {};
	int collision_scope_ = -1;
	int collide_with_scope_ = -1;
	int trigger_scope_ = 0;
	int deactivate_count_ = 0;
	simulator<T> * simulator_ = nullptr;

	// -- Cold: rarely accessed in the hot path --
	// Persistent per-pair contact cache. Each slot remembers a body this solid
	// is (or was very recently) in contact with, with the previous tick's
	// accumulated normal/friction impulses for warm-starting. The simulator's
	// post-integration solver walks every active solid's slots, builds a unique
	// pair list, and runs Gauss–Seidel sweeps over it; this is what makes a
	// settled stack transmit load through all support contacts in one tick.
	//
	// 6 slots empirically covers the largest stable support set a sphere can
	// have in 3D (a sphere wedged into the kissing arrangement has 6 neighbors).
	// When a 7th distinct partner is hit, we evict whichever cached slot
	// contributes least to gravity-aligned support (smallest dot(normal, -g)),
	// tie-breaking by oldest last_tick.
	touch touches_[max_touches];
	int touch_count_ = 0;

	std::vector<constraint<T> *> constraints_;

	collision_fn collision_callback_;
	collision_filter_fn collision_filter_;
	void * user_data_ = nullptr;

	friend class constraint<T>;
	friend class shape<T>;
	friend class simulator<T>;
};

template <typename T> inline void solid<T>::set_position(const vec3<T> & pos) {
	set_position_direct(pos);
	activate();
}

} // namespace hop
