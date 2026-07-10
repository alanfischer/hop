#pragma once

#include <algorithm>
#include <functional>
#include <hop/collision.h>
#include <hop/math/quat.h>
#include <hop/math/support.h>
#include <hop/shape.h>
#include <memory>
#include <vector>

namespace hop {

template <typename T> class constraint;
template <typename T> class simulator;
template <typename T> class manager;

// How two contacting bodies' coefficients of restitution are combined into the
// single COR used to resolve the contact. Each body carries its own mode; when
// the two differ, the higher-precedence one governs, precedence being ascending
// enum order (the numerically larger enumerator wins).
//
// Precedence is asymmetric, so a lower mode is NOT an absolute per-body
// guarantee: a `minimum`/COR-0 body still bounces fully off a `maximum`/COR-1
// partner (resolved cor = max(0, 1) = 1). Only `maximum`, as the top mode,
// always wins. To force behavior regardless of the partner, set `maximum` for
// bouncy; for damped, set `minimum` and keep partners off `multiply`/`maximum`.
enum class restitution_combine { average, minimum, multiply, maximum };

// Per-body contact-resolution strategy.
//  - speculative: discover -> solve velocity -> integrate, with an NGS position
//    backstop. Contacts are made non-penetrating at the velocity level before the
//    body moves, so resting piles dissipate to true rest and fast bodies don't
//    tunnel. This is the dynamics-solver path.
//  - sweep_slide: the swept collide-and-slide path — integrate, sweep to the
//    earliest time-of-impact, snap, and slide the remainder along the surface.
//    Geometric, convergence-independent, character-controller-style movement.
// A sweep_slide body still exchanges impulses through the shared velocity solve,
// so it is pushed by speculative bodies normally; only its *positioning* differs.
// The two can be mixed per body (e.g. a sweep_slide character among speculative
// balls). See simulator::update.
enum class contact_mode { speculative, sweep_slide };

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
		vec3<T>    impact;            // world contact point on the surface (lever-arm origin for angular surface velocity)
		vec3<T>    lever;             // contact point relative to THIS solid's center at discovery (impact − swept center). Sweep-independent body-frame offset; the angular solver anchors it at the current center so a fast oblique impact doesn't fabricate a tangential lever arm (which would spin a frictionless sphere)
		T          accum_n {};        // accumulated normal impulse magnitude (>= 0)
		vec3<T>    accum_t;           // accumulated friction impulse (this-side convention: the impulse applied to self)
		T          impact_speed {};   // approach speed at TOI; drives restitution target
		T          separation {};     // signed gap along normal at discovery: 0 touching, <0 penetrating (speculative target)
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
		orientation_.reset();
		orientation_q_.reset();
		velocity_.reset();
		angular_velocity_.reset();
		force_.reset();
		torque_.reset();
		inertia_.reset();
		inv_inertia_.reset(); // zero ⇒ no dynamic spin (rotation opt-in)
		update_inv_inertia_world(); // orientation is identity + inv_inertia zero here ⇒ M = 0
		coefficient_of_gravity_ = tr::one();
		coefficient_of_restitution_ = tr::half();
		restitution_combine_ = restitution_combine::average;
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
		for (auto & slot : touches_)
			slot = touch{};  // restore every slot to its in-class defaults
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

	// Contact-resolution strategy for this body (see contact_mode). New bodies
	// inherit the simulator's default (simulator::set_default_contact_mode) when
	// added; call this to override an individual body — e.g. a sweep_slide
	// character moving among speculative balls.
	void set_contact_mode(contact_mode m) { contact_mode_ = m; }
	contact_mode get_contact_mode() const { return contact_mode_; }
	// True when this body is resolved by the speculative solve: its position is
	// committed in the simulator's Pass B (from the solved velocity + NGS
	// correction) rather than during the Pass A swept move, and it absorbs NGS
	// position correction. The single predicate the per-body dispatch keys on, so a
	// future third mode forces one explicit decision here rather than silently
	// falling between the Pass A / Pass B branches.
	bool uses_speculative_solve() const { return contact_mode_ == contact_mode::speculative; }

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
	// Static orientation: a fixed body rotation the narrowphase honors (traceable
	// + GJK paths). No angular velocity / torque — see the rotation design notes.
	// Identity by default, in which case it is an exact no-op everywhere.
	void set_orientation(const mat3<T> & r) {
		orientation_ = r;
		set_quat_from_mat3(orientation_q_, r); // keep the integrated quat in sync with external writes
		update_inv_inertia_world();            // world I⁻¹ depends on orientation
		recompute_world_bound();
	}
	// Commit an integrated quat (Phase 8 dynamic spin): single writer of the
	// "both representations + world AABB are now consistent" transition, so the
	// mat3/quat invariant can't silently desync. set_orientation is its mat3-in twin.
	void set_orientation_from_quat(const quat<T> & q) {
		orientation_q_ = q;
		set_mat3_from_quat(orientation_, q);
		update_inv_inertia_world();            // world I⁻¹ depends on orientation
		recompute_world_bound();
	}
	const mat3<T> & get_orientation() const { return orientation_; }
	const quat<T> & get_orientation_quat() const { return orientation_q_; }
	void set_velocity(const vec3<T> & vel) {
		velocity_.set(vel);
		activate();
	}
	const vec3<T> & get_velocity() const { return velocity_; }
	// Kinematic angular velocity (axis·rate about `position_`). Phase 6: this is
	// scripted spin only — there is no inertia/torque integration, so a finite-mass
	// body does not acquire it from collisions. Its sole effect is the surface
	// velocity ω × (contact − position) the contact solver feeds into the existing
	// non-penetration / friction constraints, so a spinning kinematic platform
	// carries the riders touching it. Zero by default → exact no-op everywhere.
	void set_angular_velocity(const vec3<T> & av) {
		angular_velocity_.set(av);
		activate();
	}
	const vec3<T> & get_angular_velocity() const { return angular_velocity_; }
	void add_force(const vec3<T> & f) {
		add(force_, f);
		activate();
	}
	const vec3<T> & get_force() const { return force_; }
	void clear_force() { force_.reset(); }

	// Dynamic rotation (Phase 8). Inertia is the principal-axis diagonal in the body
	// frame; inv_inertia_ (the per-component reciprocal, 0 where a component is 0) is
	// the canonical marker — inv_inertia_==0 means "this body never spins
	// dynamically" (static/kinematic brushes, pinned props). Defaults to zero, so a
	// body acquires spin only once a game gives it finite inertia (rotation opt-in).
	void set_inertia(const vec3<T> & I) {
		inertia_.set(I);
		inv_inertia_.x = I.x > T {} ? tr::one() / I.x : T {};
		inv_inertia_.y = I.y > T {} ? tr::one() / I.y : T {};
		inv_inertia_.z = I.z > T {} ? tr::one() / I.z : T {};
		update_inv_inertia_world();
	}
	const vec3<T> & get_inertia() const { return inertia_; }
	const vec3<T> & get_inv_inertia() const { return inv_inertia_; }
	// Cached world-space inverse inertia tensor M = R·diag(inv_inertia)·Rᵀ. The
	// solver applies I⁻¹ (angular effective mass + every angular impulse) many
	// times per tick, but orientation is fixed for the duration of the contact
	// solve — so we build M once here (from set_inertia and the orientation
	// writers) and the hot path reduces to a single mat3×vec3. Zero for a
	// non-rotating body (inv_inertia == 0 ⇒ M == 0), matching the old per-call math.
	const mat3<T> & get_inv_inertia_world() const { return inv_inertia_world_; }
	void update_inv_inertia_world() {
		mat3<T> d; // diag(inv_inertia_); default-constructed to identity, so clear it
		d.data[0] = inv_inertia_.x;
		d.data[4] = inv_inertia_.y;
		d.data[8] = inv_inertia_.z;
		d.data[1] = d.data[2] = d.data[3] = T {};
		d.data[5] = d.data[6] = d.data[7] = T {};
		mat3<T> rt, rd;
		transpose(rt, orientation_);
		mul(rd, orientation_, d);
		mul(inv_inertia_world_, rd, rt);
	}
	// True iff the body integrates orientation under torque (any inv_inertia axis
	// non-zero). The integrator early-outs on false, so the cost is a single compare
	// for the default (non-rotating) body.
	bool rotates_dynamically() const {
		return inv_inertia_.x != T {} || inv_inertia_.y != T {} || inv_inertia_.z != T {};
	}
	void add_torque(const vec3<T> & t) {
		add(torque_, t);
		activate();
	}
	const vec3<T> & get_torque() const { return torque_; }
	void clear_torque() { torque_.reset(); }

	// Coefficients
	void set_coefficient_of_gravity(T c) { coefficient_of_gravity_ = c; }
	T get_coefficient_of_gravity() const { return coefficient_of_gravity_; }
	void set_coefficient_of_restitution(T c) { coefficient_of_restitution_ = c; }
	T get_coefficient_of_restitution() const { return coefficient_of_restitution_; }
	void set_restitution_combine(restitution_combine m) { restitution_combine_ = m; }
	restitution_combine get_restitution_combine() const { return restitution_combine_; }
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
			rotate_aabb(local_bound_, local_bound_, shapes_[0]->get_local_rotation());
			add(local_bound_, shapes_[0]->get_local_position());
			aa_box<T> box;
			for (int i = 1; i < static_cast<int>(shapes_.size()); ++i) {
				shape_types_ |= static_cast<int>(shapes_[i]->get_type());
				shapes_[i]->get_bound(box);
				rotate_aabb(box, box, shapes_[i]->get_local_rotation());
				add(box, shapes_[i]->get_local_position());
				local_bound_.merge(box);
			}
		}
		recompute_world_bound();
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
		recompute_world_bound();
	}

	// world_bound_ = AABB enclosing (orientation_ · local_bound_) + position_.
	// Identity orientation reproduces the old translate-only bound exactly.
	void recompute_world_bound() {
		rotate_aabb(world_bound_, local_bound_, orientation_);
		add(world_bound_, position_);
	}

	// -- Hot: every-tick gates and integration math --
	bool active_ = true;
	int scope_ = -1;
	vec3<T> position_;
	mat3<T> orientation_;         // body→world rotation; the queried form. For a dynamic body it is derived from orientation_q_ each step; identity by default
	quat<T> orientation_q_;       // integrated orientation (dynamic spin); kept in sync with orientation_ (set_orientation / integrate_angular)
	vec3<T> velocity_;
	vec3<T> angular_velocity_;    // world-frame ω (axis·rate about position_); zero by default = no-op. Kinematic scripted spin OR dynamically integrated
	vec3<T> force_;
	vec3<T> torque_;              // accumulated world-frame torque (Phase 8); cleared each integrate_angular
	vec3<T> inertia_;             // principal-axis diagonal (Ix,Iy,Iz) in the body frame; for the I·ω gyroscopic term
	vec3<T> inv_inertia_;         // per-component reciprocal of inertia_ (0 where a component is 0). PRIMARY marker: inv_inertia_==0 ⇒ never spins dynamically. Zero by default ⇒ rotation is opt-in
	mat3<T> inv_inertia_world_;   // cached R·diag(inv_inertia_)·Rᵀ; see get_inv_inertia_world/update_inv_inertia_world. Zero for a non-rotating body
	vec3<T> pos_correction_;      // speculative NGS position solver scratch (pseudo-position, not velocity)
	bool solve_frozen_ = false;   // shock-propagation scratch: treated as a rigid support for this tick's velocity solve
	contact_mode contact_mode_ = contact_mode::sweep_slide;  // positioning strategy (see contact_mode)
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
	restitution_combine restitution_combine_ = restitution_combine::average;
	T coefficient_of_static_friction_ {};
	T coefficient_of_dynamic_friction_ {};
	int collision_scope_ = -1;
	int collide_with_scope_ = -1;
	int trigger_scope_ = 0;
	int deactivate_count_ = 0;
	simulator<T> * simulator_ = nullptr;
	// Stable, monotonic id assigned by the simulator at add time. The contact
	// solver canonicalizes each pair as a<b by this id rather than by raw
	// pointer: heap addresses vary run-to-run (ASLR), and in low-precision
	// fixed-point the resulting flip in impulse-application order diverges into
	// different trajectories. An insertion id makes the solve order — and thus
	// the result — reproducible across runs.
	std::size_t solve_id_ = 0;

	// -- Cold: rarely accessed in the hot path --
	// Persistent per-pair contact cache. Each slot remembers a body this solid
	// is (or was very recently) in contact with, with the previous tick's
	// accumulated normal/friction impulses for warm-starting. The simulator's
	// post-integration solver walks every active solid's slots, builds a unique
	// pair list, and runs Gauss–Seidel sweeps over it; this is what makes a
	// settled stack transmit load through all support contacts in one tick.
	//
	// max_touches slots cover the simultaneous support set a body can hold in a
	// dense pile (floor + neighbors + a wall), with headroom past the 6-neighbor
	// kissing arrangement a sphere can wedge into. When a further distinct partner
	// is hit with the cache full, we evict whichever cached slot contributes least
	// to gravity-aligned support (smallest dot(normal, -g)), tie-breaking by oldest
	// last_tick.
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
