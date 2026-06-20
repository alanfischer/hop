#pragma once

#include <algorithm>
#include <cassert>
#include <hop/collide.h>
#include <hop/collision.h>
#include <hop/constraint.h>
#include <hop/manager.h>
#include <hop/math/bounding.h>
#include <hop/math/intersect.h>
#include <hop/math/support.h>
#include <hop/math/project.h>
#include <hop/solid.h>
#include <vector>

namespace hop {

enum class integrator_type {
	euler,
	improved,
	heun,
	runge_kutta,
};

template <typename T> class simulator {
public:
	using tr = scalar_traits<T>;

	simulator(const vec3<T> gravity = { T {}, T {}, -tr::from_milli(9810) }) {
		set_gravity(gravity);
		init_epsilon_defaults();
		collisions_.resize(64);
		// Immovable world anchor: the partner for speculative contacts against
		// manager-injected geometry that has no owning solid. solve_id 0 sorts it
		// before every real solid (which start at 1) in the canonical pair order.
		static_world_.set_infinite_mass();
		static_world_.solve_id_ = 0;
	}

	~simulator() = default;

	// Epsilon
	void set_epsilon(T epsilon) {
		static_assert(!is_fixed_scalar_v<T>, "set_epsilon not available for fixed types; use set_epsilon_bits");
		epsilon_ = tr::make_epsilon(epsilon);
	}

	void set_epsilon_bits(int bits) {
		static_assert(is_fixed_scalar_v<T>, "set_epsilon_bits only for fixed types; use set_epsilon for float/double");
		epsilon_ = tr::make_epsilon(bits);
	}

	T get_epsilon() const { return epsilon_; }

	// Integrator
	void set_integrator(integrator_type i) { integrator_ = i; }
	integrator_type get_integrator() const { return integrator_; }

	void set_average_normals(bool a) { average_normals_ = a; }
	bool get_average_normals() const { return average_normals_; }

	// Narrowphase mode for rounded-shape (sphere/capsule) vs polytope
	// (convex_solid/box) pairs. true (default): accurate GJK closest-point sweep
	// — correct edge/vertex normals (ledge ride-up) and bounded. false: the cheap
	// plane-inflation / AABB-expansion path — faster, but face-normals-only at
	// edges. Zero-radius polytope pairs always use the cheap path regardless.
	void set_accurate_narrowphase(bool a) { accurate_narrowphase_ = a; }
	bool get_accurate_narrowphase() const { return accurate_narrowphase_; }

	void set_max_position_component(T v) { max_position_component_ = v; }
	T get_max_position_component() const { return max_position_component_; }
	void set_max_velocity_component(T v) { max_velocity_component_ = v; }
	T get_max_velocity_component() const { return max_velocity_component_; }
	void set_max_force_component(T v) { max_force_component_ = v; }
	T get_max_force_component() const { return max_force_component_; }

	void set_fluid_velocity(const vec3<T> & v) { fluid_velocity_ = v; }
	const vec3<T> & get_fluid_velocity() const { return fluid_velocity_; }

	void set_gravity(const vec3<T> & g) {
		gravity_ = g;
		for (auto & s : solids_)
			s->activate();
	}
	const vec3<T> & get_gravity() const { return gravity_; }

	void set_manager(manager<T> * m) { manager_ = m; }
	manager<T> * get_manager() const { return manager_; }

	void set_micro_collision_threshold(T t) { micro_collision_threshold_ = t; }
	T get_micro_collision_threshold() const { return micro_collision_threshold_; }

	void set_deactivate_speed(T s) { deactivate_speed_ = s; }
	void set_deactivate_count(int c) { deactivate_count_ = c; }

	// Contact solver. Pass B (post-integration Gauss–Seidel sweep over the
	// touched-pair graph) iterates this many times per tick. Higher counts let
	// taller stacks transmit load in a single tick at the cost of solver time;
	// warm-starting from the previous tick's accumulated impulses means a
	// resting pile converges in roughly one iteration anyway.
	void set_solver_iterations(int n) { solver_iterations_ = n > 0 ? n : 1; }
	int get_solver_iterations() const { return solver_iterations_; }

	// Per-body, per-tick budget for the swept-collision sub-step loop in
	// update_solid. Each sub-step snaps to the earliest contact along the
	// remaining motion and slides the leftover on; a body whose step crosses
	// many colliders (a long path through a crowded scene) needs one sub-step
	// per colliding layer. When the budget runs out the body is pushed off its
	// last contact and left where it is for the tick — so too low a limit lets a
	// fast body in a dense scene slip through geometry it never traced against.
	void set_max_collision_iterations(int n) { max_collision_iterations_ = n > 0 ? n : 1; }
	int get_max_collision_iterations() const { return max_collision_iterations_; }

	// Contact resolution is a per-body property (solid::set_contact_mode); the
	// simulator has no global pipeline switch. This sets the default mode stamped
	// onto bodies as they are added (add_solid), so a scene that wants one strategy
	// throughout sets it once here; mixed scenes override individual bodies. Default
	// is sweep_slide (the swept collide-and-slide path). See contact_mode and
	// solid::set_contact_mode.
	void set_default_contact_mode(contact_mode m) { default_contact_mode_ = m; }
	contact_mode get_default_contact_mode() const { return default_contact_mode_; }

	// Shock-propagation passes appended to the speculative velocity solve (see
	// spec_shock_iters_). Lets a deep pile transmit support to the floor without
	// needing solver_iterations ≈ pile-depth. 0 disables it.
	void set_shock_iterations(int n) { spec_shock_iters_ = n < 0 ? 0 : n; }
	int get_shock_iterations() const { return spec_shock_iters_; }

	// Remaining speculative-pipeline knobs (defaults set in init_epsilon_defaults).
	// spec_margin: how far past the predicted motion contacts are discovered.
	// spec_slop: penetration tolerated without correction (resting-jitter band).
	// pos_baumgarte: fraction of remaining penetration the NGS removes per iter.
	// pos_iterations: NGS position-solver iterations per tick (0 disables it).
	void set_speculative_margin(T m) { spec_margin_ = m; }
	T get_speculative_margin() const { return spec_margin_; }
	void set_speculative_slop(T s) { spec_slop_ = s; }
	T get_speculative_slop() const { return spec_slop_; }
	void set_position_baumgarte(T b) { spec_pos_baumgarte_ = b; }
	T get_position_baumgarte() const { return spec_pos_baumgarte_; }
	void set_position_iterations(int n) { spec_pos_iters_ = n < 0 ? 0 : n; }
	int get_position_iterations() const { return spec_pos_iters_; }

	// Solid management
	void add_solid(std::shared_ptr<solid<T>> s) {
		for (auto & existing : solids_) {
			if (existing == s)
				return;
		}
		solids_.push_back(s);
		s->internal_set_simulator(this);
		s->solve_id_ = next_solve_id_++;  // stable canonical-ordering key (see solid::solve_id_)
		s->set_contact_mode(default_contact_mode_);  // per-body default; override after add_solid
		s->activate();
		spacial_collection_.resize(solids_.size());
	}

	void remove_solid(std::shared_ptr<solid<T>> s) {
		// Purge any cached references to this solid from every other body's
		// contact cache so the solver doesn't dereference a dead partner.
		auto * dead = s.get();
		for (auto & other : solids_) {
			if (other.get() == dead)
				continue;
			int w = 0;
			for (int r = 0; r < other->touch_count_; ++r) {
				if (other->touches_[r].partner != dead) {
					if (w != r)
						other->touches_[w] = other->touches_[r];
					++w;
				}
			}
			other->touch_count_ = w;
		}
		s->touch_count_ = 0;

		if (reporting_collisions_) {
			for (int i = 0; i < num_collisions_; ++i) {
				auto & c = collisions_[i];
				if (c.collider == s.get())
					c.collider = nullptr;
				if (c.collidee == s.get())
					c.collidee = nullptr;
			}
		}

		s->internal_set_simulator(nullptr);
		solids_.erase(std::remove(solids_.begin(), solids_.end(), s), solids_.end());
	}

	const std::vector<std::shared_ptr<solid<T>>> & get_solids() const { return solids_; }

	// Constraint management
	void add_constraint(typename constraint<T>::ptr c) {
		for (auto & existing : constraints_) {
			if (existing == c)
				return;
		}
		constraints_.push_back(c);
		c->internal_set_simulator(this);
	}

	void remove_constraint(typename constraint<T>::ptr c) {
		c->internal_set_simulator(nullptr);
		constraints_.erase(std::remove(constraints_.begin(), constraints_.end(), c), constraints_.end());
	}

	// Main update
	void update(T dt, int scope = 0, solid<T> * target = nullptr) {
		num_collisions_ = 0;
		++current_tick_;
		if (manager_)
			manager_->pre_update(dt);

		// Build the iteration list. When a target is given we update just that
		// one solid. Otherwise the manager may suggest a spatial-locality
		// iteration order; contract: when non-null, it must contain every solid
		// the simulator should update — anything in solids_ but absent from the
		// order is skipped. With no manager order, iterate solids_ directly.
		const std::vector<solid<T> *> * order =
		    (target == nullptr && manager_) ? manager_->get_iteration_order() : nullptr;
		// The manager's order is a spatial-locality *hint*, not a contract. If a
		// manager desync or a mid-tick add/remove leaves it out of sync with
		// solids_, ignore it for this tick and iterate solids_ directly. A stale
		// order would otherwise skip bodies (release) or dereference a dangling
		// solid pointer (use-after-free) — both far worse than losing locality
		// for one tick. (Was an assert(); promoted to a graceful fallback.)
		if (order && order->size() != solids_.size())
			order = nullptr;

		const size_t num = target ? 1 : (order ? order->size() : solids_.size());
		const bool flip = !target && (current_tick_ & 1);

		// Resolve the solid for iteration index ii (applying the per-tick flip),
		// or nullptr if it should be skipped this tick. Shared by both pipelines.
		auto select = [&](size_t ii) -> solid<T> * {
			size_t i = flip ? num - 1 - ii : ii;
			solid<T> * s = target ? target : (order ? (*order)[i] : solids_[i].get());
			if (!s->active_ || (scope != 0 && (s->scope_ & scope) == 0))
				return nullptr;
			return s;
		};

		// Pass A: integrate every body, dispatching on its contact mode. A
		// sweep_slide body resolves and commits its position now (integrate -> sweep
		// -> snap -> slide); a speculative body integrates velocity and discovers
		// contacts at its frame-start position without moving — its position is
		// committed in Pass B. `any_speculative` records whether the NGS position
		// pass and shock propagation have work to do; a scene with no speculative
		// body behaves exactly as the legacy swept pipeline did.
		bool any_speculative = false;
		for (size_t ii = 0; ii < num; ++ii) {
			solid<T> * s = select(ii);
			if (!s) continue;
			if (manager_) manager_->pre_update(s, dt);
			if (s->uses_speculative_solve()) {
				any_speculative = true;
				integrate_and_discover(s, dt);
			} else {
				update_solid(s, dt);
				if (manager_) manager_->post_update(s, dt);
			}
		}

		// Global velocity solve over the touched-pair graph: stacking force
		// propagates here and impulses are exchanged at each body's real mass —
		// including between a sweep_slide body and speculative bodies, so a
		// sweep_slide character is pushed by speculative balls normally. The target
		// is unified (see solve_contacts); the flag gates only its speculative-only
		// shock-propagation phase.
		solve_contacts(dt, any_speculative);

		if (any_speculative) {
			// Remove residual penetration the velocity solve leaves under load as a
			// pseudo-position correction (no velocity change → no energy added). Acts
			// on speculative bodies only — sweep_slide bodies placed themselves in
			// Pass A — so a speculative body takes the whole correction against a
			// sweep_slide partner. commit_solid folds the correction into the move.
			correct_positions();
			// Pass B: commit each speculative body's position; sweep_slide bodies
			// already committed (and ran post_update) in Pass A.
			for (size_t ii = 0; ii < num; ++ii) {
				solid<T> * s = select(ii);
				if (!s || !s->uses_speculative_solve()) continue;
				commit_solid(s, dt);
				if (manager_) manager_->post_update(s, dt);
			}
		}

		report_collisions();
		if (manager_)
			manager_->post_update(dt);
	}

	void update_solid(solid<T> * solid_ptr, T dt);
	// Speculative path (contact_mode::speculative). Pass A integrates velocity and
	// discovers contacts without moving the body; Pass B commits the position from
	// the solved velocity and handles deactivation.
	void integrate_and_discover(solid<T> * solid_ptr, T dt);
	void commit_solid(solid<T> * solid_ptr, T dt);
	// Iterative non-linear Gauss–Seidel position solver (speculative pipeline):
	// removes residual penetration as a pseudo-position correction, re-deriving
	// each contact's separation from the running correction so multi-contact
	// corrections converge instead of summing. Velocity is never touched.
	void correct_positions();
	// Shared deactivation/sleep logic: given the body's about-to-be-committed
	// position, decide whether it qualifies to sleep this tick. Used by both the
	// default and speculative pipelines.
	void try_deactivate(solid<T> * solid_ptr, const vec3<T> & new_pos, T dt);

	// Find solids in box
	int find_solids_in_aa_box(const aa_box<T> & box, solid<T> * solids[], int max_solids) const {
		aa_box<T> expanded(box);
		expanded.mins.x -= epsilon_;
		expanded.mins.y -= epsilon_;
		expanded.mins.z -= epsilon_;
		expanded.maxs.x += epsilon_;
		expanded.maxs.y += epsilon_;
		expanded.maxs.z += epsilon_;

		int amount = -1;
		if (manager_)
			amount = manager_->find_solids_in_aa_box(expanded, solids, max_solids);

		if (amount == -1) {
			amount = 0;
			for (auto & s : solids_) {
				if (test_intersection(expanded, s->world_bound_)) {
					if (amount < max_solids)
						solids[amount] = s.get();
					amount++;
				}
			}
		}
		if (amount > max_solids)
			amount = max_solids;
		return amount;
	}

	// Trace / test
	void trace_segment(collision<T> & result,
	                   const segment<T> & seg,
	                   int collide_with_bits = -1,
	                   solid<T> * ignore = nullptr);
	void trace_solid(collision<T> & result, solid<T> * s, const segment<T> & seg, int collide_with_bits = -1);
	void test_segment(collision<T> & result, const segment<T> & seg, solid<T> * s) {
		hop::test_segment(result, seg, s, epsilon_);
	}
	void test_solid(collision<T> & result, solid<T> * s1, const segment<T> & seg, solid<T> * s2, T margin = T {}) {
		hop::test_solid(result, s1, seg, s2, epsilon_, margin, accurate_narrowphase_);
	}

	// Utility
	void cap_vec3(vec3<T> & v, T value) const {
		v.x = tr::cap(v.x, value);
		v.y = tr::cap(v.y, value);
		v.z = tr::cap(v.z, value);
	}

	void calculate_epsilon_offset(vec3<T> & result, const vec3<T> & direction, const vec3<T> & normal) const {
		T len = length(direction);
		if (len > epsilon_) {
			result.x = (-direction.x / len) * epsilon_;
			result.y = (-direction.y / len) * epsilon_;
			result.z = (-direction.z / len) * epsilon_;
		} else {
			result.reset();
		}
	}

	bool too_small(const vec3<T> & v, T epsilon) const {
		T x = v.x, y = v.y, z = v.z;
		return x < epsilon && x > -epsilon && y < epsilon && y > -epsilon && z < epsilon && z > -epsilon;
	}

	int count_active_solids() const {
		int n = 0;
		for (auto & s : solids_)
			if (s->active_)
				n++;
		return n;
	}

private:
	void init_epsilon_defaults() {
		if constexpr (is_fixed_scalar_v<T>) {
			set_epsilon_bits(tr::default_epsilon_bits());
		} else {
			set_epsilon(tr::default_epsilon());
		}
		max_position_component_ = tr::default_max_position_component();
		max_velocity_component_ = tr::default_max_velocity_component();
		max_force_component_ = tr::default_max_force_component();
		deactivate_speed_ = tr::one() * tr::from_milli(200); // 0.2
		deactivate_count_ = 32;
		micro_collision_threshold_ = tr::one() * tr::from_milli(1000); // 1.0
		// Speculative-contacts tuning (consulted only for contact_mode::speculative bodies).
		spec_margin_ = epsilon_ * tr::from_int(8);  // discover contacts this far past the predicted motion
		spec_slop_ = epsilon_;                       // penetration tolerated without correction (anti-jitter)
		spec_pos_baumgarte_ = tr::from_milli(800);   // 0.8 — fraction of penetration removed per NGS iteration
	}

	void report_collisions();
	void trace_segment_with_current_spacials(collision<T> & result,
	                                         const segment<T> & seg,
	                                         int collide_with_bits,
	                                         solid<T> * ignore);
	void trace_solid_with_current_spacials(collision<T> & result,
	                                       solid<T> * s,
	                                       const segment<T> & seg,
	                                       int collide_with_bits);

	void constraint_link(vec3<T> & result, solid<T> * s, const vec3<T> & solid_pos, const vec3<T> & solid_vel);
	void update_acceleration(vec3<T> & result, solid<T> * s, const vec3<T> & x, const vec3<T> & v, T dt);

	// Insert or refresh a slot in s's persistent touch cache. Existing slots
	// keep their accum_n / accum_t for warm-starting; impact_speed grows
	// monotonically within a tick so the strongest impact drives restitution.
	// When the cache is full and the partner is new, evict the slot
	// contributing least to gravity-aligned support (smallest dot(n, -g)),
	// tie-breaking by oldest last_tick.
	typename solid<T>::touch * add_or_refresh_touch(solid<T> * s,
	                                                solid<T> * partner,
	                                                const vec3<T> & normal,
	                                                const vec3<T> & impact,
	                                                T impact_speed,
	                                                T separation,
	                                                int tick);
	// Find an existing cache slot for (s, partner) or nullptr.
	typename solid<T>::touch * find_touch(solid<T> * s, solid<T> * partner);

	// has_speculative gates the speculative-only shock-propagation phase (true when
	// any body resolved this tick uses the speculative solve).
	void solve_contacts(T dt, bool has_speculative);

	void integration_step(solid<T> * s,
	                      const vec3<T> & x,
	                      const vec3<T> & v,
	                      const vec3<T> & dx,
	                      const vec3<T> & dv,
	                      T dt,
	                      vec3<T> & result_x,
	                      vec3<T> & result_v);

	integrator_type integrator_ = integrator_type::heun;
	vec3<T> fluid_velocity_;
	vec3<T> gravity_;
	T epsilon_ {};
	bool average_normals_ = true;
	bool accurate_narrowphase_ = true;
	// Set by trace_solid_with_current_spacials: the un-blended normal of the
	// contact's first collider (c.collider). When average_normals blends several
	// colliders into c.normal, this still carries c.collider's true normal, which
	// the contact solver needs to avoid the settling drift (see update_solid).
	vec3<T> solid_trace_pair_normal_;
	T max_position_component_ {};
	T max_velocity_component_ {};
	T max_force_component_ {};
	std::vector<collision<T>> collisions_;
	int num_collisions_ = 0;
	std::vector<std::shared_ptr<solid<T>>> solids_;
	std::vector<typename constraint<T>::ptr> constraints_;
	std::vector<solid<T> *> spacial_collection_;
	int num_spacial_collection_ = 0;
	bool reporting_collisions_ = false;
	T micro_collision_threshold_ = tr::one();
	T deactivate_speed_ {};
	int deactivate_count_ = 0;
	manager<T> * manager_ = nullptr;
	int current_tick_ = 0;  // increments per update(); also stamps touch slot refresh
	std::size_t next_solve_id_ = 1;  // monotonic; assigned to each solid at add (see solid::solve_id_)
	// Immovable world anchor for manager-injected geometry contacts (see ctor and
	// integrate_and_discover). Never integrated, never added to solids_; used only
	// as a touch partner so the speculative solver can resolve a body against
	// external static geometry with no owning solid.
	solid<T> static_world_;

	// Pass B (post-integration contact solver) working set. The pair list is
	// rebuilt every tick from the union of all active solids' touch caches,
	// canonicalized so each (a,b) appears once with a < b. Allocations amortize
	// after the first tick since the vector is cleared, not destroyed.
	struct contact_pair {
		solid<T> * a;                // canonical: indexed earlier in solids_/iteration order
		solid<T> * b;
		vec3<T> normal;              // points from a's free side toward b
		vec3<T> v_bias;              // angular surface-velocity bias added to (v_b - v_a): ω_b×r_b - ω_a×r_a (kinematic carry; zero when neither body spins)
		T accum_n {};                // accumulated normal impulse magnitude (>= 0)
		vec3<T> accum_t;             // accumulated friction impulse (a-side convention)
		T impact_speed {};           // approach speed at TOI, for restitution
		T separation {};             // signed gap along normal (0 touching, <0 penetrating); for the speculative target
		T vn0 {};                    // relative normal velocity entering the GS (after warm-start)
		T cor {};                    // combined restitution
		T mu_s {};                   // combined static friction (cone limit while sticking)
		T mu_d {};                   // combined dynamic friction (cone limit while sliding)
		T inv_ma {};
		T inv_mb {};
		T inv_m_sum {};              // inv_ma + inv_mb, precomputed once at build
		T target {};                 // restitution target normal velocity, precomputed once
		T friction_scale {};         // -1 / inv_m_sum (friction λ scale), precomputed once
		typename solid<T>::touch * slot_a = nullptr;   // writeback target (may be null if a never observed b)
		typename solid<T>::touch * slot_b = nullptr;
	};
	std::vector<contact_pair> contact_pairs_;
	// Shock-propagation scratch, all reused per tick (no steady-state alloc):
	// shock_order_ is pair indices sorted support-end-first; shock_key_[k] is pair
	// k's gravity-depth sort key; shock_lo_[k] is its deeper (anchor) body. Depths
	// are computed once per tick since positions don't move during the solve.
	std::vector<int> shock_order_;
	std::vector<T> shock_key_;
	std::vector<solid<T> *> shock_lo_;
	// Default contact mode stamped onto bodies in add_solid (per-body overridable;
	// see set_default_contact_mode / solid::set_contact_mode).
	contact_mode default_contact_mode_ = contact_mode::sweep_slide;
	T spec_margin_ {};         // discover contacts up to this far beyond the predicted motion
	T spec_slop_ {};           // penetration tolerated without correction (kills resting jitter)
	T spec_pos_baumgarte_ {};  // fraction of remaining penetration removed per NGS position iteration
	int spec_pos_iters_ = 8;   // NGS position-solver iterations per tick
	// Shock-propagation passes appended to the velocity solve. Each pass walks
	// contacts bottom-up and freezes every body once its support-from-below is
	// resolved, so the support impulse reaches the floor in a single pass
	// regardless of stack depth — the cure for plain Gauss–Seidel needing ~depth
	// sweeps to hold a tall pile (which otherwise compresses into penetration and
	// never sleeps). 0 disables it.
	int spec_shock_iters_ = 2;
	// Iteration budget for solve_contacts' Gauss–Seidel sweeps. Each sweep
	// propagates one layer of impulse through the contact graph. Historically this
	// had to be ≳ the tallest pile, since transmitting gravity load to the floor
	// took ~depth sweeps; for speculative bodies that job is now done in O(1) by the
	// shock-propagation phase (spec_shock_iters_), so this budget no longer needs to
	// scale with stack depth — it governs friction / lateral-constraint convergence
	// and how fast a pile settles to sleep. (The sweep_slide path still relies on it
	// for depth, so the shared default stays conservative.) Warm-starting keeps
	// steady-state cost low.
	int solver_iterations_ = 16;
	// Sub-step budget for update_solid's swept-collision slide loop. Was a
	// hard-coded 5; raised to give a fast body room to resolve more colliders
	// along a long step before the loop gives up (see set_max_collision_iterations).
	int max_collision_iterations_ = 16;
};

// ============================================================================
// Implementation
// ============================================================================

template <typename T> void simulator<T>::update_solid(solid<T> * solid_ptr, T dt) {
	vec3<T> old_pos;
	vec3<T> new_pos;
	vec3<T> vel;
	vec3<T> temp;
	vec3<T> t;
	vec3<T> left_over;
	vec3<T> dx1, dx2, dv1, dv2;
	segment<T> path;
	T cor {}, impulse {}, one_over_mass {}, one_over_hit_mass {};
	int loop = 0;
	solid<T> * hit_solid = nullptr;
	collision<T> c;

	const vec3<T> zero_vec;
	const T one = tr::one();
	const T two = tr::two();
	const T three = tr::three();

	old_pos.set(solid_ptr->position_);

	// Integration
	if (integrator_ == integrator_type::euler) {
		integration_step(solid_ptr, old_pos, solid_ptr->velocity_, zero_vec, zero_vec, dt, dx1, dv1);
		mul(new_pos, dx1, dt);
		add(new_pos, old_pos);
		mul(vel, dv1, dt);
		add(vel, solid_ptr->velocity_);
	} else if (integrator_ == integrator_type::improved) {
		T hdt = dt / two;
		integration_step(solid_ptr, old_pos, solid_ptr->velocity_, zero_vec, zero_vec, dt, dx1, dv1);
		new_pos.set(dx1);
		vel.set(dv1);
		integration_step(solid_ptr, old_pos, solid_ptr->velocity_, dx1, dv1, dt, dx2, dv2);
		add(new_pos, dx2);
		mul(new_pos, hdt);
		add(new_pos, old_pos);
		add(vel, dv2);
		mul(vel, hdt);
		add(vel, solid_ptr->velocity_);
	} else if (integrator_ == integrator_type::heun) {
		T qdt = dt / tr::four();
		T ttdt = dt * two / three;
		integration_step(solid_ptr, old_pos, solid_ptr->velocity_, zero_vec, zero_vec, dt, dx1, dv1);
		new_pos.set(dx1);
		vel.set(dv1);
		integration_step(solid_ptr, old_pos, solid_ptr->velocity_, dx1, dv1, ttdt, dx2, dv2);
		mul(dx2, three);
		add(new_pos, dx2);
		mul(new_pos, qdt);
		add(new_pos, old_pos);
		mul(dv2, three);
		add(vel, dv2);
		mul(vel, qdt);
		add(vel, solid_ptr->velocity_);
	} else if (integrator_ == integrator_type::runge_kutta) {
		T hdt = dt / two;
		T sdt = dt / tr::from_int(6);
		integration_step(solid_ptr, old_pos, solid_ptr->velocity_, zero_vec, zero_vec, dt, dx1, dv1);
		new_pos.set(dx1);
		vel.set(dv1);
		integration_step(solid_ptr, old_pos, solid_ptr->velocity_, dx1, dv1, hdt, dx2, dv2);
		mul(temp, dx2, two);
		add(new_pos, temp);
		mul(temp, dv2, two);
		add(vel, temp);
		integration_step(solid_ptr, old_pos, solid_ptr->velocity_, dx2, dv2, hdt, dx1, dv1);
		mul(temp, dx1, two);
		add(new_pos, temp);
		mul(temp, dv1, two);
		add(vel, temp);
		integration_step(solid_ptr, old_pos, solid_ptr->velocity_, dx1, dv1, dt, dx2, dv2);
		add(new_pos, dx2);
		mul(new_pos, sdt);
		add(new_pos, old_pos);
		add(vel, dv2);
		mul(vel, sdt);
		add(vel, solid_ptr->velocity_);
	}

	cap_vec3(vel, max_velocity_component_);
	solid_ptr->velocity_.set(vel);
	solid_ptr->clear_force();

	bool first = true;

	if (manager_) {
		manager_->intra_update(solid_ptr, dt);
	}

	cap_vec3(old_pos, max_position_component_);
	cap_vec3(new_pos, max_position_component_);

	// Collect spacials
	if (solid_ptr->collide_with_scope_ != 0) {
		sub(temp, new_pos, old_pos);
		T m = tr::max_val(tr::abs(temp.x), tr::max_val(tr::abs(temp.y), tr::abs(temp.z))) + epsilon_;

		aa_box<T> box;
		box.set(solid_ptr->local_bound_);
		add(box, new_pos);
		box.mins.x -= m;
		box.mins.y -= m;
		box.mins.z -= m;
		box.maxs.x += m;
		box.maxs.y += m;
		box.maxs.z += m;

		num_spacial_collection_ =
		    find_solids_in_aa_box(box, spacial_collection_.data(), static_cast<int>(spacial_collection_.size()));
	}

	// Collision loop. Each iteration sweeps the body's integrated trajectory,
	// snaps to the earliest TOI, refreshes the per-body touch cache (consumed
	// later this tick by solve_contacts), and slides the leftover motion along
	// the contact tangent. Velocities are *not* mutated here — pass B owns all
	// contact-driven velocity changes, which is what lets a stack transmit
	// force through all support contacts in a single tick.
	//
	// slide_vel is a working copy of the body's velocity used only to pick
	// the slide direction for sub-step continuation. After each contact we
	// zero its component along the contact normal, so a body in a 2- or
	// 3-wall corner accumulates the constraints from every wall it has hit
	// this tick and slides along their intersection rather than diving back
	// into a previously-hit wall. solid_ptr->velocity_ is left untouched so
	// solve_contacts still sees the true pre-impact velocity for restitution.
	vec3<T> slide_vel(solid_ptr->velocity_);
	while (true) {
		if (!first) {
			sub(temp, new_pos, old_pos);
			if (too_small(temp, epsilon_)) {
				new_pos.set(old_pos);
				break;
			}
		}

		path.set_start_end(old_pos, new_pos);
		trace_solid_with_current_spacials(c, solid_ptr, path, solid_ptr->collide_with_scope_);

		if (c.time < one) {
			sub(left_over, c.point, old_pos);
			// Use c.collider's UN-blended normal (captured during the trace; see
			// solid_trace_pair_normal_) for BOTH the push-out and the solver cache, so
			// position and velocity corrections share one direction — a mismatch
			// injects energy. c.depth/c.point are already the first collider's (only the
			// normal is blended); c.normal/c.point still drive the TOI slide below.
			const vec3<T> & pair_normal = c.collider ? solid_trace_pair_normal_ : c.normal;
			if (c.time == T {} && c.depth > T {}) {
				// Penetration at frame start: separate along the contact normal by
				// the full overlap depth. Against a dynamic partner the correction
				// is split — but it must be applied to BOTH bodies here, in one
				// shot, rather than each body pushing only itself by half during
				// its own update. The old self-only split was order dependent: the
				// body updated second saw an already-reduced overlap and pushed
				// less, so the pair's center of mass crept toward whichever body
				// updated first. Over a grid of contacts that fixed iteration order
				// summed into a steady lateral drift (a settling pile sliding into
				// one corner). Moving both bodies ±depth/2 is center-of-mass
				// preserving and order-independent, killing the drift at its source.
				bool split = c.collider && !c.collider->has_infinite_mass() && c.collider->active_;
				T push = split ? c.depth * tr::half() : c.depth;
				vec3<T> correction;
				mul(correction, pair_normal, push);
				add(old_pos, correction);
				if (split) {
					// Partner moves the opposite way by the same amount. Done via
					// the swept-snap path's recovery branch invariant: this only
					// fires when bodies already overlap, so nudging the partner out
					// reduces penetration rather than creating it.
					vec3<T> partner_pos(c.collider->position_);
					sub(partner_pos, correction);
					c.collider->set_position_direct(partner_pos);
				}
			} else {
				calculate_epsilon_offset(old_pos, left_over, c.normal);
				add(old_pos, c.point);
			}
			sub(left_over, new_pos, old_pos);

			hit_solid = c.collider;

			// Relative velocity at contact (self - partner). Used both for
			// callback reporting and for computing the approach speed that
			// feeds restitution in pass B.
			if (hit_solid) {
				sub(c.velocity, solid_ptr->velocity_, hit_solid->velocity_);
			} else {
				c.velocity.set(solid_ptr->velocity_);
			}
			// c.normal points from partner toward self, so a body closing on
			// a partner has c.velocity · c.normal < 0. impact_speed flips the
			// sign so it's positive when closing, zero when separating.
			T impact_speed = -dot(c.velocity, c.normal);
			if (impact_speed < T {})
				impact_speed = T {};

			// Collision callback recording. Suppress on sustained contact
			// (partner was already touched last tick) so callbacks fire on
			// the impact, not on every refresh frame of a resting pile.
			typename solid<T>::touch * existing = hit_solid ? find_touch(solid_ptr, hit_solid) : nullptr;
			bool sustained = (existing != nullptr && existing->last_tick == current_tick_ - 1);
			if (!sustained &&
			    (solid_ptr->collision_callback_ != nullptr ||
			     (hit_solid && hit_solid->collision_callback_ != nullptr))) {
				c.collidee = solid_ptr;
				if (num_collisions_ < static_cast<int>(collisions_.size())) {
					collisions_[num_collisions_].set(c);
					num_collisions_++;
				}
			}

			// Manager-driven custom response (e.g., destroy on impact). When
			// the manager owns the response, the contact is not added to the
			// solver's cache — pass B would otherwise apply a redundant impulse.
			bool responded = false;
			if (manager_) {
				responded = manager_->collision_response(solid_ptr, old_pos, left_over, c);
			}

			if (hit_solid && !responded) {
				// Signed gap along the contact normal. The sweep_slide body has
				// already been snapped/pushed to the contact, so this is 0 at a clean
				// TOI touch and -depth when separating an existing overlap. Because
				// gap <= slop, the unified velocity target in solve_contacts reduces to
				// the legacy restitution response for this contact (the speculative
				// gap-clamp branch only fires on a positive, margin-discovered gap).
				T separation = (c.time == T{} && c.depth > T{}) ? -c.depth : T{};
				add_or_refresh_touch(solid_ptr, hit_solid, pair_normal, c.impact, impact_speed, separation, current_tick_);
				// Wake the partner if it was sleeping — pass B needs it
				// participating in the solver to redistribute force properly.
				if ((hit_solid->collide_with_scope_ & solid_ptr->collision_scope_) != 0 &&
				    hit_solid->should_collide(solid_ptr) &&
				    impact_speed > deactivate_speed_) {
					hit_solid->activate();
				}
			}

			// Project the inward component of slide_vel along this contact's
			// normal out. After multiple sub-steps slide_vel converges toward
			// the projection of the body's velocity onto the intersection of
			// all contact planes hit this tick — the right direction for a
			// body wedged in a corner to keep moving along.
			T sn = dot(slide_vel, c.normal);
			if (sn < T {}) {
				mul(temp, c.normal, sn);
				sub(slide_vel, temp);
			}

			// Project leftover motion onto the tangent plane as well.
			// Conserving the full magnitude of left_over (as the previous
			// implementation did) injects energy during angled hits.
			T dn = dot(left_over, c.normal);
			if (dn < T {}) {
				mul(temp, c.normal, dn);
				sub(left_over, temp);
			}

			if (too_small(left_over, epsilon_)) {
				new_pos.set(old_pos);
				break;
			} else if (loop >= max_collision_iterations_) {
				// Hit the sub-step budget for resolving this body's motion this
				// tick. Each iteration snaps to one contact and slides the
				// remainder on, so a body crossing many colliders in a single
				// step (a long path through a crowded scene, e.g. a fast body)
				// needs more iterations to resolve them all; running out here can
				// leave it lodged in — or past — geometry it never got to trace
				// against. Push out along the last contact normal rather than
				// zeroing motion (zeroing causes permanent overlap when two
				// dynamic bodies collide at the limit) and give up for this tick.
				mul(temp, c.normal, epsilon_ * tr::four());
				add(new_pos, old_pos, temp);
				break;
			} else {
				add(new_pos, old_pos, left_over);
				first = false;
			}
		} else {
			break;
		}
		loop++;
	}

	try_deactivate(solid_ptr, new_pos, dt);

	solid_ptr->set_position_direct(new_pos);
}

// Deactivation/sleep decision, shared by update_solid and commit_solid.
template <typename T> void simulator<T>::try_deactivate(solid<T> * solid_ptr, const vec3<T> & new_pos, T dt) {
	if (deactivate_count_ > 0 && solid_ptr->deactivate_count_ >= 0) {
		// deactivate_speed_ is a speed, so the per-tick displacement it permits
		// scales with dt: |Δx| < deactivate_speed_ · dt. Comparing the raw
		// displacement against deactivate_speed_ directly (as this once did) made
		// the effective velocity threshold 1/dt times too large — at dt = 16 ms a
		// body drifting at ~12 m/s still counted as "still" and could be frozen
		// in mid-air the instant it brushed a neighbour (the classic "ball hanging
		// in the air"). Gating on true speed lets only genuinely slow bodies sleep.
		T deactivate_disp = deactivate_speed_ * dt;
		if (tr::abs(new_pos.x - solid_ptr->position_.x) < deactivate_disp &&
		    tr::abs(new_pos.y - solid_ptr->position_.y) < deactivate_disp &&
		    tr::abs(new_pos.z - solid_ptr->position_.z) < deactivate_disp) {

			// If gravity is non-zero, we only increment the deactivation count if
			// we are actually supported by something (contact or constraint).
			bool supported = false;
			if (gravity_.x == T {} && gravity_.y == T {} && gravity_.z == T {}) {
				supported = true;
			} else {
				for (int k = 0; k < solid_ptr->touch_count_; ++k) {
					// Only a genuinely load-bearing contact counts as support, not
					// mere proximity. Margin-shell discovery (speculative pipeline)
					// records touches for neighbours up to spec_margin_ away; without
					// this gap check a body merely *near* another would be deemed
					// supported and sleep mid-air. Default-pipeline touches always
					// have separation <= 0 (post-push-out), so they still qualify.
					if (solid_ptr->touches_[k].last_tick == current_tick_ &&
					    solid_ptr->touches_[k].separation <= spec_slop_) {
						supported = true;
						break;
					}
				}
				if (!supported && !solid_ptr->constraints_.empty()) {
					supported = true; // existing code below will check if they are loaded
				}
			}

			if (supported) {
				solid_ptr->deactivate_count_++;
				if (solid_ptr->deactivate_count_ > deactivate_count_) {
					int j;
					for (j = static_cast<int>(solid_ptr->constraints_.size()) - 1; j >= 0; --j) {
						auto * con = solid_ptr->constraints_[j];
						auto * start = con->start_solid_.get();
						auto * end = con->end_solid_.get();
						// Stay awake while the constraint is loaded — otherwise both
						// endpoints can sleep at non-equilibrium and freeze the system.
						// is_loaded's argument is a *distance* tolerance (how far the
						// constraint may sit from rest and still count as unloaded), so
						// it must be a small length — epsilon_, not deactivate_speed_.
						// deactivate_speed_ is a velocity; feeding it here let a spring
						// freeze a full deactivate_speed_ away from rest (e.g. 0.2 m
						// past a 2 m rest length) before its endpoints went to sleep.
						if (con->is_loaded(epsilon_))
							break;
						// Never deactivate a body constrained to a static/infinite-mass
						// body — the constraint represents a persistent external force
						// (e.g. spring anchored to the world).
						if (start != solid_ptr) {
							if (start->mass_ == solid<T>::infinite_mass())
								break;
							if (start->active_ && start->deactivate_count_ <= deactivate_count_)
								break;
						} else if (end) {
							if (end->mass_ == solid<T>::infinite_mass())
								break;
							if (end->active_ && end->deactivate_count_ <= deactivate_count_)
								break;
						}
					}
					if (j < 0)
						solid_ptr->deactivate();
				}
			} else {
				solid_ptr->deactivate_count_ = 0;
			}
		} else {
			solid_ptr->deactivate_count_ = 0;
		}
	}
}

// Speculative Pass A: integrate velocity (semi-implicit Euler) and discover
// contacts along the predicted motion WITHOUT moving the body. Each contact is
// recorded in the touch cache with its signed gap (separation), which
// solve_contacts' speculative target consumes. Covers solid-vs-solid contacts
// found via the broad phase plus manager-injected external geometry (queried at
// the end, recorded against the world anchor). Every contact passes through the
// manager's collision_response hook, which can claim it (excluding it from the
// solver) exactly as in the legacy update_solid path.
template <typename T> void simulator<T>::integrate_and_discover(solid<T> * solid_ptr, T dt) {
	const T zero = T {};
	const T one = tr::one();

	// Reset this tick's NGS position correction (commit_solid folds it in for every
	// body, so it must start at zero whether or not the body gets contacts).
	solid_ptr->pos_correction_.reset();

	// Semi-implicit (symplectic) Euler: v += a(x, v)·dt, committed now; position
	// is integrated in Pass B from the *solved* velocity. The high-order
	// integrators (Heun/RK) target accurate ballistic free flight; a contact
	// solver wants the symplectic step so the post-solve velocity is what moves
	// the body.
	vec3<T> accel;
	update_acceleration(accel, solid_ptr, solid_ptr->position_, solid_ptr->velocity_, dt);
	vec3<T> v(solid_ptr->velocity_);
	vec3<T> dv;
	mul(dv, accel, dt);
	add(v, dv);
	cap_vec3(v, max_velocity_component_);
	solid_ptr->velocity_.set(v);
	solid_ptr->clear_force();

	if (manager_)
		manager_->intra_update(solid_ptr, dt);

	if (solid_ptr->collide_with_scope_ == 0)
		return;

	// Predicted motion this frame (pre-solve), used only to aim the discovery
	// sweep — the body is NOT moved here.
	vec3<T> old_pos(solid_ptr->position_);
	cap_vec3(old_pos, max_position_component_);
	vec3<T> delta;
	mul(delta, v, dt);

	// Broad phase over the swept motion plus the speculative margin.
	T reach = tr::max_val(tr::abs(delta.x), tr::max_val(tr::abs(delta.y), tr::abs(delta.z))) + spec_margin_ + epsilon_;
	aa_box<T> box;
	box.set(solid_ptr->local_bound_);
	add(box, old_pos);
	box.mins.x -= reach;
	box.mins.y -= reach;
	box.mins.z -= reach;
	box.maxs.x += reach;
	box.maxs.y += reach;
	box.maxs.z += reach;
	num_spacial_collection_ =
	    find_solids_in_aa_box(box, spacial_collection_.data(), static_cast<int>(spacial_collection_.size()));

	// Sweep the predicted motion. The speculative margin is applied as a uniform
	// shape inflation in the test below (margin-shell discovery), not as a
	// directional path extension — that is what catches the lateral/omnidirectional
	// resting contacts a bare directional sweep misses. The swept path still spans
	// the full predicted motion, preserving anti-tunneling for fast bodies.
	vec3<T> sweep_end;
	add(sweep_end, old_pos, delta);
	segment<T> path;
	path.set_start_end(old_pos, sweep_end);

	const int bits = solid_ptr->collide_with_scope_;
	collision<T> col;

	// Record one discovered contact, or hand it to the manager's custom response.
	// Shared by the broad-phase loop and the manager-geometry query so the signed-gap
	// recovery, collision_response hook, callback recording, touch refresh and wake
	// logic are identical on both. `col` must already hold the hit (time/depth/
	// normal/collider/point); `partner` is the touch partner — a real solid, or
	// static_world_ for manager geometry with no owning solid.
	//
	// collision_response mirrors the legacy update_solid hook: when the manager
	// claims the contact it is NOT recorded, so Pass B applies no impulse — the
	// manager has taken responsibility and may mutate the solid's velocity/position
	// directly. The `position` out-param is honored as a relocation (commit_solid
	// integrates from it); `remainder` is the predicted motion this tick, which is
	// informational here — the speculative path derives the committed position from
	// the *solved* velocity, so there is no slide remainder to consume. (A manager
	// that relocates the body this way should zero its velocity too if it wants it
	// to stay put. Contacts discovered later this tick are not re-traced from the
	// new position, unlike the legacy slide loop, so claim-and-relocate is intended
	// for the destroy / single-contact cases.)
	auto record_contact = [&](solid<T> * partner) {
		const vec3<T> & n = col.normal;
		const bool partner_is_world = (partner == &static_world_);

		// Signed gap along the normal at the body's current (start) position. Shapes
		// were inflated by spec_margin_, so an overlap of the inflated shapes
		// (col.time == 0) means the true surfaces are within the margin: the real gap
		// is (margin - inflated_depth) — 0 when touching, negative when truly
		// penetrating, positive within the shell. A contact reached only by sweeping
		// this tick (col.time > 0) is a fast approach; its gap is the closing distance
		// still to cover.
		T separation;
		if (col.time <= zero) {
			separation = spec_margin_ - col.depth;
		} else {
			vec3<T> to_contact;
			sub(to_contact, col.point, old_pos);
			separation = -dot(to_contact, n);
			if (separation < zero)
				separation = zero;
		}

		// Relative velocity at contact (self - partner), for callback + wake logic.
		if (col.collider)
			sub(col.velocity, solid_ptr->velocity_, col.collider->velocity_);
		else
			col.velocity.set(solid_ptr->velocity_);
		T impact_speed = -dot(col.velocity, n);
		if (impact_speed < zero)
			impact_speed = zero;

		// Callback recording, suppressed on sustained contact (as update_solid).
		typename solid<T>::touch * existing = find_touch(solid_ptr, partner);
		bool sustained = (existing != nullptr && existing->last_tick == current_tick_ - 1);
		if (!sustained && (solid_ptr->collision_callback_ != nullptr ||
		                   (!partner_is_world && partner->collision_callback_ != nullptr))) {
			col.collidee = solid_ptr;
			if (num_collisions_ < static_cast<int>(collisions_.size())) {
				collisions_[num_collisions_].set(col);
				num_collisions_++;
			}
		}

		// Manager-owned custom response (destroy on impact, character controller,
		// bounce pad, ...). A claimed contact is excluded from the solver.
		if (manager_) {
			vec3<T> position(old_pos);
			vec3<T> remainder(delta);
			if (manager_->collision_response(solid_ptr, position, remainder, col)) {
				if (position != old_pos) {
					old_pos.set(position);
					solid_ptr->set_position_direct(position);
				}
				return;  // claimed: no solver touch, no wake
			}
		}

		add_or_refresh_touch(solid_ptr, partner, n, col.impact, impact_speed, separation, current_tick_);

		// Wake a real sleeping partner so it participates in the solve (the world
		// anchor never sleeps).
		if (!partner_is_world &&
		    (partner->collide_with_scope_ & solid_ptr->collision_scope_) != 0 &&
		    partner->should_collide(solid_ptr) && impact_speed > deactivate_speed_) {
			partner->activate();
		}
	};

	for (int i = 0; i < num_spacial_collection_; ++i) {
		auto * s2 = spacial_collection_[i];
		if (s2 == solid_ptr)
			continue;
		if ((bits & s2->collision_scope_) == 0)
			continue;
		if (!solid_ptr->should_collide(s2) || !s2->should_collide(solid_ptr))
			continue;

		col.reset();
		col.time = one;
		test_solid(col, solid_ptr, path, s2, spec_margin_);
		if (col.time >= one && col.depth <= zero)
			continue; // not within the inflated shell and not swept into this tick

		record_contact(s2);
	}

	// --- Manager-injected geometry ---
	// External static geometry the manager owns (level BSP / heightfield / trimesh
	// registered outside hop) is not in solids_, so the broad-phase loop above
	// can't reach it. Query the manager along the same swept path with the
	// speculative margin and fold a hit into the solver the same way: recover the
	// signed gap and refresh a touch. With no owning solid the partner is the
	// immovable world anchor; if the manager attaches a real collider we use it.
	// The manager returns a single merged contact per body per tick (interface
	// limit), so a body resting on a mesh gets one contact point, not a manifold.
	if (manager_) {
		col.reset();
		col.time = one;
		manager_->trace_solid(col, solid_ptr, path, bits, spec_margin_);
		if (col.time < one || col.depth > zero) {
			// No owning solid: resolve against the immovable world anchor. If the
			// manager attached a real collider, prefer it as the partner.
			record_contact(col.collider ? col.collider : &static_world_);
		}
	}
}

// Speculative Pass B: commit position from the solved velocity plus the NGS
// position correction, then run the shared deactivation logic. The correction is
// a pseudo-position (velocity is untouched), so it removes penetration without
// adding energy.
template <typename T> void simulator<T>::commit_solid(solid<T> * solid_ptr, T dt) {
	vec3<T> old_pos(solid_ptr->position_);
	cap_vec3(old_pos, max_position_component_);
	vec3<T> delta;
	mul(delta, solid_ptr->velocity_, dt);
	vec3<T> new_pos;
	add(new_pos, old_pos, delta);
	add(new_pos, solid_ptr->pos_correction_);  // NGS pseudo-position correction
	cap_vec3(new_pos, max_position_component_);

	try_deactivate(solid_ptr, new_pos, dt);
	solid_ptr->set_position_direct(new_pos);
}

// Iterative non-linear Gauss–Seidel position solver. Accumulates a per-body
// pseudo-position correction (solid::pos_correction_, zeroed in Pass A and folded
// into the commit), never touching velocity — so it removes penetration without
// adding energy, unlike a velocity-level Baumgarte term. "Non-linear" because each
// pair's separation is re-derived from the running correction every visit, so the
// corrections from a body's several contacts converge (a floor pushing up and a
// neighbour pushing down see each other) instead of blindly summing — the failure
// mode of a one-shot projection, which over-displaced and ejected bodies.
template <typename T> void simulator<T>::correct_positions() {
	const T zero = T {};
	// Start every participating body from a zero correction. Active bodies were
	// already zeroed in Pass A; this additionally clears sleeping partners (skipped
	// in Pass A) whose stale correction would otherwise corrupt the re-derivation.
	for (auto & p : contact_pairs_) {
		p.a->pos_correction_.reset();
		p.b->pos_correction_.reset();
	}
	for (int iter = 0; iter < spec_pos_iters_; ++iter) {
		for (auto & p : contact_pairs_) {
			// Only an awake speculative body absorbs correction here: sleeping/static
			// bodies and sweep_slide bodies are immovable supports (a sweep_slide body
			// never folds pos_correction_ — it placed itself in Pass A — so giving it a
			// share would silently drop that correction). Zero their inverse mass so a
			// speculative body takes the whole correction against such a partner.
			T inv_a = (p.a->active_ && p.a->uses_speculative_solve()) ? p.inv_ma : zero;
			T inv_b = (p.b->active_ && p.b->uses_speculative_solve()) ? p.inv_mb : zero;
			T inv_sum = inv_a + inv_b;
			if (inv_sum <= zero)
				continue;
			// Current separation = the gap at discovery plus how far the running
			// corrections have already separated this pair along the normal.
			vec3<T> rel;
			sub(rel, p.b->pos_correction_, p.a->pos_correction_);
			T cur_sep = p.separation + dot(rel, p.normal);
			// Only penetration beyond the slop band is corrected.
			T pen = -cur_sep - spec_slop_;
			if (pen <= zero)
				continue;
			T corr = pen * spec_pos_baumgarte_;
			// p.normal points from a toward b: push b along +normal and a along
			// -normal, each by its share of the inverse mass.
			if (inv_b > zero) {
				vec3<T> d;
				mul(d, p.normal, corr * (inv_b / inv_sum));
				add(p.b->pos_correction_, d);
			}
			if (inv_a > zero) {
				vec3<T> d;
				mul(d, p.normal, corr * (inv_a / inv_sum));
				sub(p.a->pos_correction_, d);
			}
		}
	}
}

template <typename T> void simulator<T>::report_collisions() {
	reporting_collisions_ = true;
	for (int i = 0; i < num_collisions_; ++i) {
		auto & col = collisions_[i];
		if (col.collidee) {
			auto & listener = col.collidee->collision_callback_;
			if (listener && col.collider && (col.collidee->collide_with_scope_ & col.collider->collision_scope_) != 0) {
				listener(col);
			}
		}
		if (col.collider) {
			auto & listener = col.collider->collision_callback_;
			if (listener && col.collidee && (col.collider->collide_with_scope_ & col.collidee->collision_scope_) != 0) {
				collision<T> inverted;
				inverted.set(col);
				inverted.invert();
				listener(inverted);
			}
		}
	}
	num_collisions_ = 0;
	reporting_collisions_ = false;
}

template <typename T>
void simulator<T>::trace_segment(collision<T> & result,
                                 const segment<T> & seg,
                                 int collide_with_bits,
                                 solid<T> * ignore) {
	vec3<T> ep;
	seg.get_end_point(ep);
	aa_box<T> total;
	total.set(seg.origin, seg.origin);
	total.merge(ep);
	num_spacial_collection_ =
	    find_solids_in_aa_box(total, spacial_collection_.data(), static_cast<int>(spacial_collection_.size()));
	trace_segment_with_current_spacials(result, seg, collide_with_bits, ignore);
}

template <typename T>
void simulator<T>::trace_solid(collision<T> & result, solid<T> * s, const segment<T> & seg, int collide_with_bits) {
	vec3<T> end;
	seg.get_end_point(end);
	aa_box<T> box;
	box.set(seg.origin, seg.origin);
	box.merge(end);
	add(box.mins, s->local_bound_.mins);
	add(box.maxs, s->local_bound_.maxs);
	num_spacial_collection_ =
	    find_solids_in_aa_box(box, spacial_collection_.data(), static_cast<int>(spacial_collection_.size()));
	trace_solid_with_current_spacials(result, s, seg, collide_with_bits);
}


template <typename T>
void simulator<T>::trace_segment_with_current_spacials(collision<T> & result,
                                                       const segment<T> & seg,
                                                       int collide_with_bits,
                                                       solid<T> * ignore) {
	result.reset();

	collision<T> col;
	for (int i = 0; i < num_spacial_collection_; ++i) {
		auto * s2 = spacial_collection_[i];
		if (s2 != ignore && (collide_with_bits & s2->collision_scope_) != 0) {
			col.time = tr::one();
			test_segment(col, seg, s2);
			hop::merge_collision(result, col, epsilon_, average_normals_);
		}
	}

	if (manager_) {
		col.time = tr::one();
		manager_->trace_segment(col, seg, collide_with_bits);
		hop::merge_collision(result, col, epsilon_, average_normals_);
	}

	if (result.time == tr::one()) {
		seg.get_end_point(result.point);
		result.impact.set(result.point);
	}
}

template <typename T>
void simulator<T>::trace_solid_with_current_spacials(collision<T> & result,
                                                     solid<T> * s,
                                                     const segment<T> & seg,
                                                     int collide_with_bits) {
	result.reset();
	if (collide_with_bits == 0)
		return;

	collision<T> col;
	for (int i = 0; i < num_spacial_collection_; ++i) {
		auto * s2 = spacial_collection_[i];
		if (s != s2 && (collide_with_bits & s2->collision_scope_) != 0 && s->should_collide(s2) &&
		    s2->should_collide(s)) {
			col.time = tr::one();
			test_solid(col, s, seg, s2);
			hop::merge_collision(result, col, epsilon_, average_normals_, &solid_trace_pair_normal_);
		}
	}

	if (manager_) {
		col.time = tr::one();
		manager_->trace_solid(col, s, seg, collide_with_bits, T {});  // exact: swept query / public API
		hop::merge_collision(result, col, epsilon_, average_normals_, &solid_trace_pair_normal_);
	}

	if (result.time == tr::one()) {
		seg.get_end_point(result.point);
		result.impact.set(result.point);
	}
}

template <typename T>
void simulator<T>::constraint_link(vec3<T> & result,
                                   solid<T> * s,
                                   const vec3<T> & solid_pos,
                                   const vec3<T> & solid_vel) {
	vec3<T> start_world;
	vec3<T> end_world;
	vec3<T> tx;
	vec3<T> tv;
	result.reset();

	for (auto * c : s->constraints_) {
		if (!c->is_active())
			continue;

		// Resolve world-space anchor positions and relative velocity.
		// Translation-only: anchor velocity equals solid velocity.
		if (s == c->start_solid_.get()) {
			add(start_world, solid_pos, c->local_anchor_a_);
			if (c->end_solid_) {
				add(end_world, c->end_solid_->position_, c->local_anchor_b_);
				sub(tv, c->end_solid_->velocity_, solid_vel);
			} else {
				end_world = c->end_point_;
				neg(tv, solid_vel);
			}
		} else if (s == c->end_solid_.get()) {
			add(start_world, solid_pos, c->local_anchor_b_);
			add(end_world, c->start_solid_->position_, c->local_anchor_a_);
			sub(tv, c->start_solid_->velocity_, solid_vel);
		} else {
			continue;
		}

		sub(tx, end_world, start_world);
		T dist = length(tx);
		// Spring: any nonzero displacement engages force (the dist > 0 guard
		// just protects the divide). Rope: only stretched past rest engages.
		T gate = (c->type_ == constraint<T>::type::spring) ? T {} : c->rest_length_;
		if (dist > gate) {
			T scale = (dist - c->rest_length_) / dist;
			mul(tx, scale);
		} else {
			tx.reset();
		}

		mul(tx, c->spring_constant_);
		mul(tv, c->damping_constant_);
		add(result, tx);
		add(result, tv);
	}
}

template <typename T>
void simulator<T>::update_acceleration(vec3<T> & result, solid<T> * s, const vec3<T> & x, const vec3<T> & v, T dt) {
	vec3<T> constraint_force;
	vec3<T> fluid_force;
	T zero_val {};
	(void)dt;

	mul(result, gravity_, s->coefficient_of_gravity_);

	if (s->mass_ != zero_val) {
		constraint_link(constraint_force, s, x, v);
		add(constraint_force, s->force_);
		// Contact friction is no longer applied here as a force during
		// integration. It now lives in solve_contacts as a velocity-level
		// constraint with full Coulomb cone clamping (see solve_contacts).
		sub(fluid_force, fluid_velocity_, v);
		mul(fluid_force, s->coefficient_of_effective_drag_);
		add(constraint_force, fluid_force);
		mul(constraint_force, s->inv_mass_);
		add(result, constraint_force);
	}
}

template <typename T>
typename solid<T>::touch * simulator<T>::find_touch(solid<T> * s, solid<T> * partner) {
	for (int i = 0; i < s->touch_count_; ++i) {
		if (s->touches_[i].partner == partner)
			return &s->touches_[i];
	}
	return nullptr;
}

template <typename T>
typename solid<T>::touch * simulator<T>::add_or_refresh_touch(
    solid<T> * s, solid<T> * partner, const vec3<T> & normal, const vec3<T> & impact, T impact_speed, T separation, int tick) {
	const T zero_val {};

	// Refresh-in-place if this partner already has a slot. Sub-step iterations
	// within the same tick (same partner hit twice) take the larger impact
	// speed for restitution. Stale slots whose last refresh predates the
	// previous tick are wiped — the accumulated impulse no longer corresponds
	// to a continuous contact, so warm-starting from it would be unsound.
	for (int i = 0; i < s->touch_count_; ++i) {
		auto & slot = s->touches_[i];
		if (slot.partner == partner) {
			if (slot.last_tick != tick && slot.last_tick != tick - 1) {
				slot.accum_n = zero_val;
				slot.accum_t.reset();
			}
			slot.normal.set(normal);
			slot.impact.set(impact);
			if (slot.last_tick != tick) {
				slot.impact_speed = impact_speed;
			} else if (impact_speed > slot.impact_speed) {
				slot.impact_speed = impact_speed;
			}
			slot.separation = separation;
			slot.last_tick = tick;
			return &slot;
		}
	}

	// New partner: fill an empty slot if available.
	if (s->touch_count_ < solid<T>::max_touches) {
		auto & slot = s->touches_[s->touch_count_++];
		slot.partner = partner;
		slot.normal.set(normal);
		slot.impact.set(impact);
		slot.accum_n = zero_val;
		slot.accum_t.reset();
		slot.impact_speed = impact_speed;
		slot.separation = separation;
		slot.last_tick = tick;
		return &slot;
	}

	// Cache full: evict the slot contributing least to gravity-aligned support
	// (smallest dot(normal, -gravity)), tie-broken by oldest last_tick. For
	// zero-gravity scenes every score is zero so eviction falls back to age.
	int evict = 0;
	T best_score {};
	bool have_score = false;
	for (int i = 0; i < solid<T>::max_touches; ++i) {
		T score = -dot(s->touches_[i].normal, gravity_);
		if (!have_score || score < best_score ||
		    (score == best_score && s->touches_[i].last_tick < s->touches_[evict].last_tick)) {
			evict = i;
			best_score = score;
			have_score = true;
		}
	}
	auto & slot = s->touches_[evict];
	slot.partner = partner;
	slot.normal.set(normal);
	slot.accum_n = zero_val;
	slot.accum_t.reset();
	slot.impact_speed = impact_speed;
	slot.separation = separation;
	slot.last_tick = tick;
	return &slot;
}

template <typename T>
void simulator<T>::solve_contacts(T dt, bool has_speculative) {
	const T zero_val {};
	const T one = tr::one();
	// Used by the speculative target to convert a separation distance into an
	// allowed/required normal velocity (gap or recovery per tick).
	const T inv_dt = (dt > zero_val) ? one / dt : zero_val;

	contact_pairs_.clear();

	// --- 1. Build the canonical pair list ---
	// Walk every active solid's cache exactly once. For each refreshed slot,
	// look up the partner's slot (if any) and build a single canonical pair
	// covering both sides. The pair_built_tick marker on both slots ensures
	// the pair isn't re-added when we reach the partner's slot later in the
	// iteration. Address ordering only matters for which slot acts as "a vs.
	// b"; the slot we iterate from is always the authoritative source for
	// the refreshed normal / impact data, since the partner may not have a
	// refreshed slot (typical for dynamic→static contacts).
	// Flip traversal each tick to match update()'s alternation (cancels the
	// order-induced directional drift; the canonical a<b pair convention is
	// unaffected — only the build/solve order changes).
	const bool flip = (current_tick_ & 1) != 0;
	const int nsolids = static_cast<int>(solids_.size());
	for (int si = 0; si < nsolids; ++si) {
		auto * s = solids_[flip ? nsolids - 1 - si : si].get();
		if (!s->active_)
			continue;
		for (int i = 0; i < s->touch_count_; ++i) {
			auto & slot = s->touches_[i];
			if (slot.last_tick != current_tick_)
				continue;
			if (slot.pair_built_tick == current_tick_)
				continue;
			auto * partner = slot.partner;
			if (!partner)
				continue;

			// Canonicalize by the solids' stable insertion ids so we have a
			// reproducible "a < b" within the pair (a raw pointer compare would
			// reorder run-to-run under ASLR; see solid::solve_id_). The actual
			// slot.normal / accum_t conventions then depend on whether the
			// iterating side ended up as a or b.
			solid<T> * a;
			solid<T> * b;
			bool slot_is_a;
			if (s->solve_id_ < partner->solve_id_) {
				a = s; b = partner; slot_is_a = true;
			} else {
				a = partner; b = s; slot_is_a = false;
			}
			auto * other_slot = find_touch(partner, s);

			contact_pair p;
			p.a = a;
			p.b = b;
			// pair.normal: points from a's free side toward b (i.e., the
			// direction that pushes b away from a when we apply +λ to b's
			// velocity along it).
			//
			// slot.normal (this side, "self") points from partner toward
			// self. If self == a, slot.normal points from b toward a — the
			// opposite of pair.normal. If self == b, slot.normal points
			// from a toward b — same direction.
			if (slot_is_a) {
				neg(p.normal, slot.normal);
			} else {
				p.normal.set(slot.normal);
			}
			p.accum_n = slot.accum_n;
			if (slot_is_a) {
				neg(p.accum_t, slot.accum_t);
			} else {
				p.accum_t.set(slot.accum_t);
			}
			p.impact_speed = slot.impact_speed;
			p.separation = slot.separation;  // carried from the iterating side's slot (same as normal/impact)
			// Combine the two bodies' coefficients of restitution. When the
			// bodies' modes differ the higher-precedence one governs (larger
			// enumerator: average < minimum < multiply < maximum). See the
			// restitution_combine precedence contract in solid.h — notably a
			// `minimum` body is NOT immune to a `maximum` partner.
			const T ca = a->coefficient_of_restitution_;
			const T cb = b->coefficient_of_restitution_;
			switch (a->restitution_combine_ > b->restitution_combine_ ? a->restitution_combine_ : b->restitution_combine_) {
			case restitution_combine::minimum: p.cor = ca < cb ? ca : cb; break;
			case restitution_combine::maximum: p.cor = ca > cb ? ca : cb; break;
			case restitution_combine::multiply: p.cor = ca * cb; break;
			case restitution_combine::average:
			default: p.cor = (ca + cb) * tr::half(); break;
			}
			p.mu_s = (a->coefficient_of_static_friction_ + b->coefficient_of_static_friction_) * tr::half();
			p.mu_d = (a->coefficient_of_dynamic_friction_ + b->coefficient_of_dynamic_friction_) * tr::half();
			p.inv_ma = a->inv_mass_;
			p.inv_mb = b->inv_mass_;
			p.inv_m_sum = p.inv_ma + p.inv_mb;
			p.slot_a = slot_is_a ? &slot : other_slot;
			p.slot_b = slot_is_a ? other_slot : &slot;

			// Angular surface-velocity bias for kinematic carry (Phase 6). The
			// solver drives the relative *surface* velocity (v + ω×r) to its
			// targets, so a spinning kinematic platform drags the riders touching
			// it through the existing non-penetration / friction constraints — no
			// dedicated resolution code. ω is zero on every non-spinning body, so
			// the bias is an exact no-op (and skipped) in translation-only scenes.
			// The contact point comes from the iterating slot (authoritative for
			// normal/impact); for Phase 6 only an infinite-mass platform carries an
			// ω, and that point lies on its surface, so its lever arm is exact.
			p.v_bias.reset();
			const vec3<T> & wa = a->angular_velocity_;
			const vec3<T> & wb = b->angular_velocity_;
			const vec3<T> no_spin {};
			if (!(wa == no_spin && wb == no_spin)) {
				vec3<T> ra, rb, term_a, term_b;
				sub(ra, slot.impact, a->position_);
				sub(rb, slot.impact, b->position_);
				cross(term_a, wa, ra);
				cross(term_b, wb, rb);
				sub(p.v_bias, term_b, term_a);  // ω_b×r_b − ω_a×r_a
			}

			contact_pairs_.push_back(p);
			slot.pair_built_tick = current_tick_;
			if (other_slot)
				other_slot->pair_built_tick = current_tick_;
		}
	}

	if (contact_pairs_.empty())
		return;

	// --- 2. Snapshot pre-solver velocities ---
	// Snapshot vn0 (relative normal velocity before any impulses run this tick)
	// to guard restitution. The restitution target is only activated for pairs
	// that were closing at this moment; a pair that another constraint (or the
	// warm-start boost) has already separated should not receive a bounce
	// impulse based on stale data.
	for (auto & p : contact_pairs_) {
		vec3<T> vrel0;
		sub(vrel0, p.b->velocity_, p.a->velocity_);
		add(vrel0, p.v_bias);  // include kinematic surface velocity (ω×r) for the carry
		p.vn0 = dot(vrel0, p.normal);
		// Restitution target and the λ mass-scale are constant across all GS
		// sweeps for this pair, so derive them once here rather than per
		// iteration in the hot loop below.
		// Unified speculative target, by collision class. It is mode-agnostic: a
		// sweep_slide body has already been snapped to its contact, so its recorded
		// gap is <= slop and only the first and third branches can fire — exactly the
		// legacy `(-vn0 > micro) ? -cor*vn0 : 0` target. A speculative body can also
		// carry a positive gap (margin-shell discovery) and take the middle branch.
		//  - genuinely closing (vn0 below -micro): restitution bounce, even with a
		//    gap still remaining (discovery is speculative, so a fast body is found
		//    before it reaches the surface);
		//  - slow approach with a gap: cap so the body closes at most the gap this
		//    tick (vn >= -(gap - slop)/dt), stopping fast bodies short of tunneling,
		//    with no bounce;
		//  - at/inside the contact and not closing (resting pile): drive vn to 0.
		// Existing penetration is removed positionally by correct_positions (NGS),
		// not here, so the velocity solve adds no energy.
		const T gap = p.separation;
		if (-p.vn0 > micro_collision_threshold_) {
			// Genuine closing collision: bounce at cor times the inbound speed even
			// when a speculative gap still remains. Withholding restitution until
			// gap<=slop (as this once did) let the approach cap below bleed off the
			// entire inbound velocity first, so the body reached the surface with
			// ~zero closing speed and never bounced. cor<=1 keeps |target|<=|vn0|, so
			// restitution stays dissipative; the micro threshold lets a slow,
			// settling body fall through to the cap.
			p.target = -p.cor * p.vn0;
		} else if (gap > spec_slop_) {
			p.target = -(gap - spec_slop_) * inv_dt;
		} else {
			p.target = zero_val;
		}
		p.friction_scale = (p.inv_m_sum > zero_val) ? -one / p.inv_m_sum : zero_val;
	}

	// --- 3. Warm-start ---
	// In hop bodies are stateful; their velocity already includes the effect of
	// last tick's impulses. Warm-starting is therefore incremental: we keep
	// p.accum_n / p.accum_t from the cache as GS starting points but DO NOT
	// re-apply them to velocity (that would double-count).
	//
	// Exception: a pair that is *separating* this tick (vn0 > 0) carries no
	// sustained load — typically the tick right after a bounce. The clamp
	// new_acc >= 0 normally stops the normal constraint from pulling bodies
	// together, but a non-zero warm-started accum_n gives the GS a budget it can
	// spend pulling the pair back: with target 0 and a separating vn, lambda_n is
	// negative, and as long as accum_n stays >= 0 that negative impulse is applied
	// — cancelling the separation and freezing the body it just bounced off of.
	// (Most visible in fixed-point, where the post-bounce snap can land a hair
	// inside the contact, so it gets re-recorded while separating.) Drop the warm
	// start for separating pairs so they can only push, never pull back.
	for (auto & p : contact_pairs_) {
		if (p.vn0 > zero_val) {
			p.accum_n = zero_val;
			p.accum_t.reset();
		}
	}

	// --- 4. Gauss–Seidel sweeps ---
	// Each iteration solves the normal constraint (clamped accumulated
	// impulse ≥ 0 → no sticky pull) and then the friction constraint
	// (Coulomb cone clamped to the current accumulated normal). Order is
	// normal-then-friction so friction sees a meaningful normal load even
	// on the very first iteration.
	// Apply a constraint impulse `delta` (a-side convention) split across the
	// pair by inverse mass — the split-impulse step shared by both sweeps. The
	// inverse masses are passed explicitly (rather than read from the pair) so the
	// shock-propagation phase can substitute effective masses that zero out frozen
	// anchors.
	auto apply_pair_impulse = [zero_val](contact_pair & p, const vec3<T> & delta, T inv_a, T inv_b) {
		if (inv_a > zero_val) {
			vec3<T> da;
			mul(da, delta, -inv_a);
			add(p.a->velocity_, da);
		}
		if (inv_b > zero_val) {
			vec3<T> db;
			mul(db, delta, inv_b);
			add(p.b->velocity_, db);
		}
	};

	// One clamped normal-constraint solve for a pair: drive the relative normal
	// velocity to p.target, keep the accumulated impulse non-negative (no sticky
	// pull), and apply the resulting split impulse. Shared by the Gauss–Seidel
	// normal sweep (cached masses) and the shock-propagation walk (effective masses
	// with frozen anchors zeroed). inv_sum is passed in since the caller already
	// has it / needs to guard against the immovable-pair (inv_sum == 0) case.
	auto solve_normal = [&apply_pair_impulse, zero_val](contact_pair & p, T inv_a, T inv_b, T inv_sum) {
		vec3<T> vrel;
		sub(vrel, p.b->velocity_, p.a->velocity_);
		add(vrel, p.v_bias);  // surface velocity (ω×r): the kinematic carry term
		T vn = dot(vrel, p.normal);
		T lambda_n = (p.target - vn) / inv_sum;
		T new_acc = p.accum_n + lambda_n;
		if (new_acc < zero_val)
			new_acc = zero_val;
		T effective = new_acc - p.accum_n;
		p.accum_n = new_acc;
		if (effective != zero_val) {
			vec3<T> delta;
			mul(delta, p.normal, effective);
			apply_pair_impulse(p, delta, inv_a, inv_b);
		}
	};

	// Restitution note: p.target separates the pair at `cor` times the relative
	// normal velocity it was *actually* closing at when the solver began (vn0,
	// measured along this same pair normal). Because |target| = cor·|vn0| ≤ |vn0|
	// for cor ≤ 1, the post-solve separation can never exceed the approach —
	// restitution is guaranteed dissipative. Earlier this targeted cor·impact_speed
	// (max approach over the tick's sub-steps, per-contact TOI normal); that can
	// exceed |vn0|, giving an effective COR > 1 that compounded across a deep
	// frictionless pile and ran energy away (KE → 1e7, balls flung through the
	// floor). impact_speed is still recorded for wake/callback logic; it must not
	// drive the restitution target.
	const int npairs = static_cast<int>(contact_pairs_.size());
	for (int iter = 0; iter < solver_iterations_; ++iter) {
		// Normal sweep (flip direction each tick — see the build-loop comment)
		for (int k = 0; k < npairs; ++k) {
			auto & p = contact_pairs_[flip ? npairs - 1 - k : k];
			if (p.inv_m_sum <= zero_val)
				continue;
			solve_normal(p, p.inv_ma, p.inv_mb, p.inv_m_sum);
		}
		// Friction sweep (same per-tick flip)
		for (int k = 0; k < npairs; ++k) {
			auto & p = contact_pairs_[flip ? npairs - 1 - k : k];
			if (p.inv_m_sum <= zero_val)
				continue;
			if (p.accum_n <= zero_val || (p.mu_s <= zero_val && p.mu_d <= zero_val))
				continue;
			vec3<T> vrel;
			sub(vrel, p.b->velocity_, p.a->velocity_);
			add(vrel, p.v_bias);  // surface velocity (ω×r): friction drags the rider to match the spin
			T vn = dot(vrel, p.normal);
			vec3<T> vn_vec;
			mul(vn_vec, p.normal, vn);
			vec3<T> vt;
			sub(vt, vrel, vn_vec);  // tangential relative velocity
			vec3<T> lambda_t;
			mul(lambda_t, vt, p.friction_scale);
			vec3<T> new_accum_t;
			add(new_accum_t, p.accum_t, lambda_t);
			// Coulomb stick/slip: the contact holds (static) as long as the
			// tangential impulse needed to cancel sliding stays within the
			// static cone mu_s·N. Once that's exceeded the contact slips and
			// friction drops to the kinetic cone mu_d·N. The `max_d < mag` guard
			// scales down only — it skips the degenerate mu_d > mu_s case, which
			// would otherwise scale the impulse *up* and inject energy.
			T mag = length(new_accum_t);
			T max_s = p.mu_s * p.accum_n;
			if (mag > max_s && mag > zero_val) {
				T max_d = p.mu_d * p.accum_n;
				if (max_d < mag)
					mul(new_accum_t, max_d / mag);
			}
			vec3<T> delta;
			sub(delta, new_accum_t, p.accum_t);
			p.accum_t.set(new_accum_t);
			if (length_squared(delta) > zero_val) {
				apply_pair_impulse(p, delta, p.inv_ma, p.inv_mb);
			}
		}
	}

	// --- 4b. Shock propagation ---
	// Plain Gauss–Seidel transmits one layer of support per sweep, so an N-deep
	// pile needs ~N sweeps before the bottom contacts carry the weight above them.
	// With a fixed iteration budget a tall pile stays under-supported: it
	// compresses into penetration (sinks), and the never-zeroed normal velocities
	// keep the bottom layer churning instead of sleeping. Shock propagation fixes
	// this in O(1) sweeps: walk the contacts from the support end up the gravity
	// chain and, as each body's support-from-below is resolved, freeze it into a
	// rigid anchor (effective inverse mass 0) so the body above solves against firm
	// ground and its reaction can't shove the support back down. Normal-only —
	// friction and restitution are already handled by the GS loop above.
	// Speculative path only; needs a gravity direction to define "down" (a zero-g
	// gas has no stacking chain to propagate, so skip it).
	const bool have_gravity =
	    gravity_.x != zero_val || gravity_.y != zero_val || gravity_.z != zero_val;
	if (has_speculative && spec_shock_iters_ > 0 && npairs > 0 && have_gravity) {
		// A body is a standing anchor from the outset if it can't move (static /
		// infinite mass) or is asleep (a settled lower layer supporting the rest).
		auto pre_frozen = [zero_val](const solid<T> * s, T inv_m) {
			return inv_m <= zero_val || !s->active_;
		};
		// "Support depth" along gravity: dot(position, gravity_) grows the further a
		// body sits down the gravity vector, so the deeper body of a pair is the one
		// nearer the anchor (it supports the other). Raw dot — unnormalized gravity
		// scales every depth equally, leaving the ordering unchanged. Positions are
		// frozen during the velocity solve, so each pair's depth key and deeper-body
		// ("lo") are computed once here and reused by both the sort and every pass.
		shock_order_.resize(npairs);
		shock_key_.resize(npairs);
		shock_lo_.resize(npairs);
		for (int k = 0; k < npairs; ++k) {
			const contact_pair & p = contact_pairs_[k];
			T da = dot(p.a->position_, gravity_);
			T db = dot(p.b->position_, gravity_);
			shock_order_[k] = k;
			shock_key_[k] = tr::max_val(da, db);
			shock_lo_[k] = da >= db ? p.a : p.b;
		}
		// Order contacts support-end-first: descending depth, so a body's contacts
		// from below are visited before the contacts it supports from above.
		// stable_sort keeps the canonical pair order for equal depths, so the sweep
		// stays deterministic (fixed-point included).
		std::stable_sort(shock_order_.begin(), shock_order_.end(),
		                 [this](int i, int j) { return shock_key_[i] > shock_key_[j]; });
		for (int pass = 0; pass < spec_shock_iters_; ++pass) {
			// Re-seed the freeze state each pass: only the standing anchors start
			// frozen; the support-end-first walk re-freezes the rest against the
			// updated velocities.
			for (auto & p : contact_pairs_) {
				p.a->solve_frozen_ = pre_frozen(p.a, p.inv_ma);
				p.b->solve_frozen_ = pre_frozen(p.b, p.inv_mb);
			}
			for (int oi = 0; oi < npairs; ++oi) {
				int idx = shock_order_[oi];
				contact_pair & p = contact_pairs_[idx];
				// The deeper (more-anchored) body has had its support-from-below
				// resolved by now — earlier in the walk — so freeze it: it anchors
				// this contact and everything above it. Frozen bodies contribute zero
				// effective inverse mass, so the solve drives the free body alone and
				// its reaction can't shove the support back down.
				shock_lo_[idx]->solve_frozen_ = true;
				T inv_a = p.a->solve_frozen_ ? zero_val : p.inv_ma;
				T inv_b = p.b->solve_frozen_ ? zero_val : p.inv_mb;
				T inv_sum = inv_a + inv_b;
				if (inv_sum <= zero_val)
					continue;
				solve_normal(p, inv_a, inv_b, inv_sum);
			}
		}
	}

	// --- 5. Writeback ---
	// Store the converged per-pair impulses back into both sides' cache slots
	// so next tick's warm-start picks up where we left off.
	for (auto & p : contact_pairs_) {
		if (p.slot_a) {
			p.slot_a->accum_n = p.accum_n;
			neg(p.slot_a->accum_t, p.accum_t);
		}
		if (p.slot_b) {
			p.slot_b->accum_n = p.accum_n;
			p.slot_b->accum_t.set(p.accum_t);
		}
	}

	// --- 6. Cap solver-mutated velocities, then wake perturbed partners ---
	// Cap: the per-body velocity cap in update_solid runs at integration time —
	// a full tick before it would catch anything solve_contacts produced here.
	// Without it a Gauss–Seidel overshoot (deep/stiff pile, near-singular
	// inv_m_sum, or fixed-point round-off) ships into the next tick's integration
	// at full magnitude and can tunnel a body through geometry. Capping after the
	// sweeps (rather than inside apply_pair_impulse, which runs per impulse) keeps
	// the solver's output bounded like integration's for one cheap idempotent pass.
	// Wake: a pile near rest can be perturbed by a single ball landing on top of
	// it; without this the perturbed ball pushes the pile down via the solver but
	// the (sleeping) members never re-enter update_solid to notice their new
	// velocity. Both run over the same pair list, so do them in one walk; capping
	// first means the wake threshold reads the final (capped) velocity.
	const bool do_cap = max_velocity_component_ > zero_val;
	auto finalize = [this, do_cap](solid<T> * s, T inv_m) {
		if (inv_m <= T{})
			return;
		if (do_cap)
			cap_vec3(s->velocity_, max_velocity_component_);
		if (!s->active_ &&
		    (tr::abs(s->velocity_.x) > deactivate_speed_ ||
		     tr::abs(s->velocity_.y) > deactivate_speed_ ||
		     tr::abs(s->velocity_.z) > deactivate_speed_)) {
			s->activate();
		}
	};
	for (auto & p : contact_pairs_) {
		finalize(p.a, p.inv_ma);
		finalize(p.b, p.inv_mb);
	}
}

template <typename T>
void simulator<T>::integration_step(solid<T> * s,
                                    const vec3<T> & x,
                                    const vec3<T> & v,
                                    const vec3<T> & dx,
                                    const vec3<T> & dv,
                                    T dt,
                                    vec3<T> & result_x,
                                    vec3<T> & result_v) {
	vec3<T> tx;
	vec3<T> tv;
	mul(tx, dx, dt);
	add(tx, x);
	mul(tv, dv, dt);
	add(tv, v);
	result_x.set(tv);
	update_acceleration(result_v, s, tx, tv, dt);
}

} // namespace hop
