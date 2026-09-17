#pragma once

#include <hop/math/quat.h>
#include <hop/math/support.h>
#include <memory>

namespace hop {

template <typename T> class solid;
template <typename T> class simulator;

// Swing-twist decomposition of a relative orientation about local +X.
//
// Splits `q_rel` — the child's joint frame expressed in the parent's — into the part
// that tilts the twist axis away from +X (the swing, a cone) and the part that spins
// about it (the twist, a hinge). Every angular joint limit is one or the other, and
// the two are independent, which is the only reason a cone limit and a twist limit can
// be two separate one-dimensional rows rather than one coupled mess.
//
// `swing` comes back in [0, pi] with a unit `swing_axis` that is exactly perpendicular
// to +X by construction (the x component of q_swing cancels algebraically), and `twist`
// signed in [-pi, pi] about +X. Both are expressed in the PARENT's joint frame; the
// caller rotates them into world.
//
// +X as the twist axis is Bullet's convention and therefore Godot's, so a cone-twist
// built for GodotPhysics3D or Jolt means the same thing here and the editor's gizmo
// still points where the limit actually is.
template <typename T>
inline void decompose_swing_twist(const quat<T> & q_rel,
                                  T & swing,
                                  vec3<T> & swing_axis,
                                  T & twist,
                                  T epsilon) {
	using tr = scalar_traits<T>;
	quat<T> q(q_rel);
	// -q is the same rotation; canonicalizing to w >= 0 picks the short way round, so a
	// joint 1 degree from its rest pose never reads as 359 degrees of swing.
	if (q.w < T {})
		neg(q);
	quat<T> q_twist(q.x, T {}, T {}, q.w);
	// Degenerate at a swing of pi, where the twist axis has been folded onto its own
	// negative and the split is genuinely undefined. Call it no twist and let the swing
	// limit — which is wide awake at pi — be the one that answers.
	if (!normalize_carefully(q_twist, epsilon))
		q_twist.reset();
	quat<T> q_twist_inv, q_swing;
	conjugate(q_twist_inv, q_twist);  // unit, so the conjugate is the inverse
	mul(q_swing, q, q_twist_inv);
	swing = get_axis_angle_from_quat(swing_axis, q_swing, epsilon);
	twist = tr::atan2(q_twist.x, q_twist.w) * tr::two();
}

// The child's joint frame expressed in the parent's — exactly the input
// decompose_swing_twist wants — plus the parent's joint frame in world, which a caller
// needs to rotate the returned axes out of joint space. `orient_b` is null when the far end
// is a fixed world point, whose frame IS world.
//
// Six lines, but they live here rather than being written twice: the solver derives a
// joint's swing and twist to enforce them, and constraint::measure_limits derives the same
// numbers for is_loaded and for what the tests and demo tables report. If those two ever
// disagreed about the frame convention, the corpse would be held to limits the tests could
// not see.
template <typename T>
inline void joint_relative_orientation(quat<T> & parent_world,
                                       quat<T> & q_rel,
                                       const quat<T> & orient_a,
                                       const quat<T> & frame_a,
                                       const quat<T> * orient_b,
                                       const quat<T> & frame_b) {
	quat<T> qb, qa_inv;
	mul(parent_world, orient_a, frame_a);
	if (orient_b)
		mul(qb, *orient_b, frame_b);
	else
		qb = frame_b;
	conjugate(qa_inv, parent_world);
	mul(q_rel, qa_inv, qb);
}

template <typename T> class constraint {
public:
	using ptr = std::shared_ptr<constraint<T>>;
	using tr = scalar_traits<T>;

	// Behavior of the distance term.
	//   spring: bilateral. Force = k * (|d| - rest) along d. Pulls when stretched, pushes when compressed.
	//   rope:   unilateral. Force kicks in only when |d| > rest (max-length leash).
	//   rigid:  bilateral and ENFORCED — a ball-socket pin, not a force at all. The two
	//           anchors are held coincident (rest_length_ takes no part in it) by the
	//           simulator's Pass-B solver: it drives the full 3-DOF relative anchor
	//           velocity to zero and pushes the residual separation out positionally.
	//           This is what a jointed chain needs. A force spring hung with limbs off it
	//           sags — it needs a nonzero stretch to produce any force at all — and
	//           stiffening k to hide the sag is what makes an explicitly-integrated
	//           spring chain ring and then leave.
	//
	//           Needs contact_mode::speculative on its bodies. A sweep_slide body commits
	//           its position in Pass A, before the solver runs, so the joint impulse
	//           would only reach it a tick late and there would be no position pass to
	//           take the residual out. hop-godot puts every RIGID body on speculative.
	enum class type { spring, rope, rigid };

	constraint() { reset(); }

	constraint(std::shared_ptr<solid<T>> start, std::shared_ptr<solid<T>> end) {
		reset();
		set_start_solid(start);
		set_end_solid(end);
	}

	constraint(std::shared_ptr<solid<T>> start, const vec3<T> & end_point) {
		reset();
		set_start_solid(start);
		set_end_point(end_point);
	}

	void destroy() {
		if (start_solid_) {
			start_solid_->activate();
			start_solid_->internal_remove_constraint(this);
			start_solid_ = nullptr;
		}
		if (end_solid_) {
			end_solid_->activate();
			end_solid_->internal_remove_constraint(this);
			end_solid_ = nullptr;
		}
	}

	void reset() {
		destroy();
		type_ = type::rope;
		rest_length_ = tr::one();
		spring_constant_ = tr::one();
		damping_constant_ = tr::one();
		bias_ = tr::from_milli(300);   // Godot's PIN_JOINT_BIAS default
		impulse_clamp_ = T {};         // uncapped
		frame_a_.reset();
		frame_b_.reset();
		swing_span_ = -tr::one();      // negative = unlimited; a plain pin
		twist_span_ = -tr::one();
		limit_bias_ = tr::from_milli(300);        // Godot's CONE_TWIST_JOINT_BIAS default
		limit_softness_ = tr::from_milli(800);    // ...SOFTNESS
		limit_relaxation_ = tr::one();            // ...RELAXATION
		settle_ticks_ = 30;                       // half a second at 60 Hz
		limit_watch_.clear();
		local_anchor_a_.reset();
		local_anchor_b_.reset();
		end_point_.reset();
		simulator_ = nullptr;
	}

	void set_type(type t) {
		type_ = t;
		activate_endpoints();
	}
	type get_type() const { return type_; }

	void set_start_solid(std::shared_ptr<solid<T>> s) {
		activate_endpoints();
		if (start_solid_) {
			start_solid_->internal_remove_constraint(this);
			start_solid_ = nullptr;
		}
		if (s) {
			s->internal_add_constraint(this);
			s->activate();
			start_solid_ = s;
		}
	}
	solid<T> * get_start_solid() const { return start_solid_.get(); }

	void set_end_solid(std::shared_ptr<solid<T>> s) {
		activate_endpoints();
		if (end_solid_) {
			end_solid_->internal_remove_constraint(this);
			end_solid_ = nullptr;
		}
		if (s) {
			s->internal_add_constraint(this);
			s->activate();
			end_solid_ = s;
		}
	}
	solid<T> * get_end_solid() const { return end_solid_.get(); }

	void set_end_point(const vec3<T> & p) {
		activate_endpoints();
		if (end_solid_) {
			end_solid_->internal_remove_constraint(this);
			end_solid_ = nullptr;
		}
		end_point_ = p;
	}
	const vec3<T> & get_end_point() const { return end_point_; }

	// Anchor offset in the solid's local frame; default (0,0,0) is the solid's center.
	void set_local_anchor_a(const vec3<T> & a) {
		local_anchor_a_ = a;
		activate_endpoints();
	}
	const vec3<T> & get_local_anchor_a() const { return local_anchor_a_; }

	void set_local_anchor_b(const vec3<T> & a) {
		local_anchor_b_ = a;
		activate_endpoints();
	}
	const vec3<T> & get_local_anchor_b() const { return local_anchor_b_; }

	// For spring: natural length where force is zero.
	// For rope:   maximum length before pull-only force engages.
	void set_rest_length(T r) {
		rest_length_ = r;
		activate_endpoints();
	}
	T get_rest_length() const { return rest_length_; }

	void set_spring_constant(T c) { spring_constant_ = c; }
	T get_spring_constant() const { return spring_constant_; }
	// For spring/rope: the damping force per unit relative anchor speed (N per m/s).
	// For rigid:      the FRACTION of the residual relative anchor velocity the solver
	//                 removes per iteration — dimensionless, 1 = fully rigid. Godot's
	//                 PIN_JOINT_DAMPING, which also defaults to 1.
	void set_damping_constant(T c) { damping_constant_ = c; }
	T get_damping_constant() const { return damping_constant_; }

	// --- rigid only; ignored by spring and rope ---
	// Fraction of the remaining anchor separation the position pass removes per
	// iteration. Godot's PIN_JOINT_BIAS.
	void set_bias(T b) { bias_ = b; }
	T get_bias() const { return bias_; }
	// Cap on the magnitude of the impulse one solver iteration may apply; 0 = uncapped.
	// Godot's PIN_JOINT_IMPULSE_CLAMP, and the safety valve that turns a solver blow-up
	// into a visibly floppy joint rather than a body launched out of the map.
	void set_impulse_clamp(T c) { impulse_clamp_ = c; }
	T get_impulse_clamp() const { return impulse_clamp_; }

	// --- rigid only: angular limits, which turn the pin into a cone-twist ---
	//
	// A cone-twist IS a ball-socket plus limits, so it lives on the same constraint
	// rather than beside it: two objects over one pair of bodies would double the rows
	// and then fight over them.
	//
	// The joint's rest frame in each body's local space. The joint is satisfied — zero
	// swing, zero twist — when R_a*frame_a and R_b*frame_b coincide. Default identity,
	// which is what a pin built before any of this existed gets.
	//
	// Aim these deliberately: a cone is symmetric and an elbow is not. Rotating frame_a
	// so the cone is CENTRED on the middle of the arc a joint actually travels turns a
	// symmetric primitive into a one-sided hinge, and that is the difference between a
	// corpse with knees and a corpse with tentacles.
	void set_frame_a(const quat<T> & q) { frame_a_ = q; }
	void set_frame_a(const mat3<T> & m) { set_quat_from_mat3(frame_a_, m); }
	const quat<T> & get_frame_a() const { return frame_a_; }
	void set_frame_b(const quat<T> & q) { frame_b_ = q; }
	void set_frame_b(const mat3<T> & m) { set_quat_from_mat3(frame_b_, m); }
	const quat<T> & get_frame_b() const { return frame_b_; }

	// Half-angle of the cone the child's twist axis may tilt through, and how far it may
	// spin about that axis, both in radians. NEGATIVE MEANS NO LIMIT, which is what keeps
	// every pin built before Phase 13 bit-identical.
	void set_swing_span(T s) {
		swing_span_ = s;
		activate_endpoints();
	}
	T get_swing_span() const { return swing_span_; }
	void set_twist_span(T s) {
		twist_span_ = s;
		activate_endpoints();
	}
	T get_twist_span() const { return twist_span_; }
	bool has_limits() const { return swing_span_ >= T {} || twist_span_ >= T {}; }

	// Godot's three cone-twist knobs.
	//   BIAS       — fraction of the angular violation the position pass removes per
	//                iteration, the same meaning bias_ has for the pin.
	//   SOFTNESS   — the fraction of the span at which the limit STARTS to resist,
	//                rather than switching on hard at the boundary. Inside softness*span
	//                the limit does nothing at all; between there and the span it damps
	//                the approach without pushing back positionally; past the span it is
	//                a full unilateral stop. This is what makes a limit read as flesh
	//                instead of a detent.
	//   RELAXATION — a scale on the velocity impulse.
	void set_limit_bias(T b) { limit_bias_ = b; }
	T get_limit_bias() const { return limit_bias_; }
	void set_limit_softness(T s) { limit_softness_ = s; }
	T get_limit_softness() const { return limit_softness_; }
	void set_limit_relaxation(T r) { limit_relaxation_ = r; }
	T get_limit_relaxation() const { return limit_relaxation_; }

	// How many consecutive ticks a limit may fail to make any progress on its violation
	// before the solver calls it SETTLED: its way back into its cone is BLOCKED — by a
	// floor, by the pile the corpse landed in, by the weight of the chain hanging off it —
	// and no amount of pushing is going to move it. Before this it did not stop trying: it
	// shoved at up to the recovery cap every tick for as long as the joint existed, which
	// is what made a landed ragdoll shiver where it lay and never sleep.
	//
	// A settled limit keeps its stop — it still refuses to let the joint open any further.
	// What it gives up is the RECOVERY and its claim on the sleep test, which is exactly
	// how hop already treats a body resting on a floor: held, but not working.
	//
	// Zero disables the rule, which is what hop did before this existed.
	void set_settle_ticks(int t) { settle_ticks_ = t > 0 ? t : 0; }
	int get_settle_ticks() const { return settle_ticks_; }
	bool limit_settled() const { return limit_watch_.settled; }

	// Settling is a property of a pair that has stopped moving, so anything that gets the
	// pair moving again re-opens the question — solid::activate calls this when a body wakes.
	void clear_settle() { limit_watch_.clear(); }

	// Bookkeeping for the rule above, called once per tick by the solver with the violation
	// it just measured (zero when the joint is inside its cone, slop included). The question
	// is asked once per WINDOW rather than once per tick, because a limit that is losing does
	// not sit still while it loses — it oscillates, and a tick-by-tick "is that better than
	// last tick?" reads every upswing as a fresh start and never concludes anything.
	void note_limit_violation(T violation) {
		limit_watch_.note(violation, settle_ticks_, tr::from_milli(2));  // ~0.1 degree
	}

	bool is_active() const { return simulator_ != nullptr; }

	// Current swing and twist of this joint, in radians, measured between the two rest
	// frames. Returns false (and leaves the outputs alone) when the constraint has no
	// start solid to measure from. Public because the demos and tests grade a corpse on
	// it, and because is_loaded needs the same numbers.
	bool measure_limits(T & swing, T & twist, T epsilon) const {
		if (!start_solid_)
			return false;
		quat<T> parent_world, q_rel;
		joint_relative_orientation(parent_world, q_rel,
		                           start_solid_->get_orientation_quat(), frame_a_,
		                           end_solid_ ? &end_solid_->get_orientation_quat() : nullptr,
		                           frame_b_);
		vec3<T> axis;
		decompose_swing_twist(q_rel, swing, axis, twist, epsilon);
		return true;
	}

	// True if the constraint's length sits more than `tolerance` from where it
	// produces no force (rest length for a spring; rest length on the long side
	// for a rope). `tolerance` is a *distance*, not a force or a speed — pass a
	// small length such as the simulator's epsilon. Used by the sleep heuristic
	// so both endpoints don't freeze at a non-equilibrium displacement.
	bool is_loaded(T tolerance) const {
		if (!start_solid_)
			return false;
		vec3<T> a_world;
		vec3<T> b_world;
		vec3<T> a_lever;
		// Anchors rotate with their solid (Phase 10); zero anchor → position_.
		mul(a_lever, start_solid_->orientation_, local_anchor_a_);
		add(a_world, start_solid_->position_, a_lever);
		if (end_solid_) {
			vec3<T> b_lever;
			mul(b_lever, end_solid_->orientation_, local_anchor_b_);
			add(b_world, end_solid_->position_, b_lever);
		} else {
			b_world = end_point_;
		}
		T d2 = length_squared(a_world, b_world);
		// A rigid pin holds its anchors coincident, so its equilibrium is zero separation
		// whatever rest_length_ says. This is the whole reason a ragdoll can sleep: a
		// SATISFIED pin sits at ~zero error and reads unloaded, where a spring holding a
		// limb up against gravity is loaded BY DEFINITION (no stretch, no force) and
		// would keep every bone awake for the corpse's whole lifetime.
		if (type_ == type::rigid) {
			if (d2 > tolerance * tolerance)
				return true;
			// A joint RESTING on its limit is a body resting on a floor: held, but not
			// working, and it must be allowed to sleep. So an engaged limit counts as
			// load only while it is still being VIOLATED by more than a degree — get this
			// backwards and every corpse with an arm against its stop stays awake for its
			// whole lifetime. The slop is an angle, not `tolerance`, which is a distance;
			// a degree is far below what anyone can see and far above what the position
			// pass leaves behind.
			if (!has_limits())
				return false;
			T swing {}, twist {};
			if (!measure_limits(swing, twist, tolerance))
				return false;
			// A limit nothing can win is furniture too: it is held out of its cone by
			// something the solver cannot move, so it is no more "working" than a crate
			// resting on a floor is. Reporting it as load forever is what kept every
			// bone of a landed corpse awake for the corpse's whole lifetime.
			if (limit_settled())
				return false;
			const T slop = tr::from_milli(17);  // ~1 degree
			if (swing_span_ >= T {} && swing > swing_span_ + slop)
				return true;
			if (twist_span_ >= T {} && tr::abs(twist) > twist_span_ + slop)
				return true;
			return false;
		}
		T hi = rest_length_ + tolerance;
		T hi2 = hi * hi;
		if (type_ == type::spring) {
			T lo = rest_length_ - tolerance;
			if (lo <= T {})
				return d2 > hi2;
			return d2 < lo * lo || d2 > hi2;
		}
		return d2 > hi2;
	}

private:
	void activate_endpoints() {
		if (start_solid_)
			start_solid_->activate();
		if (end_solid_)
			end_solid_->activate();
	}

	void internal_set_simulator(simulator<T> * s) { simulator_ = s; }

	type type_ {};
	std::shared_ptr<solid<T>> start_solid_;
	std::shared_ptr<solid<T>> end_solid_;
	vec3<T> local_anchor_a_;
	vec3<T> local_anchor_b_;
	vec3<T> end_point_;

	T rest_length_ {};
	T spring_constant_ {};
	T damping_constant_ {};
	T bias_ {};
	T impulse_clamp_ {};
	quat<T> frame_a_;
	quat<T> frame_b_;
	T swing_span_ {};
	T twist_span_ {};
	T limit_bias_ {};
	T limit_softness_ {};
	T limit_relaxation_ {};
	int settle_ticks_ = 0;
	// One window's worth of "is this error going anywhere?", kept per error kind.
	struct progress_watch {
		bool settled = false;
		int count = 0;
		T window_error {};

		void clear() {
			settled = false;
			count = 0;
			window_error = T {};
		}

		void note(T error, int window, T progress) {
			if (error <= T {}) {   // no error at all: nothing to be stuck on
				clear();
				return;
			}
			if (window <= 0) {     // rule off
				settled = false;
				return;
			}
			if (count == 0) {
				window_error = error;
				count = 1;
				return;
			}
			if (++count < window)
				return;
			settled = !(error < window_error - progress);
			window_error = error;
			count = 1;
		}
	};
	progress_watch limit_watch_;

	simulator<T> * simulator_ = nullptr;

	friend class solid<T>;
	friend class simulator<T>;
};

} // namespace hop
