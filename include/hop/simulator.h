#pragma once

#include <algorithm>
#include <cassert>
#include <hop/collision.h>
#include <hop/constraint.h>
#include <hop/manager.h>
#include <hop/math/bounding.h>
#include <hop/math/intersect.h>
#include <hop/math/support.h>
#include <hop/math/project.h>
#include <hop/solid.h>
#include <stdexcept>
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

	// Bit OR'd into the `scope` argument of update(dt, scope) to opt into
	// dispatching collision callbacks for this tick. Reuses the scope mask
	// because no solid would set its scope_ to bit 30 in practice.
	enum { scope_report_collisions = 1 << 30 };

	simulator() {
		set_gravity({ T {}, T {}, -tr::from_milli(9810) });
		init_epsilon_defaults();
		collisions_.resize(64);
	}

	~simulator() = default;

	// Epsilon
	void set_epsilon(T epsilon) {
		static_assert(std::is_same_v<T, float>, "set_epsilon only for float; use set_epsilon_bits for fixed16");
		tr::make_epsilon(epsilon_state_, epsilon);
		epsilon_ = epsilon_state_.epsilon;
		half_epsilon_ = epsilon_state_.half_epsilon;
		quarter_epsilon_ = epsilon_state_.quarter_epsilon;
	}

	void set_epsilon_bits(int bits) {
		static_assert(std::is_same_v<T, fixed16>, "set_epsilon_bits only for fixed16; use set_epsilon for float");
		tr::make_epsilon(epsilon_state_, bits);
		epsilon_ = epsilon_state_.epsilon;
		half_epsilon_ = epsilon_state_.half_epsilon;
		quarter_epsilon_ = epsilon_state_.quarter_epsilon;
	}

	T get_epsilon() const { return epsilon_; }

	// Integrator
	void set_integrator(integrator_type i) { integrator_ = i; }
	integrator_type get_integrator() const { return integrator_; }

	void set_snap_to_grid(bool s) { snap_to_grid_ = s; }
	bool get_snap_to_grid() const { return snap_to_grid_; }
	void set_average_normals(bool a) { average_normals_ = a; }
	bool get_average_normals() const { return average_normals_; }

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

	// Solid management
	void add_solid(std::shared_ptr<solid<T>> s) {
		for (auto & existing : solids_) {
			if (existing == s)
				return;
		}
		solids_.push_back(s);
		s->internal_set_simulator(this);
		s->activate();
		spacial_collection_.resize(solids_.size());
	}

	void remove_solid(std::shared_ptr<solid<T>> s) {
		s->touching_ = nullptr;
		s->touched1_ = nullptr;
		s->touched2_ = nullptr;

		for (auto & other : solids_) {
			if (other->touching_ == s.get())
				other->touching_ = nullptr;
			if (other->touched1_ == s.get())
				other->touched1_ = nullptr;
			if (other->touched2_ == s.get())
				other->touched2_ = nullptr;
		}

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

	int get_num_solids() const { return static_cast<int>(solids_.size()); }
	solid<T> * get_solid(int i) const { return solids_[i].get(); }

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
	void update(int dt, int scope = 0, solid<T> * target = nullptr) {
		T fdt = tr::from_milli(dt);
		num_collisions_ = 0;
		++current_tick_;
		if (manager_)
			manager_->pre_update(dt, fdt);

		// Manager may suggest a spatial-locality iteration order. Contract:
		// when non-null, it must contain every solid the simulator should
		// update — anything in solids_ but absent from the order is skipped.
		const std::vector<solid<T> *> * order =
		    (target == nullptr && manager_) ? manager_->get_iteration_order() : nullptr;
		assert(!order || order->size() == solids_.size());

		int num = (target != nullptr) ? 1
		          : (order ? static_cast<int>(order->size()) : static_cast<int>(solids_.size()));
		for (int i = 0; i < num; ++i) {
			solid<T> * s = (target != nullptr) ? target
			               : (order ? (*order)[i] : solids_[i].get());

			if (!s->active_ || (scope != 0 && (s->scope_ & scope) == 0))
				continue;

			s->last_dt_ = dt;
			if (manager_) {
				manager_->pre_update(s, dt, fdt);
			}

			update_solid(s, dt, fdt);

			if (manager_) {
				manager_->post_update(s, dt, fdt);
			}
		}

		if (scope & scope_report_collisions)
			report_collisions();
		if (manager_)
			manager_->post_update(dt, fdt);
	}

	void update_solid(solid<T> * solid_ptr, int dt, T fdt);

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
	void test_segment(collision<T> & result, const segment<T> & seg, solid<T> * s);
	void test_solid(collision<T> & result, solid<T> * s1, const segment<T> & seg, solid<T> * s2);

	// Utility
	void cap_vec3(vec3<T> & v, T value) const {
		v.x = tr::cap(v.x, value);
		v.y = tr::cap(v.y, value);
		v.z = tr::cap(v.z, value);
	}

	void calculate_epsilon_offset(vec3<T> & result, const vec3<T> & direction, const vec3<T> & normal) const {
		if (snap_to_grid_) {
			result.x = (normal.x >= quarter_epsilon_) ? epsilon_ : ((normal.x <= -quarter_epsilon_) ? -epsilon_ : T {});
			result.y = (normal.y >= quarter_epsilon_) ? epsilon_ : ((normal.y <= -quarter_epsilon_) ? -epsilon_ : T {});
			result.z = (normal.z >= quarter_epsilon_) ? epsilon_ : ((normal.z <= -quarter_epsilon_) ? -epsilon_ : T {});
		} else {
			T len = length(direction);
			if (len > epsilon_) {
				result.x = (-direction.x / len) * epsilon_;
				result.y = (-direction.y / len) * epsilon_;
				result.z = (-direction.z / len) * epsilon_;
			} else {
				result.reset();
			}
		}
	}

	void snap_to_grid_vec(vec3<T> & pos) const {
		if (snap_to_grid_) {
			tr::snap_to_grid(pos.x, epsilon_state_);
			tr::snap_to_grid(pos.y, epsilon_state_);
			tr::snap_to_grid(pos.z, epsilon_state_);
		}
	}

	bool too_small(const vec3<T> & v, T epsilon) const {
		return v.x < epsilon && v.x > -epsilon && v.y < epsilon && v.y > -epsilon && v.z < epsilon && v.z > -epsilon;
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
		if constexpr (std::is_same_v<T, fixed16>) {
			set_epsilon_bits(tr::default_epsilon_bits());
		} else {
			set_epsilon(tr::default_epsilon());
		}
		max_position_component_ = tr::default_max_position_component();
		max_velocity_component_ = tr::default_max_velocity_component();
		max_force_component_ = tr::default_max_force_component();
		deactivate_speed_ = tr::default_deactivate_speed(epsilon_state_);
		deactivate_count_ = 4;
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
	void trace_aa_box(collision<T> & c, const segment<T> & seg, const aa_box<T> & box);
	void trace_sphere(collision<T> & c, const segment<T> & seg, const hop::sphere<T> & sph);
	void trace_capsule(collision<T> & c, const segment<T> & seg, const hop::capsule<T> & cap);
	void trace_capsule_capsule(collision<T> & c,
	                           const segment<T> & seg,
	                           const vec3<T> & base,
	                           const vec3<T> & D1,
	                           const vec3<T> & D2,
	                           T radius);
	void trace_convex_solid(collision<T> & c, const segment<T> & seg, const hop::convex_solid<T> & cs);

	// Dispatch helpers for test_solid's convex-vs-* branches. `inflated_cs` is
	// the target-side convex solid with its planes already inflated by the
	// Minkowski-sum amount for the opposing shape (scalar radius, or per-plane
	// `support(shape, p.normal)` for convex-vs-convex). Both helpers normalize
	// col.point to s1's center at impact.
	//
	// Forward: sh1 is primitive, sh2 is convex. sh1_offset is sh1's reference
	// point in s1's local frame (already includes lp1).
	void trace_forward_convex(collision<T> & col,
	                          const segment<T> & seg,
	                          const solid<T> * s2,
	                          const vec3<T> & lp2,
	                          const hop::convex_solid<T> & inflated_cs,
	                          const vec3<T> & sh1_offset);
	// Inverted: sh1 is convex, sh2 is primitive. Traces sh2's reference point
	// backwards against sh1's convex. sh2_offset is sh2's reference point in
	// s2's local frame (does NOT include lp_delta — helper adds it).
	void trace_inverted_convex(collision<T> & col,
	                           const segment<T> & seg,
	                           const solid<T> * s1,
	                           const solid<T> * s2,
	                           const vec3<T> & lp_delta,
	                           const hop::convex_solid<T> & inflated_cs,
	                           const vec3<T> & sh2_offset);
	void friction_link(vec3<T> & result,
	                   solid<T> * s,
	                   const vec3<T> & solid_vel,
	                   solid<T> * hit,
	                   const vec3<T> & hit_normal,
	                   const vec3<T> & applied_force,
	                   T fdt);
	// Apply Coulomb friction impulse at the moment of collision, opposing the
	// tangential relative velocity, capped by µ_d * |normal_impulse|.
	void collision_friction(solid<T> * s, solid<T> * hit, const collision<T> & c, T normal_impulse, T one_over_mass);
	void constraint_link(vec3<T> & result, solid<T> * s, const vec3<T> & solid_pos, const vec3<T> & solid_vel);
	void update_acceleration(vec3<T> & result, solid<T> * s, const vec3<T> & x, const vec3<T> & v, T fdt);
	void integration_step(solid<T> * s,
	                      const vec3<T> & x,
	                      const vec3<T> & v,
	                      const vec3<T> & dx,
	                      const vec3<T> & dv,
	                      T fdt,
	                      vec3<T> & result_x,
	                      vec3<T> & result_v);

	integrator_type integrator_ = integrator_type::heun;
	vec3<T> fluid_velocity_;
	vec3<T> gravity_;
	epsilon_state<T> epsilon_state_;
	T epsilon_ {};
	T half_epsilon_ {};
	T quarter_epsilon_ {};
	bool snap_to_grid_ = false;
	bool average_normals_ = false;
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
	int current_tick_ = 0;  // increments per update(); used to guard cross-update double-impulse

};

// ============================================================================
// Implementation
// ============================================================================

template <typename T> void simulator<T>::update_solid(solid<T> * solid_ptr, int dt, T fdt) {
	vec3<T> old_pos;
	vec3<T> new_pos;
	vec3<T> old_vel;
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
	old_vel.set(solid_ptr->velocity_);

	// Integration
	if (integrator_ == integrator_type::euler) {
		integration_step(solid_ptr, old_pos, solid_ptr->velocity_, zero_vec, zero_vec, fdt, dx1, dv1);
		mul(new_pos, dx1, fdt);
		add(new_pos, old_pos);
		mul(vel, dv1, fdt);
		add(vel, solid_ptr->velocity_);
	} else if (integrator_ == integrator_type::improved) {
		T hfdt = fdt / two;
		integration_step(solid_ptr, old_pos, solid_ptr->velocity_, zero_vec, zero_vec, fdt, dx1, dv1);
		new_pos.set(dx1);
		vel.set(dv1);
		integration_step(solid_ptr, old_pos, solid_ptr->velocity_, dx1, dv1, fdt, dx2, dv2);
		add(new_pos, dx2);
		mul(new_pos, hfdt);
		add(new_pos, old_pos);
		add(vel, dv2);
		mul(vel, hfdt);
		add(vel, solid_ptr->velocity_);
	} else if (integrator_ == integrator_type::heun) {
		T qfdt = fdt / tr::four();
		T ttfdt = fdt * two / three;
		integration_step(solid_ptr, old_pos, solid_ptr->velocity_, zero_vec, zero_vec, fdt, dx1, dv1);
		new_pos.set(dx1);
		vel.set(dv1);
		integration_step(solid_ptr, old_pos, solid_ptr->velocity_, dx1, dv1, ttfdt, dx2, dv2);
		mul(dx2, three);
		add(new_pos, dx2);
		mul(new_pos, qfdt);
		add(new_pos, old_pos);
		mul(dv2, three);
		add(vel, dv2);
		mul(vel, qfdt);
		add(vel, solid_ptr->velocity_);
	} else if (integrator_ == integrator_type::runge_kutta) {
		T hfdt = fdt / two;
		T sfdt = fdt / tr::from_int(6);
		integration_step(solid_ptr, old_pos, solid_ptr->velocity_, zero_vec, zero_vec, fdt, dx1, dv1);
		new_pos.set(dx1);
		vel.set(dv1);
		integration_step(solid_ptr, old_pos, solid_ptr->velocity_, dx1, dv1, hfdt, dx2, dv2);
		mul(temp, dx2, two);
		add(new_pos, temp);
		mul(temp, dv2, two);
		add(vel, temp);
		integration_step(solid_ptr, old_pos, solid_ptr->velocity_, dx2, dv2, hfdt, dx1, dv1);
		mul(temp, dx1, two);
		add(new_pos, temp);
		mul(temp, dv1, two);
		add(vel, temp);
		integration_step(solid_ptr, old_pos, solid_ptr->velocity_, dx1, dv1, fdt, dx2, dv2);
		add(new_pos, dx2);
		mul(new_pos, sfdt);
		add(new_pos, old_pos);
		add(vel, dv2);
		mul(vel, sfdt);
		add(vel, solid_ptr->velocity_);
	}

	cap_vec3(vel, max_velocity_component_);
	solid_ptr->velocity_.set(vel);
	solid_ptr->clear_force();

	bool first = true;
	bool skip = false;

	if (manager_) {
		manager_->intra_update(solid_ptr, dt, fdt);
	}

	snap_to_grid_vec(old_pos);
	cap_vec3(old_pos, max_position_component_);
	snap_to_grid_vec(new_pos);
	cap_vec3(new_pos, max_position_component_);

	// Collect spacials
	if (solid_ptr->collide_with_scope_ != 0) {
		sub(temp, new_pos, old_pos);
		{
			if (temp.x < T {})
				temp.x = -temp.x;
			if (temp.y < T {})
				temp.y = -temp.y;
			if (temp.z < T {})
				temp.z = -temp.z;

			T m = temp.x;
			if (temp.y > m)
				m = temp.y;
			if (temp.z > m)
				m = temp.z;
			m = m + epsilon_;

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
	}

	// Collision loop
	loop = 0;
	while (!skip) {
		if (!first) {
			snap_to_grid_vec(old_pos);
			snap_to_grid_vec(new_pos);
			sub(temp, new_pos, old_pos);
			if (too_small(temp, epsilon_)) {
				new_pos.set(old_pos);
				break;
			}
		}

		path.set_start_end(old_pos, new_pos);
		trace_solid_with_current_spacials(c, solid_ptr, path, solid_ptr->collide_with_scope_);

		if (c.time < one) {
			snap_to_grid_vec(c.point);
			sub(left_over, c.point, old_pos);
			calculate_epsilon_offset(old_pos, left_over, c.normal);
			add(old_pos, c.point);
			sub(left_over, new_pos, old_pos);

			// Store collision for reporting
			if (c.collider != solid_ptr->touching_ &&
			    (solid_ptr->collision_callback_ != nullptr ||
			     (c.collider && c.collider->collision_callback_ != nullptr))) {
				c.collidee = solid_ptr;
				if (c.collider) {
					sub(c.velocity, solid_ptr->velocity_, c.collider->velocity_);
				} else {
					c.velocity.set(solid_ptr->velocity_);
				}
				if (num_collisions_ < static_cast<int>(collisions_.size())) {
					collisions_[num_collisions_].set(c);
					num_collisions_++;
				}
			}
			hit_solid = c.collider;

			// Cross-update guard. Check per-pair: have solid_ptr and hit_solid
			// already exchanged a collision impulse this tick (in some other
			// solid's earlier update)? If so, re-impulsing them here would
			// double-count and inject energy. But we can't simply skip — the
			// integrated new_pos may have solid_ptr overlapping hit_solid,
			// and unchecked overlap accumulates over ticks until bodies
			// phase through each other (e.g. a 20-ball stack collapsing to a
			// blob). Apply a position-only contact constraint instead:
			// remove solid_ptr's velocity component along the contact normal
			// (no penetration through the partner) and snap position to the
			// contact surface. Energy strictly decreases (we only damp, never
			// boost). The earlier impulse from the partner's update gave both
			// sides their proper post-collision velocities; this constraint
			// just enforces the geometric "you can't pass through" invariant.
			if (hit_solid && solid_ptr->impulse_partner_tick_ == current_tick_) {
				bool already_impulsed = false;
				for (int k = 0; k < solid_ptr->impulse_partner_count_; ++k) {
					if (solid_ptr->impulse_partners_[k] == hit_solid) {
						already_impulsed = true;
						break;
					}
				}
				if (already_impulsed) {
					T v_along_n = dot(solid_ptr->velocity_, c.normal);
					if (v_along_n < T{}) {
						vec3<T> remove;
						mul(remove, c.normal, v_along_n);
						sub(solid_ptr->velocity_, remove);
					}
					new_pos.set(old_pos);  // stop at contact surface
					break;
				}
			}

			bool responded = false;
			if (manager_) {
				responded = manager_->collision_response(solid_ptr, old_pos, left_over, c);
			}

			if (!responded) {
				// Conservation of momentum
				if (solid_ptr->coefficient_of_restitution_override_ || !hit_solid) {
					cor = solid_ptr->coefficient_of_restitution_;
				} else {
					cor = (solid_ptr->coefficient_of_restitution_ + hit_solid->coefficient_of_restitution_) / two;
				}

				if (hit_solid) {
					sub(temp, hit_solid->velocity_, solid_ptr->velocity_);
				} else {
					neg(temp, solid_ptr->velocity_);
				}

				if (dot(temp, c.normal) < micro_collision_threshold_) {
					cor = T {};
				}

				T numerator = (one + cor) * dot(temp, c.normal);
				temp.reset();

				T zero_val {};
				if (solid_ptr->mass_ != zero_val && (!hit_solid || hit_solid->mass_ != zero_val)) {
					one_over_mass = solid_ptr->inv_mass_;
					one_over_hit_mass = hit_solid ? hit_solid->inv_mass_ : zero_val;

					if ((one_over_mass + one_over_hit_mass) != zero_val) {
						impulse = numerator / (one_over_mass + one_over_hit_mass);
					} else {
						impulse = zero_val;
					}

					if (solid_ptr->mass_ != solid<T>::infinite_mass()) {
						mul(t, c.normal, impulse);
						mul(t, one_over_mass);
						add(solid_ptr->velocity_, t);
						collision_friction(solid_ptr, hit_solid, c, impulse, one_over_mass);
					}
					if (hit_solid && hit_solid->mass_ != solid<T>::infinite_mass()) {
						mul(temp, c.normal, impulse);
						mul(temp, one_over_hit_mass);
					}
				} else if (hit_solid) {
					mul(temp, c.normal, numerator);
				} else if (solid_ptr->mass_ == zero_val) {
					mul(t, c.normal, numerator);
					add(solid_ptr->velocity_, t);
				}

				if (hit_solid && (hit_solid->collide_with_scope_ & solid_ptr->collision_scope_) != 0 &&
				    hit_solid->should_collide(solid_ptr) &&
				    (tr::abs(temp.x) >= deactivate_speed_ || tr::abs(temp.y) >= deactivate_speed_ ||
				     tr::abs(temp.z) >= deactivate_speed_)) {
					hit_solid->activate();
					sub(hit_solid->velocity_, temp);
				}

				// Record the impulsed pair so subsequent solid updates that
				// re-detect this pair recognize it and apply only a contact
				// constraint (not a second impulse). Recorded on both sides
				// since either side's later update may detect the pair.
				if (hit_solid) {
					auto record = [&](solid<T> * a, solid<T> * b) {
						if (a->impulse_partner_tick_ != current_tick_) {
							a->impulse_partner_count_ = 0;
							a->impulse_partner_tick_ = current_tick_;
						}
						if (a->impulse_partner_count_ < solid<T>::max_impulse_partners_per_tick)
							a->impulse_partners_[a->impulse_partner_count_++] = b;
					};
					record(solid_ptr, hit_solid);
					record(hit_solid, solid_ptr);
				}
			}

			// Touching code
			solid_ptr->touched2_ = solid_ptr->touched1_;
			solid_ptr->touched2_normal_.set(solid_ptr->touched1_normal_);
			if (solid_ptr->touched1_ == c.collider) {
				solid_ptr->touching_ = c.collider;
				solid_ptr->touching_normal_.set(c.normal);
			} else {
				solid_ptr->touched1_ = c.collider;
				solid_ptr->touched1_normal_.set(c.normal);
				solid_ptr->touching_ = nullptr;
			}

			if (too_small(left_over, epsilon_)) {
				new_pos.set(old_pos);
				break;
			} else if (loop > 4) {
				// Push the body out along the last contact normal rather
				// than zeroing velocity. Zeroing causes permanent overlap
				// when two dynamic bodies collide at the loop limit.
				mul(temp, c.normal, epsilon_ * tr::from_int(4));
				add(new_pos, old_pos, temp);
				break;
			} else {
				if (!normalize_carefully(vel, solid_ptr->velocity_, epsilon_)) {
					new_pos.set(old_pos);
					break;
				} else {
					mul(vel, length(left_over));
					mul(temp, c.normal, dot(vel, c.normal));
					sub(vel, temp);
					add(new_pos, old_pos, vel);
				}
				first = false;
			}
		} else {
			break;
		}
		loop++;
	}

	// Reset touching if no collision
	if (!skip && c.time == one && loop == 0) {
		solid_ptr->touching_ = nullptr;
		solid_ptr->touched1_ = nullptr;
		solid_ptr->touched2_ = nullptr;
	}

	// Deactivation
	if (solid_ptr->deactivate_count_ >= 0) {
		if (tr::abs(new_pos.x - solid_ptr->position_.x) < deactivate_speed_ &&
		    tr::abs(new_pos.y - solid_ptr->position_.y) < deactivate_speed_ &&
		    tr::abs(new_pos.z - solid_ptr->position_.z) < deactivate_speed_) {
			solid_ptr->deactivate_count_++;
			if (solid_ptr->deactivate_count_ > deactivate_count_) {
				int j;
				for (j = static_cast<int>(solid_ptr->constraints_.size()) - 1; j >= 0; --j) {
					auto * con = solid_ptr->constraints_[j];
					auto * start = con->start_solid_.get();
					auto * end = con->end_solid_.get();
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
	}

	solid_ptr->set_position_direct(new_pos);
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

template <typename T> void simulator<T>::test_segment(collision<T> & result, const segment<T> & seg, solid<T> * s) {
	collision<T> col;
	col.collider = s;
	const T one = tr::one();

	auto & shapes = s->shapes_;
	int n = static_cast<int>(shapes.size());
	bool modify_scope = false;

	for (int i = 0; i < n; ++i) {
		auto * sh = shapes[i].get();
		const vec3<T> & lp = sh->get_local_position();
		switch (sh->type_) {
		case shape_type::box: {
			aa_box<T> box;
			box.set(sh->box_);
			add(box, s->position_);
			add(box, lp);
			trace_aa_box(col, seg, box);
			break;
		}
		case shape_type::sphere: {
			hop::sphere<T> sph;
			sph.set(sh->sphere_);
			add(sph, s->position_);
			add(sph, lp);
			trace_sphere(col, seg, sph);
			break;
		}
		case shape_type::capsule: {
			hop::capsule<T> cap;
			cap.set(sh->capsule_);
			add(cap, s->position_);
			add(cap, lp);
			trace_capsule(col, seg, cap);
			break;
		}
		case shape_type::convex_solid: {
			hop::convex_solid<T> cs;
			cs.set(*sh->convex_solid_);
			segment<T> tmp;
			tmp.set(seg);
			sub(tmp.origin, s->position_);
			sub(tmp.origin, lp);
			trace_convex_solid(col, tmp, cs);
			if (col.time < one) {
				add(col.point, s->position_);
				add(col.point, lp);
			}
			break;
		}
		case shape_type::traceable: {
			vec3<T> traceable_origin;
			add(traceable_origin, s->position_, lp);
			sh->traceable_->trace_segment(col, traceable_origin, seg);
			modify_scope = true;
			break;
		}
		}

		// Segment traces have no Minkowski expansion: impact == point
		if (col.time < one)
			col.impact.set(col.point);

		// Trigger zones: OR collidee's trigger bits into the result on static
		// overlap (t == 0). Lets callers tag a solid as a damage zone / volume
		// and read which zones a trace ended up inside via result.trigger_scope.
		if (col.time == T {})
			col.trigger_scope |= s->trigger_scope_;

		int trigger_scope = result.trigger_scope;
		if (col.time < one) {
			if (col.time < result.time) {
				result.set(col);
			} else if (result.time == col.time) {
				add(result.normal, col.normal);
				if (!normalize_carefully(result.normal, epsilon_))
					result.set(col);
			}
			modify_scope |= (col.time == T {});
		}

		result.trigger_scope = modify_scope ? (trigger_scope | col.trigger_scope) : trigger_scope;
	}
}

template <typename T>
void simulator<T>::test_solid(collision<T> & result, solid<T> * s1, const segment<T> & seg, solid<T> * s2) {
	collision<T> col;
	col.collider = s2;
	const T one = tr::one();
	T zero_val {};

	// Squared segment length is invariant across all shape pairs in this call;
	// hoist it for the sphere/sphere fast reject below.
	T dir_sq = length_squared(seg.direction);

	auto & shapes1 = s1->shapes_;
	auto & shapes2 = s2->shapes_;
	int n1 = static_cast<int>(shapes1.size());
	int n2 = static_cast<int>(shapes2.size());
	bool modify_scope = false;

	for (int i = 0; i < n1; ++i) {
		auto * sh1 = shapes1[i].get();
		for (int j = 0; j < n2; ++j) {
			auto * sh2 = shapes2[j].get();
			modify_scope = false;

			const vec3<T> & lp1 = sh1->get_local_position();
			const vec3<T> & lp2 = sh2->get_local_position();
			vec3<T> lp_delta;
			sub(lp_delta, lp2, lp1);

			// Traceable paths route through the traceable callback regardless of
			// the other side's primitive type, so dispatch on traceable-ness first.
			if (sh1->type_ == shape_type::traceable) {
				segment<T> iseg;
				add(iseg.origin, s2->position_, lp2);
				mul(iseg.direction, seg.direction, -tr::one());
				vec3<T> tr_origin;
				add(tr_origin, seg.origin, lp1);
				sh1->traceable_->trace_solid(col, s2, tr_origin, iseg);
				col.invert();
				sub(iseg.origin, col.point);
				add(col.point, seg.origin, iseg.origin);
				modify_scope = true;
			} else if (sh2->type_ == shape_type::traceable) {
				vec3<T> tr_origin;
				add(tr_origin, s2->position_, lp2);
				sh2->traceable_->trace_solid(col, s1, tr_origin, seg);
				modify_scope = true;
			}
			// AABox vs *
			else if (sh1->type_ == shape_type::box && sh2->type_ == shape_type::box) {
				aa_box<T> box;
				box.set(sh2->box_);
				add(box, s2->position_);
				add(box, lp_delta);
				sub(box.maxs, sh1->box_.mins);
				sub(box.mins, sh1->box_.maxs);
				trace_aa_box(col, seg, box);
			} else if (sh1->type_ == shape_type::box && sh2->type_ == shape_type::sphere) {
				aa_box<T> box;
				box.set(sh2->sphere_.radius);
				add(box, sh2->sphere_.origin);
				add(box, s2->position_);
				add(box, lp_delta);
				sub(box.maxs, sh1->box_.mins);
				sub(box.mins, sh1->box_.maxs);
				trace_aa_box(col, seg, box);
			} else if (sh1->type_ == shape_type::box && sh2->type_ == shape_type::capsule) {
				aa_box<T> box;
				sh2->get_bound(box);
				add(box, s2->position_);
				add(box, lp_delta);
				sub(box.maxs, sh1->box_.mins);
				sub(box.mins, sh1->box_.maxs);
				trace_aa_box(col, seg, box);
			} else if (sh1->type_ == shape_type::box && sh2->type_ == shape_type::convex_solid) {
				// Conservative: inflate sh2's planes by max half-extent of sh1's aa_box.
				hop::convex_solid<T> cs;
				cs.set(*sh2->convex_solid_);
				vec3<T> half;
				sub(half, sh1->box_.maxs, sh1->box_.mins);
				mul(half, tr::half());
				T max_half = half.x;
				if (half.y > max_half)
					max_half = half.y;
				if (half.z > max_half)
					max_half = half.z;
				for (auto & p : cs.planes)
					p.distance = p.distance + max_half;
				// sh1 reference = box center + lp1.
				vec3<T> sh1_offset;
				add(sh1_offset, sh1->box_.mins, sh1->box_.maxs);
				mul(sh1_offset, tr::half());
				add(sh1_offset, lp1);
				trace_forward_convex(col, seg, s2, lp2, cs, sh1_offset);
			}
			// Sphere vs *
			else if (sh1->type_ == shape_type::sphere && sh2->type_ == shape_type::box) {
				aa_box<T> box1;
				box1.set(sh1->sphere_.radius);
				add(box1, sh1->sphere_.origin);
				aa_box<T> box;
				box.set(sh2->box_);
				add(box, s2->position_);
				add(box, lp_delta);
				sub(box.maxs, box1.mins);
				sub(box.mins, box1.maxs);
				trace_aa_box(col, seg, box);
			} else if (sh1->type_ == shape_type::sphere && sh2->type_ == shape_type::sphere) {
				vec3<T> origin;
				origin.set(s2->position_);
				add(origin, lp_delta);
				sub(origin, sh1->sphere_.origin);
				add(origin, sh2->sphere_.origin);
				T r_sum = sh1->sphere_.radius + sh2->sphere_.radius;
				// Fast reject: if start-position centers are too far apart for the
				// swept sphere to ever reach the target sphere this tick, skip the
				// quadratic root solve. Conservative no-sqrt bound:
				//   2·(r_sum² + |dir|²) ≥ (r_sum + |dir|)²   (AM-GM)
				// — looser than the sqrt-form by a factor of ≤2 but free.
				vec3<T> diff;
				sub(diff, seg.origin, origin);
				T limit = (r_sum * r_sum + dir_sq) * tr::two();
				if (length_squared(diff) > limit) {
					col.time = one;  // explicit miss; merge below leaves result untouched
				} else {
					hop::sphere<T> sph;
					sph.set(origin, r_sum);
					trace_sphere(col, seg, sph);
				}
			} else if (sh1->type_ == shape_type::sphere && sh2->type_ == shape_type::capsule) {
				vec3<T> origin;
				origin.set(s2->position_);
				add(origin, lp_delta);
				sub(origin, sh1->sphere_.origin);
				add(origin, sh2->capsule_.origin);
				hop::capsule<T> cap;
				cap.set(origin, sh2->capsule_.direction, sh2->capsule_.radius + sh1->sphere_.radius);
				trace_capsule(col, seg, cap);
			} else if (sh1->type_ == shape_type::sphere && sh2->type_ == shape_type::convex_solid) {
				hop::convex_solid<T> cs;
				cs.set(*sh2->convex_solid_);
				for (auto & p : cs.planes)
					p.distance = p.distance + sh1->sphere_.radius;
				vec3<T> sh1_offset;
				add(sh1_offset, sh1->sphere_.origin, lp1);
				trace_forward_convex(col, seg, s2, lp2, cs, sh1_offset);
			}
			// Capsule vs *
			else if (sh1->type_ == shape_type::capsule && sh2->type_ == shape_type::box) {
				aa_box<T> box1;
				sh1->get_bound(box1);
				aa_box<T> box;
				box.set(sh2->box_);
				add(box, s2->position_);
				add(box, lp_delta);
				sub(box.maxs, box1.mins);
				sub(box.mins, box1.maxs);
				trace_aa_box(col, seg, box);
			} else if (sh1->type_ == shape_type::capsule && sh2->type_ == shape_type::sphere) {
				vec3<T> origin;
				origin.set(s2->position_);
				add(origin, lp_delta);
				sub(origin, sh1->capsule_.origin);
				add(origin, sh2->sphere_.origin);
				vec3<T> dir;
				dir.set(sh1->capsule_.direction);
				neg(dir);
				hop::capsule<T> cap;
				cap.set(origin, dir, sh1->capsule_.radius + sh2->sphere_.radius);
				trace_capsule(col, seg, cap);
			} else if (sh1->type_ == shape_type::capsule && sh2->type_ == shape_type::convex_solid) {
				// Inflate sh2's planes by sh1's capsule radius, then trace sh1's
				// two spine endpoints as separate segments. Conservative
				// Minkowski-sum approximation.
				hop::convex_solid<T> cs;
				cs.set(*sh2->convex_solid_);
				for (auto & p : cs.planes)
					p.distance = p.distance + sh1->capsule_.radius;
				vec3<T> bottom_offset;
				add(bottom_offset, sh1->capsule_.origin, lp1);
				vec3<T> top_offset;
				add(top_offset, bottom_offset, sh1->capsule_.direction);
				collision<T> col_bottom;
				col_bottom.time = one;
				trace_forward_convex(col_bottom, seg, s2, lp2, cs, bottom_offset);
				collision<T> col_top;
				col_top.time = one;
				trace_forward_convex(col_top, seg, s2, lp2, cs, top_offset);
				// Merge the earlier hit's fields without touching col.collider /
				// col.trigger_scope (see test_solid preamble for why).
				const collision<T> & hit = (col_bottom.time < col_top.time) ? col_bottom : col_top;
				col.time = hit.time;
				col.normal.set(hit.normal);
				if (col.time < one)
					col.point.set(hit.point);
			} else if (sh1->type_ == shape_type::capsule && sh2->type_ == shape_type::capsule) {
				vec3<T> base;
				base.set(s2->position_);
				add(base, lp_delta);
				sub(base, sh1->capsule_.origin);
				add(base, sh2->capsule_.origin);
				trace_capsule_capsule(col,
				                      seg,
				                      base,
				                      sh1->capsule_.direction,
				                      sh2->capsule_.direction,
				                      sh1->capsule_.radius + sh2->capsule_.radius);
			}
			// Convex solid vs aa_box
			else if (sh1->type_ == shape_type::convex_solid && sh2->type_ == shape_type::box) {
				hop::convex_solid<T> cs;
				cs.set(*sh1->convex_solid_);
				vec3<T> half;
				sub(half, sh2->box_.maxs, sh2->box_.mins);
				mul(half, tr::half());
				T max_half = half.x;
				if (half.y > max_half)
					max_half = half.y;
				if (half.z > max_half)
					max_half = half.z;
				for (auto & p : cs.planes)
					p.distance = p.distance + max_half;
				vec3<T> sh2_offset;
				add(sh2_offset, sh2->box_.mins, sh2->box_.maxs);
				mul(sh2_offset, tr::half());
				trace_inverted_convex(col, seg, s1, s2, lp_delta, cs, sh2_offset);
			}
			// Convex solid vs sphere
			else if (sh1->type_ == shape_type::convex_solid && sh2->type_ == shape_type::sphere) {
				hop::convex_solid<T> cs;
				cs.set(*sh1->convex_solid_);
				for (auto & p : cs.planes)
					p.distance = p.distance + sh2->sphere_.radius;
				trace_inverted_convex(col, seg, s1, s2, lp_delta, cs, sh2->sphere_.origin);
			}
			// Convex solid vs capsule
			else if (sh1->type_ == shape_type::convex_solid && sh2->type_ == shape_type::capsule) {
				hop::convex_solid<T> cs;
				cs.set(*sh1->convex_solid_);
				for (auto & p : cs.planes)
					p.distance = p.distance + sh2->capsule_.radius;
				collision<T> col_bottom;
				col_bottom.time = one;
				trace_inverted_convex(col_bottom, seg, s1, s2, lp_delta, cs, sh2->capsule_.origin);
				vec3<T> top_offset;
				add(top_offset, sh2->capsule_.origin, sh2->capsule_.direction);
				collision<T> col_top;
				col_top.time = one;
				trace_inverted_convex(col_top, seg, s1, s2, lp_delta, cs, top_offset);
				const collision<T> & hit = (col_bottom.time < col_top.time) ? col_bottom : col_top;
				if (hit.time < one) {
					col.time = hit.time;
					col.normal.set(hit.normal);
					col.point.set(hit.point);
				}
			}
			// Convex solid vs convex solid (exact Minkowski sum: inflate sh2's planes per sh1's support)
			else if (sh1->type_ == shape_type::convex_solid && sh2->type_ == shape_type::convex_solid) {
				hop::convex_solid<T> cs;
				cs.set(*sh2->convex_solid_);
				for (auto & p : cs.planes) {
					vec3<T> sup;
					support(sup, *sh1->convex_solid_, p.normal);
					p.distance = p.distance + dot(sup, p.normal);
				}
				// sh1 reference point in s1's local frame = lp1 (sh1's convex origin sits there).
				trace_forward_convex(col, seg, s2, lp2, cs, lp1);
			}

			// Compute impact point for solid traces.
			// col.point represents s1's center at impact time; sh1's world contact
			// surface is offset from that by sh1's local_position + support(shape, -normal).
			if (col.time < one && sh1->type_ != shape_type::traceable && sh2->type_ != shape_type::traceable) {
				vec3<T> sup;
				vec3<T> neg_n;
				neg(neg_n, col.normal);
				support(sup, *sh1, neg_n);
				add(col.impact, col.point, sup);
				add(col.impact, lp1);
			} else if (col.time < one) {
				col.impact.set(col.point);
			}

			// Per-pair reset of col.trigger_scope so stale bits from a previous
			// shape pair don't leak in. Then OR collidee's trigger bits on
			// static overlap. Works for primitive AND traceable pairs so
			// trimesh trigger volumes report the same way as primitive ones.
			col.trigger_scope = 0;
			if (col.time == zero_val)
				col.trigger_scope = s2->trigger_scope_;

			int trigger_scope = result.trigger_scope;
			if (col.time < one) {
				if (col.time < result.time) {
					result.set(col);
				} else if (result.time == col.time) {
					add(result.normal, col.normal);
					if (!normalize_carefully(result.normal, epsilon_))
						result.set(col);
				}
				modify_scope |= (col.time == zero_val);
			}
			result.trigger_scope = modify_scope ? (trigger_scope | col.trigger_scope) : trigger_scope;
		}
	}
}

template <typename T>
void simulator<T>::trace_segment_with_current_spacials(collision<T> & result,
                                                       const segment<T> & seg,
                                                       int collide_with_bits,
                                                       solid<T> * ignore) {
	result.time = tr::one();
	result.trigger_scope = 0;

	collision<T> col;
	for (int i = 0; i < num_spacial_collection_; ++i) {
		auto * s2 = spacial_collection_[i];
		if (s2 != ignore && (collide_with_bits & s2->collision_scope_) != 0) {
			col.time = tr::one();
			test_segment(col, seg, s2);
			int trigger_scope = result.trigger_scope;
			if (col.time < tr::one()) {
				if (col.time < result.time)
					result.set(col);
				else if (average_normals_ && result.time == col.time) {
					add(result.normal, col.normal);
					if (!normalize_carefully(result.normal, epsilon_))
						result.set(col);
				}
			}
			result.trigger_scope = trigger_scope | col.trigger_scope;
		}
	}

	if (manager_) {
		col.time = tr::one();
		manager_->trace_segment(col, seg, collide_with_bits);
		int trigger_scope = result.trigger_scope;
		if (col.time < tr::one()) {
			if (col.time < result.time)
				result.set(col);
			else if (average_normals_ && result.time == col.time) {
				add(result.normal, col.normal);
				if (!normalize_carefully(result.normal, epsilon_))
					result.set(col);
			}
		}
		result.trigger_scope = trigger_scope | col.trigger_scope;
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
	result.time = tr::one();
	if (collide_with_bits == 0)
		return;

	collision<T> col;
	for (int i = 0; i < num_spacial_collection_; ++i) {
		auto * s2 = spacial_collection_[i];
		if (s != s2 && (collide_with_bits & s2->collision_scope_) != 0 && s->should_collide(s2) &&
		    s2->should_collide(s)) {
			col.time = tr::one();
			test_solid(col, s, seg, s2);
			int trigger_scope = result.trigger_scope;
			if (col.time < tr::one()) {
				if (col.time < result.time)
					result.set(col);
				else if (average_normals_ && result.time == col.time) {
					add(result.normal, col.normal);
					if (!normalize_carefully(result.normal, epsilon_))
						result.set(col);
				}
			}
			result.trigger_scope = trigger_scope | col.trigger_scope;
		}
	}

	if (manager_) {
		col.time = tr::one();
		manager_->trace_solid(col, s, seg, collide_with_bits);
		int trigger_scope = result.trigger_scope;
		if (col.time < tr::one()) {
			if (col.time < result.time)
				result.set(col);
			else if (average_normals_ && result.time == col.time) {
				add(result.normal, col.normal);
				if (!normalize_carefully(result.normal, epsilon_))
					result.set(col);
			}
		}
		result.trigger_scope = trigger_scope | col.trigger_scope;
	}

	if (result.time == tr::one()) {
		seg.get_end_point(result.point);
		result.impact.set(result.point);
	}
}

template <typename T> void simulator<T>::trace_aa_box(collision<T> & c, const segment<T> & seg, const aa_box<T> & box) {
	const T one = tr::one();
	if (test_inside(box, seg.origin)) {
		if (length_squared(seg.direction) > T {}) {
			T x;
			T dix, diy, diz, dax, day, daz;
			x = seg.origin.x - box.mins.x;
			dix = tr::abs(x);
			x = seg.origin.y - box.mins.y;
			diy = tr::abs(x);
			x = seg.origin.z - box.mins.z;
			diz = tr::abs(x);
			x = seg.origin.x - box.maxs.x;
			dax = tr::abs(x);
			x = seg.origin.y - box.maxs.y;
			day = tr::abs(x);
			x = seg.origin.z - box.maxs.z;
			daz = tr::abs(x);

			auto neg_x = constants<T>::neg_x_unit_vec3();
			auto neg_y = constants<T>::neg_y_unit_vec3();
			auto neg_z = constants<T>::neg_z_unit_vec3();
			auto pos_x = constants<T>::x_unit_vec3();
			auto pos_y = constants<T>::y_unit_vec3();
			auto pos_z = constants<T>::z_unit_vec3();

			if (dix <= diy && dix <= diz && dix <= dax && dix <= day && dix <= daz) {
				if (dot(seg.direction, neg_x) >= T {})
					return;
				c.normal.set(neg_x);
			} else if (diy <= diz && diy <= dax && diy <= day && diy <= daz) {
				if (dot(seg.direction, neg_y) >= T {})
					return;
				c.normal.set(neg_y);
			} else if (diz <= dax && diz <= day && diz <= daz) {
				if (dot(seg.direction, neg_z) >= T {})
					return;
				c.normal.set(neg_z);
			} else if (dax <= day && dax <= daz) {
				if (dot(seg.direction, pos_x) >= T {})
					return;
				c.normal.set(pos_x);
			} else if (day <= daz) {
				if (dot(seg.direction, pos_y) >= T {})
					return;
				c.normal.set(pos_y);
			} else {
				if (dot(seg.direction, pos_z) >= T {})
					return;
				c.normal.set(pos_z);
			}
		}
		c.time = T {};
		c.point.set(seg.origin);
	} else {
		c.time = find_intersection(seg, box, c.point, c.normal);
	}
}

template <typename T>
void simulator<T>::trace_sphere(collision<T> & c, const segment<T> & seg, const hop::sphere<T> & sph) {
	const T one = tr::one();
	if (test_inside(sph, seg.origin)) {
		vec3<T> n;
		n.set(seg.origin);
		sub(n, sph.origin);
		if (!normalize_carefully(n, epsilon_)) {
			// Origin exactly at sphere center — use negative direction as normal
			normalize(n, seg.direction);
			neg(n);
		}
		if (dot(n, seg.direction) <= epsilon_) {
			c.time = T {};
			c.point.set(seg.origin);
			c.normal.set(n);
		} else {
			c.time = one;
		}
	} else {
		c.time = find_intersection(seg, sph, c.point, c.normal);
	}
}

template <typename T>
void simulator<T>::trace_capsule(collision<T> & c, const segment<T> & seg, const hop::capsule<T> & cap) {
	vec3<T> p1, p2;
	segment<T> s;
	s.origin.set(cap.origin);
	s.direction.set(cap.direction);
	project(p1, p2, s, seg, epsilon_);
	hop::sphere<T> sph;
	sph.set(p1, cap.radius);
	trace_sphere(c, seg, sph);
}

template <typename T>
void simulator<T>::trace_capsule_capsule(
    collision<T> & c, const segment<T> & seg, const vec3<T> & base, const vec3<T> & D1, const vec3<T> & D2, T radius) {
	const T one = tr::one();
	T zero_val {};

	// The Minkowski sum of two capsule spines forms a parallelogram with vertices:
	//   V0 = base,  V1 = base + D2,  V2 = base + D2 - D1,  V3 = base - D1
	// inflated by the combined radius.
	//
	// We trace the 4 edges as capsules, plus an analytical interior-interior solve
	// for when both closest-point parameters are in (0,1).

	c.time = one;

	// Edge 1: V0->V1 = capsule(base, D2, R) — sh1-start vs sh2-spine
	hop::capsule<T> edge_cap;
	edge_cap.set(base, D2, radius);
	collision<T> edge_col;
	trace_capsule(edge_col, seg, edge_cap);
	if (edge_col.time < c.time) {
		c.time = edge_col.time;
		c.point.set(edge_col.point);
		c.normal.set(edge_col.normal);
	}

	// Edge 2: V3->V2 = capsule(base - D1, D2, R) — sh1-end vs sh2-spine
	vec3<T> v3;
	sub(v3, base, D1);
	edge_cap.set(v3, D2, radius);
	edge_col.reset();
	trace_capsule(edge_col, seg, edge_cap);
	if (edge_col.time < c.time) {
		c.time = edge_col.time;
		c.point.set(edge_col.point);
		c.normal.set(edge_col.normal);
	}

	// Edge 3: V0->V3 = capsule(base, -D1, R) — sh2-start vs sh1-spine
	vec3<T> neg_D1;
	neg(neg_D1, D1);
	edge_cap.set(base, neg_D1, radius);
	edge_col.reset();
	trace_capsule(edge_col, seg, edge_cap);
	if (edge_col.time < c.time) {
		c.time = edge_col.time;
		c.point.set(edge_col.point);
		c.normal.set(edge_col.normal);
	}

	// Edge 4: V1->V2 = capsule(base + D2, -D1, R) — sh2-end vs sh1-spine
	vec3<T> v1;
	add(v1, base, D2);
	edge_cap.set(v1, neg_D1, radius);
	edge_col.reset();
	trace_capsule(edge_col, seg, edge_cap);
	if (edge_col.time < c.time) {
		c.time = edge_col.time;
		c.point.set(edge_col.point);
		c.normal.set(edge_col.normal);
	}

	// Interior-interior analytical solve:
	// When both closest-point parameters s,u are in (0,1), the closest-point
	// vector between the two spines is:
	//   diff(t) = G(t) + s(t)*D1 - u(t)*D2
	// where G(t) = seg.origin + t*V - base (moving point minus base), and
	// s(t), u(t) are linear in t from the 2x2 Cramer system.
	// We solve |diff(t)|² = R² as a quadratic in t.

	T a11 = dot(D1, D1); // |D1|²
	T a12 = dot(D1, D2); // D1·D2
	T a22 = dot(D2, D2); // |D2|²
	T det = a11 * a22 - a12 * a12;

	// Skip if spines are (nearly) parallel — edges already cover this case
	if (tr::abs(det) > epsilon_) {
		T inv_det = one / det;
		vec3<T> V;
		V.set(seg.direction); // motion direction

		// G0 = seg.origin - base (initial vector from base to moving point)
		vec3<T> G0;
		sub(G0, seg.origin, base);

		// The closest-point system gives:
		//   s*a11 - u*a12 = -(D1·G)   =>  g1 - t*v1
		//   s*a12 - u*a22 = -(D2·G)   =>  g2 - t*v2
		// where g1 = -D1·G0 = D1·(base-seg.origin), v1 = D1·V, etc.
		T v1 = dot(D1, V);
		T v2 = dot(D2, V);
		T g1 = -dot(D1, G0);
		T g2 = -dot(D2, G0);

		// Cramer's rule for s0, u0 (at t=0):
		T s0 = (a22 * g1 - a12 * g2) * inv_det;
		T u0 = (a12 * g1 - a11 * g2) * inv_det;
		// Rates ds/dt, du/dt:
		T ds = (-a22 * v1 + a12 * v2) * inv_det;
		T du = (-a12 * v1 + a11 * v2) * inv_det;

		// P0 = diff(0) = G0 + s0*D1 - u0*D2
		vec3<T> P0;
		P0.set(G0);
		vec3<T> tmp;
		mul(tmp, D1, s0);
		add(P0, tmp);
		mul(tmp, D2, u0);
		sub(P0, tmp);

		// dP = d(diff)/dt = V + ds*D1 - du*D2
		vec3<T> dP;
		dP.set(V);
		mul(tmp, D1, ds);
		add(dP, tmp);
		mul(tmp, D2, du);
		sub(dP, tmp);

		// Quadratic: |P0 + t*dP|² = R²
		// A*t² + B*t + C = 0
		T A = dot(dP, dP);
		T B = tr::from_int(2) * dot(P0, dP);
		T C = dot(P0, P0) - radius * radius;

		T disc = B * B - tr::from_int(4) * A * C;
		if (disc >= zero_val && A > epsilon_) {
			T sqrt_disc = tr::sqrt(disc);
			T t_hit = (-B - sqrt_disc) / (tr::from_int(2) * A);

			if (t_hit >= zero_val && t_hit < c.time) {
				// Validate s(t) and u(t) are in [0,1]
				T s_t = s0 + ds * t_hit;
				T u_t = u0 + du * t_hit;

				if (s_t > zero_val && s_t < one && u_t > zero_val && u_t < one) {
					// Normal = normalized diff(t_hit)
					vec3<T> diff;
					mul(diff, dP, t_hit);
					add(diff, P0);

					vec3<T> norm;
					norm.set(diff);
					if (normalize_carefully(norm, epsilon_)) {
						c.time = t_hit;
						mul(c.point, seg.direction, t_hit);
						add(c.point, seg.origin);
						c.normal.set(norm);
					}
				}
			}
		}
	}
}

template <typename T>
void simulator<T>::trace_convex_solid(collision<T> & c, const segment<T> & seg, const hop::convex_solid<T> & cs) {
	const T one = tr::one();
	T zero_val {};
	c.time = one;

	// Check if segment origin is inside the convex solid (all planes)
	bool inside = true;
	T closest_dist = -tr::default_max_position_component();
	int closest_plane = -1;
	for (int i = 0; i < static_cast<int>(cs.planes.size()); ++i) {
		T d = dot(cs.planes[i].normal, seg.origin) - cs.planes[i].distance;
		if (d > -half_epsilon_) {
			inside = false;
			break;
		}
		if (d > closest_dist) {
			closest_dist = d;
			closest_plane = i;
		}
	}
	if (inside && closest_plane >= 0) {
		c.time = zero_val;
		c.point.set(seg.origin);
		c.normal.set(cs.planes[closest_plane].normal);
		return;
	}

	for (int i = 0; i < static_cast<int>(cs.planes.size()); ++i) {
		T denom = dot(cs.planes[i].normal, seg.direction);
		if (denom >= zero_val)
			continue; // Only accept entry planes (segment moving into plane)
		T t = (cs.planes[i].distance - dot(cs.planes[i].normal, seg.origin)) / denom;
		if (t >= zero_val && t <= one) {
			vec3<T> u;
			mul(u, seg.direction, t);
			add(u, seg.origin);

			bool b = true;
			for (int j = 0; j < static_cast<int>(cs.planes.size()); ++j) {
				if (i == j)
					continue;
				if (dot(cs.planes[j].normal, u) - cs.planes[j].distance > zero_val) {
					b = false;
					break;
				}
			}
			if (b) {
				if (t < c.time) {
					c.time = t;
					c.point.set(u);
					c.normal.set(cs.planes[i].normal);
				}
			}
		}
	}
}

template <typename T>
void simulator<T>::trace_forward_convex(collision<T> & col,
                                        const segment<T> & seg,
                                        const solid<T> * s2,
                                        const vec3<T> & lp2,
                                        const hop::convex_solid<T> & inflated_cs,
                                        const vec3<T> & sh1_offset) {
	segment<T> tmp;
	tmp.set(seg);
	sub(tmp.origin, s2->get_position());
	sub(tmp.origin, lp2);
	add(tmp.origin, sh1_offset);
	trace_convex_solid(col, tmp, inflated_cs);
	if (col.time < tr::one()) {
		// Normalize col.point to s1's center at impact.
		vec3<T> travel;
		mul(travel, seg.direction, col.time);
		add(col.point, seg.origin, travel);
	}
}

template <typename T>
void simulator<T>::trace_inverted_convex(collision<T> & col,
                                         const segment<T> & seg,
                                         const solid<T> * s1,
                                         const solid<T> * s2,
                                         const vec3<T> & lp_delta,
                                         const hop::convex_solid<T> & inflated_cs,
                                         const vec3<T> & sh2_offset) {
	segment<T> tmp;
	mul(tmp.direction, seg.direction, -tr::one());
	tmp.origin.set(s2->position_);
	sub(tmp.origin, s1->get_position());
	add(tmp.origin, lp_delta);
	add(tmp.origin, sh2_offset);
	collision<T> icol;
	icol.time = tr::one();
	trace_convex_solid(icol, tmp, inflated_cs);
	if (icol.time < tr::one()) {
		col.time = icol.time;
		col.normal.set(icol.normal);
		neg(col.normal);
		// Normalize col.point to s1's center at impact.
		vec3<T> travel;
		mul(travel, seg.direction, icol.time);
		add(col.point, seg.origin, travel);
	}
}

template <typename T>
void simulator<T>::collision_friction(
    solid<T> * s, solid<T> * hit, const collision<T> & c, T normal_impulse, T one_over_mass) {
	T zero_val {};
	if (s->coefficient_of_dynamic_friction_ <= zero_val)
		return;
	T abs_impulse = normal_impulse < zero_val ? -normal_impulse : normal_impulse;
	if (abs_impulse <= zero_val)
		return;

	vec3<T> vtan;
	vec3<T> dir;

	// Tangential relative velocity (post-restitution)
	if (hit) {
		sub(vtan, s->velocity_, hit->velocity_);
	} else {
		vtan.set(s->velocity_);
	}
	mul(dir, c.normal, dot(vtan, c.normal));
	sub(vtan, dir);

	T tan_speed = length(vtan);
	if (tan_speed <= epsilon_)
		return;

	// Cap friction to not exceed what's needed to stop tangential motion
	T max_dv = s->coefficient_of_dynamic_friction_ * abs_impulse * one_over_mass;
	if (max_dv > tan_speed)
		max_dv = tan_speed;

	mul(dir, vtan, -(max_dv / tan_speed));
	add(s->velocity_, dir);
}

template <typename T>
void simulator<T>::friction_link(vec3<T> & result,
                                 solid<T> * s,
                                 const vec3<T> & solid_vel,
                                 solid<T> * hit,
                                 const vec3<T> & hit_normal,
                                 const vec3<T> & applied_force,
                                 T fdt) {
	result.reset();
	T zero_val {};
	if (s->mass_ > zero_val && hit->mass_ != zero_val &&
	    (s->coefficient_of_static_friction_ > zero_val || s->coefficient_of_dynamic_friction_ > zero_val)) {
		vec3<T> vr;
		vec3<T> ff;
		vec3<T> fs;
		vec3<T> norm_vr;

		T fn = (dot(gravity_, hit_normal) * s->coefficient_of_gravity_) * s->mass_ + dot(applied_force, hit_normal);

		sub(vr, solid_vel, hit->velocity_);
		mul(norm_vr, hit_normal, dot(vr, hit_normal));
		sub(vr, norm_vr);
		cap_vec3(vr, max_velocity_component_);
		T len_vr = length(vr);

		if (fn != zero_val && len_vr > zero_val && fdt > zero_val) {
			div(norm_vr, vr, len_vr);
			mul(ff, norm_vr, fn);
			mul(result, ff, s->coefficient_of_static_friction_);
			mul(result, fdt);

			mul(fs, vr, -s->mass_);
			mul(norm_vr, hit_normal, dot(applied_force, hit_normal));
			sub(norm_vr, applied_force, norm_vr);
			mul(norm_vr, fdt);
			add(fs, norm_vr);
			cap_vec3(fs, max_force_component_);

			if (length_squared(fs) > length_squared(result)) {
				mul(result, ff, s->coefficient_of_dynamic_friction_);
			} else {
				div(result, fs, fdt);
			}
		}
	}
}

template <typename T>
void simulator<T>::constraint_link(vec3<T> & result,
                                   solid<T> * s,
                                   const vec3<T> & solid_pos,
                                   const vec3<T> & solid_vel) {
	vec3<T> tx;
	vec3<T> tv;
	result.reset();

	for (auto * c : s->constraints_) {
		if (!c->is_active())
			continue;
		else if (s == c->start_solid_.get()) {
			if (c->end_solid_) {
				sub(tx, c->end_solid_->position_, solid_pos);
				sub(tv, c->end_solid_->velocity_, solid_vel);
			} else {
				sub(tx, c->end_point_, solid_pos);
				mul(tv, solid_vel, -tr::one());
			}
		} else if (s == c->end_solid_.get()) {
			sub(tx, c->start_solid_->position_, solid_pos);
			sub(tv, c->start_solid_->velocity_, solid_vel);
		} else {
			continue;
		}
		T dist = length(tx);
		if (dist > c->distance_threshold_) {
			T scale = (dist - c->distance_threshold_) / dist;
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
void simulator<T>::update_acceleration(vec3<T> & result, solid<T> * s, const vec3<T> & x, const vec3<T> & v, T fdt) {
	vec3<T> friction_force;
	vec3<T> constraint_force;
	vec3<T> fluid_force;
	T zero_val {};

	mul(result, gravity_, s->coefficient_of_gravity_);

	if (s->mass_ != zero_val) {
		constraint_link(constraint_force, s, x, v);
		add(constraint_force, s->force_);
		if (s->touched1_) {
			friction_link(friction_force, s, v, s->touched1_, s->touched1_normal_, constraint_force, fdt);
			add(constraint_force, friction_force);
			if (s->touched2_ && s->touched2_ != s->touched1_) {
				friction_link(friction_force, s, v, s->touched2_, s->touched2_normal_, constraint_force, fdt);
				add(constraint_force, friction_force);
			}
		}
		sub(fluid_force, fluid_velocity_, v);
		mul(fluid_force, s->coefficient_of_effective_drag_);
		add(constraint_force, fluid_force);
		mul(constraint_force, s->inv_mass_);
		add(result, constraint_force);
	}
}

template <typename T>
void simulator<T>::integration_step(solid<T> * s,
                                    const vec3<T> & x,
                                    const vec3<T> & v,
                                    const vec3<T> & dx,
                                    const vec3<T> & dv,
                                    T fdt,
                                    vec3<T> & result_x,
                                    vec3<T> & result_v) {
	vec3<T> tx;
	vec3<T> tv;
	mul(tx, dx, fdt);
	add(tx, x);
	mul(tv, dv, fdt);
	add(tv, v);
	result_x.set(tv);
	update_acceleration(result_v, s, tx, tv, fdt);
}

} // namespace hop
