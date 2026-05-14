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
		assert(!order || order->size() == solids_.size());

		const size_t num = target ? 1 : (order ? order->size() : solids_.size());
		for (size_t i = 0; i < num; ++i) {
			solid<T> * s;
			if (target)
				s = target;
			else if (order)
				s = (*order)[i];
			else
				s = solids_[i].get();

			if (!s->active_ || (scope != 0 && (s->scope_ & scope) == 0))
				continue;

			if (manager_)
				manager_->pre_update(s, dt);

			update_solid(s, dt);

			if (manager_)
				manager_->post_update(s, dt);
		}

		report_collisions();
		if (manager_)
			manager_->post_update(dt);
	}

	void update_solid(solid<T> * solid_ptr, T dt);

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
	void test_solid(collision<T> & result, solid<T> * s1, const segment<T> & seg, solid<T> * s2) {
		hop::test_solid(result, s1, seg, s2, epsilon_);
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
		deactivate_speed_ = tr::default_deactivate_speed(epsilon_);
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

	void friction_link(vec3<T> & result,
	                   solid<T> * s,
	                   const vec3<T> & solid_vel,
	                   solid<T> * hit,
	                   const vec3<T> & hit_normal,
	                   const vec3<T> & applied_force,
	                   T dt);
	// Apply Coulomb friction impulse at the moment of collision, opposing the
	// tangential relative velocity, capped by µ_d * |normal_impulse|.
	void collision_friction(solid<T> * s, solid<T> * hit, const collision<T> & c, T normal_impulse, T one_over_mass);
	void constraint_link(vec3<T> & result, solid<T> * s, const vec3<T> & solid_pos, const vec3<T> & solid_vel);
	void update_acceleration(vec3<T> & result, solid<T> * s, const vec3<T> & x, const vec3<T> & v, T dt);
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

	// Collision loop
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
			if (c.time == T {} && c.depth > T {}) {
				// Penetration at frame start: push old_pos out along the contact
				// normal by the full overlap depth. Split evenly if both bodies
				// are dynamic so each side's update contributes half the correction.
				T push = c.depth;
				if (c.collider && !c.collider->has_infinite_mass())
					push = c.depth * tr::half();
				vec3<T> correction;
				mul(correction, c.normal, push);
				add(old_pos, correction);
			} else {
				calculate_epsilon_offset(old_pos, left_over, c.normal);
				add(old_pos, c.point);
			}
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
				mul(temp, c.normal, epsilon_ * tr::four());
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
	if (c.time == one && loop == 0) {
		solid_ptr->touching_ = nullptr;
		solid_ptr->touched1_ = nullptr;
		solid_ptr->touched2_ = nullptr;
	}

	// Deactivation
	if (deactivate_count_ > 0 && solid_ptr->deactivate_count_ >= 0) {
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
					// Stay awake while the constraint is loaded — otherwise both
					// endpoints can sleep at non-equilibrium and freeze the system.
					if (con->is_loaded(deactivate_speed_))
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
			hop::merge_collision(result, col, epsilon_, average_normals_);
		}
	}

	if (manager_) {
		col.time = tr::one();
		manager_->trace_solid(col, s, seg, collide_with_bits);
		hop::merge_collision(result, col, epsilon_, average_normals_);
	}

	if (result.time == tr::one()) {
		seg.get_end_point(result.point);
		result.impact.set(result.point);
	}
}

template <typename T>
void simulator<T>::collision_friction(
    solid<T> * s, solid<T> * hit, const collision<T> & c, T normal_impulse, T one_over_mass) {
	T zero_val {};
	if (s->coefficient_of_dynamic_friction_ <= zero_val)
		return;
	T abs_impulse = tr::abs(normal_impulse);
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
                                 T dt) {
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

		if (fn != zero_val && len_vr > zero_val && dt > zero_val) {
			div(norm_vr, vr, len_vr);
			mul(ff, norm_vr, fn);
			mul(result, ff, s->coefficient_of_static_friction_);
			mul(result, dt);

			mul(fs, vr, -s->mass_);
			mul(norm_vr, hit_normal, dot(applied_force, hit_normal));
			sub(norm_vr, applied_force, norm_vr);
			mul(norm_vr, dt);
			add(fs, norm_vr);
			cap_vec3(fs, max_force_component_);

			if (length_squared(fs) > length_squared(result)) {
				mul(result, ff, s->coefficient_of_dynamic_friction_);
			} else {
				div(result, fs, dt);
			}
		}
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
	vec3<T> friction_force;
	vec3<T> constraint_force;
	vec3<T> fluid_force;
	T zero_val {};

	mul(result, gravity_, s->coefficient_of_gravity_);

	if (s->mass_ != zero_val) {
		constraint_link(constraint_force, s, x, v);
		add(constraint_force, s->force_);
		if (s->touched1_) {
			friction_link(friction_force, s, v, s->touched1_, s->touched1_normal_, constraint_force, dt);
			add(constraint_force, friction_force);
			if (s->touched2_ && s->touched2_ != s->touched1_) {
				friction_link(friction_force, s, v, s->touched2_, s->touched2_normal_, constraint_force, dt);
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
