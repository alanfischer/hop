#pragma once

#include <vector>
#include <algorithm>
#include <stdexcept>
#include <hop/math/math_ops.h>
#include <hop/math/intersect.h>
#include <hop/math/project.h>
#include <hop/math/bounding.h>
#include <hop/collision.h>
#include <hop/constraint.h>
#include <hop/solid.h>
#include <hop/manager.h>

namespace hop {

enum class integrator_type {
	euler,
	improved,
	heun,
	runge_kutta,
};

template<typename T>
class simulator {
public:
	using tr = scalar_traits<T>;

	enum { scope_report_collisions = 1 << 30 };

	simulator() {
		set_gravity({T{}, T{}, -tr::from_milli(9810)});
		init_epsilon_defaults();
		collisions_.resize(64);
	}

	~simulator() = default;

	// Epsilon
	void set_epsilon(T epsilon) {
		if constexpr (std::is_same_v<T, float>) {
			tr::make_epsilon(epsilon_state_, epsilon);
		}
		epsilon_ = epsilon_state_.epsilon;
		half_epsilon_ = epsilon_state_.half_epsilon;
		quarter_epsilon_ = epsilon_state_.quarter_epsilon;
	}

	void set_epsilon_bits(int bits) {
		static_assert(std::is_same_v<T, fixed16>, "set_epsilon_bits only for fixed16");
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

	void set_fluid_velocity(const vec3<T>& v) { fluid_velocity_ = v; }
	const vec3<T>& get_fluid_velocity() const { return fluid_velocity_; }

	void set_gravity(const vec3<T>& g) {
		gravity_ = g;
		for (auto& s : solids_) s->activate();
	}
	const vec3<T>& get_gravity() const { return gravity_; }

	void set_manager(manager<T>* m) { manager_ = m; }
	manager<T>* get_manager() const { return manager_; }

	void set_micro_collision_threshold(T t) { micro_collision_threshold_ = t; }
	T get_micro_collision_threshold() const { return micro_collision_threshold_; }

	void set_deactivate_speed(T s) { deactivate_speed_ = s; }
	void set_deactivate_count(int c) { deactivate_count_ = c; }

	// Solid management
	void add_solid(std::shared_ptr<solid<T>> s) {
		for (auto& existing : solids_) {
			if (existing == s) return;
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

		for (auto& other : solids_) {
			if (other->touching_ == s.get()) other->touching_ = nullptr;
			if (other->touched1_ == s.get()) other->touched1_ = nullptr;
			if (other->touched2_ == s.get()) other->touched2_ = nullptr;
		}

		if (reporting_collisions_) {
			for (int i = 0; i < num_collisions_; ++i) {
				auto& c = collisions_[i];
				if (c.collider.get() == s.get()) c.collider = nullptr;
				if (c.collidee.get() == s.get()) c.collidee = nullptr;
			}
		}

		s->internal_set_simulator(nullptr);
		solids_.erase(std::remove(solids_.begin(), solids_.end(), s), solids_.end());
	}

	int get_num_solids() const { return static_cast<int>(solids_.size()); }
	solid<T>* get_solid(int i) const { return solids_[i].get(); }

	// Constraint management
	void add_constraint(typename constraint<T>::ptr c) {
		for (auto& existing : constraints_) {
			if (existing == c) return;
		}
		constraints_.push_back(c);
		c->internal_set_simulator(this);
	}

	void remove_constraint(typename constraint<T>::ptr c) {
		c->internal_set_simulator(nullptr);
		constraints_.erase(std::remove(constraints_.begin(), constraints_.end(), c), constraints_.end());
	}

	// Main update
	void update(int dt, int scope = 0, solid<T>* target = nullptr) {
		T fdt = tr::from_milli(dt);
		num_collisions_ = 0;
		if (manager_) manager_->pre_update(dt, fdt);

		int num = (target != nullptr) ? 1 : static_cast<int>(solids_.size());
		for (int i = 0; i < num; ++i) {
			solid<T>* s = (num > 1 || target == nullptr) ? solids_[i].get() : target;

			if (!s->active_ || (scope != 0 && (s->scope_ & scope) == 0)) continue;

			s->last_dt_ = dt;
			if (s->do_update_callback_ && (s->manager_ || manager_)) {
				(s->manager_ ? s->manager_ : manager_)->pre_update(s, dt, fdt);
			}

			update_solid(s, dt, fdt);

			if (s->do_update_callback_ && manager_) {
				(s->manager_ ? s->manager_ : manager_)->post_update(s, dt, fdt);
			}
		}

		if (scope & scope_report_collisions) report_collisions();
		if (manager_) manager_->post_update(dt, fdt);
	}

	void update_solid(solid<T>* solid_ptr, int dt, T fdt);

	// Find solids in box
	int find_solids_in_aa_box(const aa_box<T>& box, solid<T>* solids[], int max_solids) const {
		aa_box<T> expanded(box);
		expanded.mins.x -= epsilon_; expanded.mins.y -= epsilon_; expanded.mins.z -= epsilon_;
		expanded.maxs.x += epsilon_; expanded.maxs.y += epsilon_; expanded.maxs.z += epsilon_;

		int amount = -1;
		if (manager_) amount = manager_->find_solids_in_aa_box(expanded, solids, max_solids);

		if (amount == -1) {
			amount = 0;
			for (auto& s : solids_) {
				if (test_intersection(expanded, s->world_bound_)) {
					if (amount < max_solids) solids[amount] = s.get();
					amount++;
				}
			}
		}
		if (amount > max_solids) amount = max_solids;
		return amount;
	}

	// Trace / test
	void trace_segment(collision<T>& result, const segment<T>& seg, int collide_with_bits = -1, solid<T>* ignore = nullptr);
	void trace_solid(collision<T>& result, solid<T>* s, const segment<T>& seg, int collide_with_bits = -1);
	void test_segment(collision<T>& result, const segment<T>& seg, solid<T>* s);
	void test_solid(collision<T>& result, solid<T>* s1, const segment<T>& seg, solid<T>* s2);

	// Utility
	void cap_vec3(vec3<T>& v, T value) const {
		v.x = tr::cap(v.x, value);
		v.y = tr::cap(v.y, value);
		v.z = tr::cap(v.z, value);
	}

	void calculate_epsilon_offset(vec3<T>& result, const vec3<T>& direction, const vec3<T>& normal) const {
		if (snap_to_grid_) {
			result.x = (normal.x >= quarter_epsilon_) ? epsilon_ : ((normal.x <= -quarter_epsilon_) ? -epsilon_ : T{});
			result.y = (normal.y >= quarter_epsilon_) ? epsilon_ : ((normal.y <= -quarter_epsilon_) ? -epsilon_ : T{});
			result.z = (normal.z >= quarter_epsilon_) ? epsilon_ : ((normal.z <= -quarter_epsilon_) ? -epsilon_ : T{});
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

	void snap_to_grid_vec(vec3<T>& pos) const {
		if (snap_to_grid_) {
			tr::snap_to_grid(pos.x, epsilon_state_);
			tr::snap_to_grid(pos.y, epsilon_state_);
			tr::snap_to_grid(pos.z, epsilon_state_);
		}
	}

	bool too_small(const vec3<T>& v, T epsilon) const {
		return v.x < epsilon && v.x > -epsilon &&
		       v.y < epsilon && v.y > -epsilon &&
		       v.z < epsilon && v.z > -epsilon;
	}

	int count_active_solids() const {
		int n = 0;
		for (auto& s : solids_) if (s->active_) n++;
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
	void trace_segment_with_current_spacials(collision<T>& result, const segment<T>& seg, int collide_with_bits, solid<T>* ignore);
	void trace_solid_with_current_spacials(collision<T>& result, solid<T>* s, const segment<T>& seg, int collide_with_bits);
	void trace_aa_box(collision<T>& c, const segment<T>& seg, const aa_box<T>& box);
	void trace_sphere(collision<T>& c, const segment<T>& seg, const hop::sphere<T>& sph);
	void trace_capsule(collision<T>& c, const segment<T>& seg, const hop::capsule<T>& cap);
	void trace_convex_solid(collision<T>& c, const segment<T>& seg, const hop::convex_solid<T>& cs);
	void friction_link(vec3<T>& result, solid<T>* s, const vec3<T>& solid_vel, solid<T>* hit, const vec3<T>& hit_normal, const vec3<T>& applied_force, T fdt);
	void constraint_link(vec3<T>& result, solid<T>* s, const vec3<T>& solid_pos, const vec3<T>& solid_vel);
	void update_acceleration(vec3<T>& result, solid<T>* s, const vec3<T>& x, const vec3<T>& v, T fdt);
	void integration_step(solid<T>* s, const vec3<T>& x, const vec3<T>& v, const vec3<T>& dx, const vec3<T>& dv, T fdt, vec3<T>& result_x, vec3<T>& result_v);

	integrator_type integrator_ = integrator_type::heun;
	vec3<T> fluid_velocity_;
	vec3<T> gravity_;
	epsilon_state<T> epsilon_state_;
	T epsilon_{};
	T half_epsilon_{};
	T quarter_epsilon_{};
	bool snap_to_grid_ = false;
	bool average_normals_ = false;
	T max_position_component_{};
	T max_velocity_component_{};
	T max_force_component_{};
	std::vector<collision<T>> collisions_;
	int num_collisions_ = 0;
	std::vector<std::shared_ptr<solid<T>>> solids_;
	std::vector<typename constraint<T>::ptr> constraints_;
	std::vector<solid<T>*> spacial_collection_;
	int num_spacial_collection_ = 0;
	bool reporting_collisions_ = false;
	T micro_collision_threshold_ = tr::one();
	T deactivate_speed_{};
	int deactivate_count_ = 0;
	manager<T>* manager_ = nullptr;

	// Cache temporaries
	vec3<T> cache_update_old_position_;
	vec3<T> cache_update_new_position_;
	vec3<T> cache_update_old_velocity_;
	vec3<T> cache_update_velocity_;
	vec3<T> cache_update_temp_;
	vec3<T> cache_update_t_;
	vec3<T> cache_update_left_over_;
	vec3<T> cache_update_dx1_;
	vec3<T> cache_update_dx2_;
	vec3<T> cache_update_dv1_;
	vec3<T> cache_update_dv2_;
	segment<T> cache_update_path_;
	aa_box<T> cache_update_box_;
	collision<T> cache_update_c_;
	vec3<T> cache_trace_segment_end_point_;
	aa_box<T> cache_trace_segment_total_;
	collision<T> cache_trace_segment_collision_;
	collision<T> cache_trace_solid_collision_;
	collision<T> cache_test_solid_collision_;
	aa_box<T> cache_test_solid_box_;
	aa_box<T> cache_test_solid_box1_;
	vec3<T> cache_test_solid_origin_;
	hop::sphere<T> cache_test_solid_sphere_;
	hop::capsule<T> cache_test_solid_capsule_;
	vec3<T> cache_test_solid_direction_;
	collision<T> cache_test_segment_collision_;
	aa_box<T> cache_test_segment_box_;
	hop::sphere<T> cache_test_segment_sphere_;
	hop::capsule<T> cache_test_segment_capsule_;
	vec3<T> cache_trace_sphere_n_;
	vec3<T> cache_trace_capsule_p1_;
	vec3<T> cache_trace_capsule_p2_;
	segment<T> cache_trace_capsule_s_;
	hop::sphere<T> cache_trace_capsule_sphere_;
	vec3<T> cache_friction_link_vr_;
	vec3<T> cache_friction_link_ff_;
	vec3<T> cache_friction_link_fs_;
	vec3<T> cache_friction_link_norm_vr_;
	vec3<T> cache_constraint_link_tx_;
	vec3<T> cache_constraint_link_tv_;
	vec3<T> cache_update_acceleration_friction_force_;
	vec3<T> cache_update_acceleration_constraint_force_;
	vec3<T> cache_update_acceleration_fluid_force_;
	vec3<T> cache_integration_step_tx_;
	vec3<T> cache_integration_step_tv_;
};

// ============================================================================
// Implementation
// ============================================================================

template<typename T>
void simulator<T>::update_solid(solid<T>* solid_ptr, int dt, T fdt) {
	auto& old_pos = cache_update_old_position_;
	auto& new_pos = cache_update_new_position_;
	auto& old_vel = cache_update_old_velocity_;
	auto& vel = cache_update_velocity_;
	auto& temp = cache_update_temp_;
	auto& t = cache_update_t_;
	auto& left_over = cache_update_left_over_;
	auto& dx1 = cache_update_dx1_;
	auto& dx2 = cache_update_dx2_;
	auto& dv1 = cache_update_dv1_;
	auto& dv2 = cache_update_dv2_;
	auto& path = cache_update_path_;
	T cor{}, impulse{}, one_over_mass{}, one_over_hit_mass{};
	int loop = 0;
	solid<T>* hit_solid = nullptr;
	auto& c = cache_update_c_.reset();

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
	}
	else if (integrator_ == integrator_type::improved) {
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
	}
	else if (integrator_ == integrator_type::heun) {
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
	}
	else if (integrator_ == integrator_type::runge_kutta) {
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

	if (solid_ptr->do_update_callback_ && (solid_ptr->manager_ || manager_)) {
		(solid_ptr->manager_ ? solid_ptr->manager_ : manager_)->intra_update(solid_ptr, dt, fdt);
	}

	snap_to_grid_vec(old_pos);
	cap_vec3(old_pos, max_position_component_);
	snap_to_grid_vec(new_pos);
	cap_vec3(new_pos, max_position_component_);

	// Collect spacials
	if (solid_ptr->collide_with_scope_ != 0) {
		sub(temp, new_pos, old_pos);
		{
			if (temp.x < T{}) temp.x = -temp.x;
			if (temp.y < T{}) temp.y = -temp.y;
			if (temp.z < T{}) temp.z = -temp.z;

			T m = temp.x;
			if (temp.y > m) m = temp.y;
			if (temp.z > m) m = temp.z;
			m = m + epsilon_;

			auto& box = cache_update_box_.set(solid_ptr->local_bound_);
			add(box, new_pos);
			box.mins.x -= m; box.mins.y -= m; box.mins.z -= m;
			box.maxs.x += m; box.maxs.y += m; box.maxs.z += m;

			num_spacial_collection_ = find_solids_in_aa_box(box, spacial_collection_.data(), static_cast<int>(spacial_collection_.size()));
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
			if (c.collider.get() != solid_ptr->touching_ &&
				(solid_ptr->collision_listener_ != nullptr || (c.collider && c.collider->collision_listener_ != nullptr)))
			{
				c.collidee = solid_ptr->shared_from_this();
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
			hit_solid = c.collider.get();

			bool responded = false;
			if (solid_ptr->do_update_callback_) {
				responded = (solid_ptr->manager_ ? solid_ptr->manager_ : manager_)->collision_response(solid_ptr, old_pos, left_over, c);
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
					cor = T{};
				}

				T numerator = (one + cor) * dot(temp, c.normal);
				temp.reset();

				T zero_val{};
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
					}
					if (hit_solid && hit_solid->mass_ != solid<T>::infinite_mass()) {
						mul(temp, c.normal, impulse);
						mul(temp, one_over_hit_mass);
					}
				}
				else if (hit_solid) {
					mul(temp, c.normal, numerator);
				}
				else if (solid_ptr->mass_ == zero_val) {
					mul(t, c.normal, numerator);
					add(solid_ptr->velocity_, t);
				}

				if (hit_solid && (hit_solid->collide_with_scope_ & solid_ptr->collision_scope_) != 0 &&
					(tr::abs(temp.x) >= deactivate_speed_ || tr::abs(temp.y) >= deactivate_speed_ || tr::abs(temp.z) >= deactivate_speed_))
				{
					hit_solid->activate();
					sub(hit_solid->velocity_, temp);
				}
			}

			// Touching code
			solid_ptr->touched2_ = solid_ptr->touched1_;
			solid_ptr->touched2_normal_.set(solid_ptr->touched1_normal_);
			if (solid_ptr->touched1_ == c.collider.get()) {
				solid_ptr->touching_ = c.collider.get();
				solid_ptr->touching_normal_.set(c.normal);
			} else {
				solid_ptr->touched1_ = c.collider.get();
				solid_ptr->touched1_normal_.set(c.normal);
				solid_ptr->touching_ = nullptr;
			}

			if (too_small(left_over, epsilon_)) {
				new_pos.set(old_pos);
				break;
			} else if (loop > 4) {
				solid_ptr->velocity_.reset();
				new_pos.set(old_pos);
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
			tr::abs(new_pos.z - solid_ptr->position_.z) < deactivate_speed_)
		{
			solid_ptr->deactivate_count_++;
			if (solid_ptr->deactivate_count_ > deactivate_count_) {
				int j;
				for (j = static_cast<int>(solid_ptr->constraints_.size()) - 1; j >= 0; --j) {
					auto* con = solid_ptr->constraints_[j];
					auto* start = con->start_solid_.get();
					auto* end = con->end_solid_.get();
					if (start != solid_ptr) {
						if (start->active_ && start->deactivate_count_ <= deactivate_count_) break;
					} else if (end) {
						if (end->active_ && end->deactivate_count_ <= deactivate_count_) break;
					}
				}
				if (j < 0) solid_ptr->deactivate();
			}
		} else {
			solid_ptr->deactivate_count_ = 0;
		}
	}

	solid_ptr->set_position_direct(new_pos);
}

template<typename T>
void simulator<T>::report_collisions() {
	reporting_collisions_ = true;
	for (int i = 0; i < num_collisions_; ++i) {
		auto& col = collisions_[i];
		if (col.collidee) {
			auto* listener = col.collidee->collision_listener_;
			if (listener && col.collider && (col.collidee->collide_with_scope_ & col.collider->collision_scope_) != 0) {
				listener->on_collision(col);
			}
		}
		if (col.collider) {
			auto* listener = col.collider->collision_listener_;
			if (listener && col.collidee && (col.collider->collide_with_scope_ & col.collidee->collision_scope_) != 0) {
				collision<T> inverted;
				inverted.set(col);
				inverted.invert();
				listener->on_collision(inverted);
			}
		}
	}
	num_collisions_ = 0;
	reporting_collisions_ = false;
}

template<typename T>
void simulator<T>::trace_segment(collision<T>& result, const segment<T>& seg, int collide_with_bits, solid<T>* ignore) {
	auto& ep = cache_trace_segment_end_point_;
	seg.get_end_point(ep);
	auto& total = cache_trace_segment_total_.set(seg.origin, seg.origin);
	total.merge(ep);
	num_spacial_collection_ = find_solids_in_aa_box(total, spacial_collection_.data(), static_cast<int>(spacial_collection_.size()));
	trace_segment_with_current_spacials(result, seg, collide_with_bits, ignore);
}

template<typename T>
void simulator<T>::trace_solid(collision<T>& result, solid<T>* s, const segment<T>& seg, int collide_with_bits) {
	auto& end = cache_test_solid_origin_;
	seg.get_end_point(end);
	auto& box = cache_test_solid_box_.set(seg.origin, seg.origin);
	box.merge(end);
	add(box.mins, s->local_bound_.mins);
	add(box.maxs, s->local_bound_.maxs);
	num_spacial_collection_ = find_solids_in_aa_box(box, spacial_collection_.data(), static_cast<int>(spacial_collection_.size()));
	trace_solid_with_current_spacials(result, s, seg, collide_with_bits);
}

template<typename T>
void simulator<T>::test_segment(collision<T>& result, const segment<T>& seg, solid<T>* s) {
	auto& col = cache_test_segment_collision_.reset();
	col.collider = s->shared_from_this();
	const T one = tr::one();

	auto& shapes = s->shapes_;
	int n = static_cast<int>(shapes.size());
	bool modify_scope = false;

	for (int i = 0; i < n; ++i) {
		auto* sh = shapes[i].get();
		switch (sh->type_) {
			case shape_type::aa_box: {
				auto& box = cache_test_segment_box_.set(sh->aa_box_);
				add(box, s->position_);
				trace_aa_box(col, seg, box);
				break;
			}
			case shape_type::sphere: {
				auto& sph = cache_test_segment_sphere_.set(sh->sphere_);
				add(sph, s->position_);
				trace_sphere(col, seg, sph);
				break;
			}
			case shape_type::capsule: {
				auto& cap = cache_test_segment_capsule_.set(sh->capsule_);
				add(cap, s->position_);
				trace_capsule(col, seg, cap);
				break;
			}
			case shape_type::convex_solid:
				throw std::logic_error("trace_segment not implemented for convex_solid");
			case shape_type::traceable:
				sh->traceable_->trace_segment(col, s->position_, seg);
				modify_scope = true;
				break;
		}

		// Segment traces have no Minkowski expansion: impact == point
		if (col.time < one) col.impact.set(col.point);

		if (col.time == T{}) col.scope |= s->internal_scope_;

		int scope = result.scope;
		if (col.time < one) {
			if (col.time < result.time) {
				result.set(col);
			} else if (result.time == col.time) {
				add(result.normal, col.normal);
				if (!normalize_carefully(result.normal, epsilon_)) result.set(col);
			}
			modify_scope |= (col.time == T{});
		}

		result.scope = modify_scope ? (scope | col.scope) : scope;
	}
}

template<typename T>
void simulator<T>::test_solid(collision<T>& result, solid<T>* s1, const segment<T>& seg, solid<T>* s2) {
	auto& col = cache_test_solid_collision_.reset();
	col.collider = s2->shared_from_this();
	const T one = tr::one();
	T zero_val{};

	auto& shapes1 = s1->shapes_;
	auto& shapes2 = s2->shapes_;
	int n1 = static_cast<int>(shapes1.size());
	int n2 = static_cast<int>(shapes2.size());
	bool modify_scope = false;

	for (int i = 0; i < n1; ++i) {
		auto* sh1 = shapes1[i].get();
		for (int j = 0; j < n2; ++j) {
			auto* sh2 = shapes2[j].get();
			modify_scope = false;

			// AABox vs *
			if (sh1->type_ == shape_type::aa_box && sh2->type_ == shape_type::aa_box) {
				auto& box = cache_test_solid_box_.set(sh2->aa_box_);
				add(box, s2->position_);
				sub(box.maxs, sh1->aa_box_.mins);
				sub(box.mins, sh1->aa_box_.maxs);
				trace_aa_box(col, seg, box);
			}
			else if (sh1->type_ == shape_type::aa_box && sh2->type_ == shape_type::sphere) {
				auto& box = cache_test_solid_box_.set(sh2->sphere_.radius);
				add(box, sh2->sphere_.origin);
				add(box, s2->position_);
				sub(box.maxs, sh1->aa_box_.mins);
				sub(box.mins, sh1->aa_box_.maxs);
				trace_aa_box(col, seg, box);
			}
			else if (sh1->type_ == shape_type::aa_box && sh2->type_ == shape_type::capsule) {
				auto& box = cache_test_solid_box_;
				sh2->get_bound(box);
				add(box, s2->position_);
				sub(box.maxs, sh1->aa_box_.mins);
				sub(box.mins, sh1->aa_box_.maxs);
				trace_aa_box(col, seg, box);
			}
			// Sphere vs *
			else if (sh1->type_ == shape_type::sphere && sh2->type_ == shape_type::aa_box) {
				auto& box1 = cache_test_solid_box1_.set(sh1->sphere_.radius);
				add(box1, sh1->sphere_.origin);
				auto& box = cache_test_solid_box_.set(sh2->aa_box_);
				add(box, s2->position_);
				sub(box.maxs, box1.mins);
				sub(box.mins, box1.maxs);
				trace_aa_box(col, seg, box);
			}
			else if (sh1->type_ == shape_type::sphere && sh2->type_ == shape_type::sphere) {
				auto& origin = cache_test_solid_origin_.set(s2->position_);
				sub(origin, sh1->sphere_.origin);
				add(origin, sh2->sphere_.origin);
				auto& sph = cache_test_solid_sphere_.set(origin, sh2->sphere_.radius + sh1->sphere_.radius);
				trace_sphere(col, seg, sph);
			}
			else if (sh1->type_ == shape_type::sphere && sh2->type_ == shape_type::capsule) {
				auto& origin = cache_test_solid_origin_.set(s2->position_);
				sub(origin, sh1->sphere_.origin);
				add(origin, sh2->capsule_.origin);
				auto& cap = cache_test_solid_capsule_.set(origin, sh2->capsule_.direction, sh2->capsule_.radius + sh1->sphere_.radius);
				trace_capsule(col, seg, cap);
			}
			else if (sh1->type_ == shape_type::sphere && sh2->type_ == shape_type::convex_solid) {
				hop::convex_solid<T> cs;
				cs.set(sh2->convex_solid_);
				for (auto& p : cs.planes) p.distance = p.distance + sh1->sphere_.radius;
				segment<T> tmp;
				tmp.set(seg);
				sub(tmp.origin, s2->get_position());
				add(tmp.origin, sh1->sphere_.origin);
				trace_convex_solid(col, tmp, cs);
				if (col.time < one) {
					add(col.point, s2->get_position());
				}
			}
			// Capsule vs *
			else if (sh1->type_ == shape_type::capsule && sh2->type_ == shape_type::aa_box) {
				auto& box1 = cache_test_solid_box1_;
				sh1->get_bound(box1);
				auto& box = cache_test_solid_box_.set(sh2->aa_box_);
				add(box, s2->position_);
				sub(box.maxs, box1.mins);
				sub(box.mins, box1.maxs);
				trace_aa_box(col, seg, box);
			}
			else if (sh1->type_ == shape_type::capsule && sh2->type_ == shape_type::sphere) {
				auto& origin = cache_test_solid_origin_.set(s2->position_);
				sub(origin, sh1->capsule_.origin);
				add(origin, sh2->sphere_.origin);
				auto& dir = cache_test_solid_direction_.set(sh1->capsule_.direction);
				neg(dir);
				auto& cap = cache_test_solid_capsule_.set(origin, dir, sh1->capsule_.radius + sh2->sphere_.radius);
				trace_capsule(col, seg, cap);
			}
			else if (sh1->type_ == shape_type::capsule && sh2->type_ == shape_type::capsule) {
				auto& origin = cache_test_solid_origin_.set(s2->position_);
				sub(origin, sh1->capsule_.origin);
				add(origin, sh2->capsule_.origin);
				auto& cap = cache_test_solid_capsule_.set(origin, sh2->capsule_.direction, sh1->capsule_.radius + sh2->capsule_.radius);
				trace_capsule(col, seg, cap);
			}
			// Traceable
			else if (sh1->type_ == shape_type::traceable && sh2->type_ != shape_type::traceable) {
				segment<T> iseg;
				iseg.origin.set(s2->position_);
				mul(iseg.direction, seg.direction, -tr::one());
				sh1->traceable_->trace_solid(col, s2, seg.origin, iseg);
				col.invert();
				sub(iseg.origin, col.point);
				add(col.point, seg.origin, iseg.origin);
				modify_scope = true;
			}
			else if (sh1->type_ != shape_type::traceable && sh2->type_ == shape_type::traceable) {
				sh2->traceable_->trace_solid(col, s1, s2->position_, seg);
				modify_scope = true;
			}

			// Compute impact point for solid traces
			if (col.time < one && sh1->type_ != shape_type::traceable && sh2->type_ != shape_type::traceable) {
				vec3<T> sup;
				vec3<T> neg_n;
				neg(neg_n, col.normal);
				switch (sh1->type_) {
					case shape_type::aa_box:
						support(sup, sh1->aa_box_, neg_n);
						break;
					case shape_type::sphere:
						support(sup, sh1->sphere_, neg_n);
						break;
					case shape_type::capsule:
						support(sup, sh1->capsule_, neg_n);
						break;
					default:
						sup.reset();
						break;
				}
				add(col.impact, col.point, sup);
			} else if (col.time < one) {
				col.impact.set(col.point);
			}

			if (sh1->type_ != shape_type::traceable && sh2->type_ != shape_type::traceable && col.time == zero_val) {
				col.scope = s2->scope_;
			}
			if (col.time == zero_val) {
				col.scope |= s2->internal_scope_;
			}

			int scope = result.scope;
			if (col.time < one) {
				if (col.time < result.time) {
					result.set(col);
				} else if (result.time == col.time) {
					add(result.normal, col.normal);
					if (!normalize_carefully(result.normal, epsilon_)) result.set(col);
				}
				modify_scope |= (col.time == zero_val);
			}
			result.scope = modify_scope ? (scope | col.scope) : scope;
		}
	}
}

template<typename T>
void simulator<T>::trace_segment_with_current_spacials(collision<T>& result, const segment<T>& seg, int collide_with_bits, solid<T>* ignore) {
	result.time = tr::one();
	result.scope = 0;

	auto& col = cache_trace_segment_collision_.reset();
	for (int i = 0; i < num_spacial_collection_; ++i) {
		auto* s2 = spacial_collection_[i];
		if (s2 != ignore && (collide_with_bits & s2->collision_scope_) != 0) {
			col.time = tr::one();
			test_segment(col, seg, s2);
			int scope = result.scope;
			if (col.time < tr::one()) {
				if (col.time < result.time) result.set(col);
				else if (average_normals_ && result.time == col.time) {
					add(result.normal, col.normal);
					if (!normalize_carefully(result.normal, epsilon_)) result.set(col);
				}
			}
			result.scope = scope | col.scope;
		}
	}

	if (manager_) {
		col.time = tr::one();
		manager_->trace_segment(col, seg, collide_with_bits);
		int scope = result.scope;
		if (col.time < tr::one()) {
			if (col.time < result.time) result.set(col);
			else if (average_normals_ && result.time == col.time) {
				add(result.normal, col.normal);
				if (!normalize_carefully(result.normal, epsilon_)) result.set(col);
			}
		}
		result.scope = scope | col.scope;
	}

	if (result.time == tr::one()) {
		seg.get_end_point(result.point);
		result.impact.set(result.point);
	}
}

template<typename T>
void simulator<T>::trace_solid_with_current_spacials(collision<T>& result, solid<T>* s, const segment<T>& seg, int collide_with_bits) {
	result.time = tr::one();
	if (collide_with_bits == 0) return;

	auto& col = cache_trace_solid_collision_.reset();
	for (int i = 0; i < num_spacial_collection_; ++i) {
		auto* s2 = spacial_collection_[i];
		if (s != s2 && (collide_with_bits & s2->collision_scope_) != 0) {
			col.time = tr::one();
			test_solid(col, s, seg, s2);
			int scope = result.scope;
			if (col.time < tr::one()) {
				if (col.time < result.time) result.set(col);
				else if (average_normals_ && result.time == col.time) {
					add(result.normal, col.normal);
					if (!normalize_carefully(result.normal, epsilon_)) result.set(col);
				}
			}
			result.scope = scope | col.scope;
		}
	}

	if (manager_) {
		col.time = tr::one();
		manager_->trace_solid(col, s, seg, collide_with_bits);
		int scope = result.scope;
		if (col.time < tr::one()) {
			if (col.time < result.time) result.set(col);
			else if (average_normals_ && result.time == col.time) {
				add(result.normal, col.normal);
				if (!normalize_carefully(result.normal, epsilon_)) result.set(col);
			}
		}
		result.scope = scope | col.scope;
	}

	if (result.time == tr::one()) {
		seg.get_end_point(result.point);
		result.impact.set(result.point);
	}
}

template<typename T>
void simulator<T>::trace_aa_box(collision<T>& c, const segment<T>& seg, const aa_box<T>& box) {
	const T one = tr::one();
	if (test_inside(box, seg.origin)) {
		if (length_squared(seg.direction) > T{}) {
			T x;
			T dix, diy, diz, dax, day, daz;
			x = seg.origin.x - box.mins.x; dix = tr::abs(x);
			x = seg.origin.y - box.mins.y; diy = tr::abs(x);
			x = seg.origin.z - box.mins.z; diz = tr::abs(x);
			x = seg.origin.x - box.maxs.x; dax = tr::abs(x);
			x = seg.origin.y - box.maxs.y; day = tr::abs(x);
			x = seg.origin.z - box.maxs.z; daz = tr::abs(x);

			auto neg_x = constants<T>::neg_x_unit_vec3();
			auto neg_y = constants<T>::neg_y_unit_vec3();
			auto neg_z = constants<T>::neg_z_unit_vec3();
			auto pos_x = constants<T>::x_unit_vec3();
			auto pos_y = constants<T>::y_unit_vec3();
			auto pos_z = constants<T>::z_unit_vec3();

			if (dix <= diy && dix <= diz && dix <= dax && dix <= day && dix <= daz) {
				if (dot(seg.direction, neg_x) >= T{}) return;
				c.normal.set(neg_x);
			} else if (diy <= diz && diy <= dax && diy <= day && diy <= daz) {
				if (dot(seg.direction, neg_y) >= T{}) return;
				c.normal.set(neg_y);
			} else if (diz <= dax && diz <= day && diz <= daz) {
				if (dot(seg.direction, neg_z) >= T{}) return;
				c.normal.set(neg_z);
			} else if (dax <= day && dax <= daz) {
				if (dot(seg.direction, pos_x) >= T{}) return;
				c.normal.set(pos_x);
			} else if (day <= daz) {
				if (dot(seg.direction, pos_y) >= T{}) return;
				c.normal.set(pos_y);
			} else {
				if (dot(seg.direction, pos_z) >= T{}) return;
				c.normal.set(pos_z);
			}
		}
		c.time = T{};
		c.point.set(seg.origin);
	} else {
		c.time = find_intersection(seg, box, c.point, c.normal);
	}
}

template<typename T>
void simulator<T>::trace_sphere(collision<T>& c, const segment<T>& seg, const hop::sphere<T>& sph) {
	const T one = tr::one();
	if (test_inside(sph, seg.origin)) {
		auto& n = cache_trace_sphere_n_.set(seg.origin);
		sub(n, sph.origin);
		if (!normalize_carefully(n, epsilon_)) {
			// Origin exactly at sphere center — use negative direction as normal
			normalize(n, seg.direction);
			neg(n);
		}
		if (dot(n, seg.direction) <= epsilon_) {
			c.time = T{};
			c.point.set(seg.origin);
			c.normal.set(n);
		} else {
			c.time = one;
		}
	} else {
		c.time = find_intersection(seg, sph, c.point, c.normal);
	}
}

template<typename T>
void simulator<T>::trace_capsule(collision<T>& c, const segment<T>& seg, const hop::capsule<T>& cap) {
	auto& p1 = cache_trace_capsule_p1_.reset();
	auto& p2 = cache_trace_capsule_p2_.reset();
	auto& s = cache_trace_capsule_s_;
	s.origin.set(cap.origin);
	s.direction.set(cap.direction);
	project(p1, p2, s, seg, epsilon_);
	auto& sph = cache_trace_capsule_sphere_.set(p1, cap.radius);
	trace_sphere(c, seg, sph);
}

template<typename T>
void simulator<T>::trace_convex_solid(collision<T>& c, const segment<T>& seg, const hop::convex_solid<T>& cs) {
	const T one = tr::one();
	T zero_val{};
	c.time = one;

	// Check if segment origin is inside the convex solid (all planes)
	bool inside = true;
	T closest_dist = -tr::default_max_position_component();
	int closest_plane = -1;
	for (int i = 0; i < static_cast<int>(cs.planes.size()); ++i) {
		T d = dot(cs.planes[i].normal, seg.origin) - cs.planes[i].distance;
		if (d > zero_val) { inside = false; break; }
		if (d > closest_dist) { closest_dist = d; closest_plane = i; }
	}
	if (inside && closest_plane >= 0) {
		c.time = zero_val;
		c.point.set(seg.origin);
		c.normal.set(cs.planes[closest_plane].normal);
		return;
	}

	for (int i = 0; i < static_cast<int>(cs.planes.size()); ++i) {
		T denom = dot(cs.planes[i].normal, seg.direction);
		if (denom >= zero_val) continue; // Only accept entry planes (segment moving into plane)
		T t = (cs.planes[i].distance - dot(cs.planes[i].normal, seg.origin)) / denom;
		if (t >= zero_val && t <= one) {
			vec3<T> u;
			mul(u, seg.direction, t);
			add(u, seg.origin);

			bool b = true;
			for (int j = 0; j < static_cast<int>(cs.planes.size()); ++j) {
				if (i == j) continue;
				if (dot(cs.planes[j].normal, u) - cs.planes[j].distance > zero_val) { b = false; break; }
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

template<typename T>
void simulator<T>::friction_link(vec3<T>& result, solid<T>* s, const vec3<T>& solid_vel, solid<T>* hit, const vec3<T>& hit_normal, const vec3<T>& applied_force, T fdt) {
	result.reset();
	T zero_val{};
	if (s->mass_ > zero_val && hit->mass_ != zero_val && (s->coefficient_of_static_friction_ > zero_val || s->coefficient_of_dynamic_friction_ > zero_val)) {
		auto& vr = cache_friction_link_vr_;
		auto& ff = cache_friction_link_ff_;
		auto& fs = cache_friction_link_fs_;
		auto& norm_vr = cache_friction_link_norm_vr_;

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

template<typename T>
void simulator<T>::constraint_link(vec3<T>& result, solid<T>* s, const vec3<T>& solid_pos, const vec3<T>& solid_vel) {
	auto& tx = cache_constraint_link_tx_;
	auto& tv = cache_constraint_link_tv_;
	result.reset();

	for (auto* c : s->constraints_) {
		if (!c->is_active()) continue;
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

template<typename T>
void simulator<T>::update_acceleration(vec3<T>& result, solid<T>* s, const vec3<T>& x, const vec3<T>& v, T fdt) {
	auto& friction_force = cache_update_acceleration_friction_force_;
	auto& constraint_force = cache_update_acceleration_constraint_force_;
	auto& fluid_force = cache_update_acceleration_fluid_force_;
	T zero_val{};

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

template<typename T>
void simulator<T>::integration_step(solid<T>* s, const vec3<T>& x, const vec3<T>& v, const vec3<T>& dx, const vec3<T>& dv, T fdt, vec3<T>& result_x, vec3<T>& result_v) {
	auto& tx = cache_integration_step_tx_;
	auto& tv = cache_integration_step_tv_;
	mul(tx, dx, fdt);
	add(tx, x);
	mul(tv, dv, fdt);
	add(tv, v);
	result_x.set(tv);
	update_acceleration(result_v, s, tx, tv, fdt);
}

} // namespace hop
