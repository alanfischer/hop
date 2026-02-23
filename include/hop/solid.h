#pragma once

#include <vector>
#include <memory>
#include <algorithm>
#include <hop/math/math_ops.h>
#include <hop/collision_listener.h>
#include <hop/shape.h>

namespace hop {

template<typename T> class constraint;
template<typename T> class simulator;
template<typename T> class manager;

template<typename T>
class solid : public std::enable_shared_from_this<solid<T>> {
public:
	using ptr = std::shared_ptr<solid<T>>;
	using tr = scalar_traits<T>;

	static T infinite_mass() { return -tr::one(); }

	solid() { reset(); }

	void destroy() {
		while (!constraints_.empty()) {
			auto* c = constraints_[0];
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
		internal_scope_ = 0;
		collision_scope_ = -1;
		collide_with_scope_ = -1;
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
		coefficient_of_effective_drag_ = T{};
		shape_types_ = 0;
		local_bound_.reset();
		world_bound_.reset();
		collision_listener_ = nullptr;
		user_data_ = nullptr;
		active_ = true;
		deactivate_count_ = 0;
		touched1_ = nullptr;
		touched1_normal_.reset();
		touched2_ = nullptr;
		touched2_normal_.reset();
		touching_ = nullptr;
		touching_normal_.reset();
		do_update_callback_ = false;
		simulator_ = nullptr;
	}

	// Scope
	void set_scope(int s) { scope_ = s; }
	int get_scope() const { return scope_; }
	void set_internal_scope(int s) { internal_scope_ = s; }
	int get_internal_scope() const { return internal_scope_; }
	void set_collision_scope(int s) { collision_scope_ = s; }
	int get_collision_scope() const { return collision_scope_; }
	void set_collide_with_scope(int s) { collide_with_scope_ = s; }
	int get_collide_with_scope() const { return collide_with_scope_; }

	// Mass
	void set_mass(T mass) {
		mass_ = mass;
		if (mass > T{}) {
			inv_mass_ = tr::one() / mass;
		} else {
			inv_mass_ = T{};
		}
	}
	T get_mass() const { return mass_; }
	void set_infinite_mass() { mass_ = infinite_mass(); inv_mass_ = T{}; }
	bool has_infinite_mass() const { return mass_ == infinite_mass(); }

	// Position / Velocity / Force
	void set_position(const vec3<T>& pos);
	const vec3<T>& get_position() const { return position_; }
	void set_velocity(const vec3<T>& vel) { velocity_.set(vel); activate(); }
	const vec3<T>& get_velocity() const { return velocity_; }
	void add_force(const vec3<T>& f) { add(force_, f); activate(); }
	const vec3<T>& get_force() const { return force_; }
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

	void remove_all_shapes() { shapes_.clear(); update_local_bound(); activate(); }
	shape<T>* get_shape(int i) const { return shapes_[i].get(); }
	int get_num_shapes() const { return static_cast<int>(shapes_.size()); }
	int get_shape_types() const { return shape_types_; }

	const aa_box<T>& get_local_bound() const { return local_bound_; }
	const aa_box<T>& get_world_bound() const { return world_bound_; }

	solid<T>* get_touching() const { return touching_; }
	const vec3<T>& get_touching_normal() const { return touching_normal_; }

	void set_collision_listener(collision_listener<T>* l) { collision_listener_ = l; }
	collision_listener<T>* get_collision_listener() const { return collision_listener_; }

	void set_user_data(void* d) { user_data_ = d; }
	void* get_user_data() const { return user_data_; }

	void activate() {
		if (deactivate_count_ > 0) deactivate_count_ = 0;
		if (!active_) {
			active_ = true;
			for (auto* c : constraints_) {
				if (c->start_solid_.get() != this && c->start_solid_) c->start_solid_->activate();
				else if (c->end_solid_.get() != this && c->end_solid_) c->end_solid_->activate();
			}
		}
	}

	void set_stay_active(bool a) { deactivate_count_ = a ? -1 : 0; activate(); }
	void deactivate() { active_ = false; deactivate_count_ = 0; }
	bool active() const { return active_ && simulator_ != nullptr; }

	void set_do_update_callback(bool c) { do_update_callback_ = c; }
	void set_manager(manager<T>* m) { manager_ = m; }
	simulator<T>* get_simulator() const { return simulator_; }

	void set_position_direct(const vec3<T>& pos) {
		position_.set(pos);
		add(world_bound_, local_bound_, position_);
	}

	void update_local_bound() {
		shape_types_ = 0;
		if (shapes_.empty()) {
			local_bound_.reset();
		} else {
			shape_types_ |= static_cast<int>(shapes_[0]->get_type());
			shapes_[0]->get_bound(local_bound_);
			aa_box<T> box;
			for (int i = 1; i < static_cast<int>(shapes_.size()); ++i) {
				shape_types_ |= static_cast<int>(shapes_[i]->get_type());
				shapes_[i]->get_bound(box);
				local_bound_.merge(box);
			}
		}
		add(world_bound_, local_bound_, position_);
	}

private:
	void internal_set_simulator(simulator<T>* s) { simulator_ = s; }
	void internal_add_constraint(constraint<T>* c) { constraints_.push_back(c); }
	void internal_remove_constraint(constraint<T>* c) {
		constraints_.erase(std::remove(constraints_.begin(), constraints_.end(), c), constraints_.end());
	}

	int scope_ = -1;
	int internal_scope_ = 0;
	int collision_scope_ = -1;
	int collide_with_scope_ = -1;
	T mass_{};
	T inv_mass_{};
	vec3<T> position_;
	vec3<T> velocity_;
	vec3<T> force_;
	T coefficient_of_gravity_{};
	T coefficient_of_restitution_{};
	bool coefficient_of_restitution_override_ = false;
	T coefficient_of_static_friction_{};
	T coefficient_of_dynamic_friction_{};
	T coefficient_of_effective_drag_{};

	std::vector<typename shape<T>::ptr> shapes_;
	int shape_types_ = 0;
	aa_box<T> local_bound_;
	aa_box<T> world_bound_;

	std::vector<constraint<T>*> constraints_;

	collision_listener<T>* collision_listener_ = nullptr;
	void* user_data_ = nullptr;

	bool active_ = true;
	int deactivate_count_ = 0;

	solid<T>* touched1_ = nullptr;
	vec3<T> touched1_normal_;
	solid<T>* touched2_ = nullptr;
	vec3<T> touched2_normal_;
	solid<T>* touching_ = nullptr;
	vec3<T> touching_normal_;
	int last_dt_ = 0;

	bool do_update_callback_ = false;
	manager<T>* manager_ = nullptr;

	simulator<T>* simulator_ = nullptr;

	friend class constraint<T>;
	friend class shape<T>;
	friend class simulator<T>;
};

// set_position needs simulator — defined after simulator is available, but
// we provide a basic version here that the simulator's updateSolid will override
template<typename T>
inline void solid<T>::set_position(const vec3<T>& pos) {
	set_position_direct(pos);
	activate();
}

} // namespace hop
