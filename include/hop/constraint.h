#pragma once

#include <hop/math/support.h>
#include <memory>

namespace hop {

template <typename T> class solid;
template <typename T> class simulator;

template <typename T> class constraint {
public:
	using ptr = std::shared_ptr<constraint<T>>;
	using tr = scalar_traits<T>;

	// Behavior of the distance term.
	//   spring: bilateral. Force = k * (|d| - rest) along d. Pulls when stretched, pushes when compressed.
	//   rope:   unilateral. Force kicks in only when |d| > rest (max-length leash).
	enum class type { spring, rope };

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
	void set_damping_constant(T c) { damping_constant_ = c; }
	T get_damping_constant() const { return damping_constant_; }

	bool is_active() const { return simulator_ != nullptr; }

	// True if the constraint is producing force above `tolerance` — used by the sleep
	// heuristic so both endpoints don't freeze at non-equilibrium displacement.
	bool is_loaded(T tolerance) const {
		if (!start_solid_)
			return false;
		vec3<T> a_world;
		vec3<T> b_world;
		add(a_world, start_solid_->position_, local_anchor_a_);
		if (end_solid_) {
			add(b_world, end_solid_->position_, local_anchor_b_);
		} else {
			b_world = end_point_;
		}
		T d2 = length_squared(a_world, b_world);
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

	simulator<T> * simulator_ = nullptr;

	friend class solid<T>;
	friend class simulator<T>;
};

} // namespace hop
