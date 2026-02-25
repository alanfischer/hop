#pragma once

#include <hop/math/math_ops.h>
#include <memory>

namespace hop {

template <typename T> class solid;
template <typename T> class simulator;

template <typename T> class constraint {
public:
	using ptr = std::shared_ptr<constraint<T>>;
	using tr = scalar_traits<T>;

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
		spring_constant_ = tr::one();
		damping_constant_ = tr::one();
		distance_threshold_ = tr::one();
		simulator_ = nullptr;
	}

	void set_start_solid(std::shared_ptr<solid<T>> s) {
		if (end_solid_)
			end_solid_->activate();
		if (start_solid_) {
			start_solid_->activate();
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
		if (start_solid_)
			start_solid_->activate();
		if (end_solid_) {
			end_solid_->activate();
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
		if (start_solid_)
			start_solid_->activate();
		if (end_solid_) {
			end_solid_->activate();
			end_solid_->internal_remove_constraint(this);
			end_solid_ = nullptr;
		}
		end_point_ = p;
	}
	const vec3<T> & get_end_point() const { return end_point_; }

	void set_spring_constant(T c) { spring_constant_ = c; }
	T get_spring_constant() const { return spring_constant_; }
	void set_damping_constant(T c) { damping_constant_ = c; }
	T get_damping_constant() const { return damping_constant_; }
	void set_distance_threshold(T t) { distance_threshold_ = t; }
	T get_distance_threshold() const { return distance_threshold_; }

	bool is_active() const { return simulator_ != nullptr; }
	simulator<T> * get_simulator() const { return simulator_; }

private:
	void internal_set_simulator(simulator<T> * s) { simulator_ = s; }

	std::shared_ptr<solid<T>> start_solid_;
	std::shared_ptr<solid<T>> end_solid_;
	vec3<T> end_point_;

	T spring_constant_ {};
	T damping_constant_ {};
	T distance_threshold_ {};

	simulator<T> * simulator_ = nullptr;

	friend class solid<T>;
	friend class simulator<T>;
};

} // namespace hop
