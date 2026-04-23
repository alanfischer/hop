#pragma once

#include <hop/math/bounding.h>
#include <hop/math/convex_solid.h>
#include <hop/math/math_ops.h>
#include <hop/traceable.h>
#include <memory>

namespace hop {

template <typename T> class solid;
template <typename T> class simulator;

enum class shape_type {
	box = 1 << 0,
	sphere = 1 << 1,
	capsule = 1 << 2,
	convex_solid = 1 << 3,
	traceable = 1 << 4,
};

template <typename T> class shape : public std::enable_shared_from_this<shape<T>> {
public:
	using ptr = std::shared_ptr<shape<T>>;
	using tr = scalar_traits<T>;

	shape() = default;
	explicit shape(const aa_box<T> & box) : type_(shape_type::box), box_(box) {}
	explicit shape(const hop::sphere<T> & s) : type_(shape_type::sphere), sphere_(s) {}
	explicit shape(const hop::capsule<T> & c) : type_(shape_type::capsule), capsule_(c) {}
	explicit shape(const hop::convex_solid<T> & cs) : type_(shape_type::convex_solid), convex_solid_(cs) {}
	explicit shape(hop::traceable<T> * t) : type_(shape_type::traceable), traceable_(t) {}

	void reset_shape() {
		if (solid_) {
			solid_->remove_shape(this->shared_from_this());
			solid_ = nullptr;
		}
		box_ = {};
		type_ = shape_type::box;
		sphere_ = {};
		capsule_ = {};
		local_position_.reset();
	}

	// Local position: offset of this shape within its owning solid's frame.
	// Default is zero — shape sits at the solid's origin.
	// Note: get_bound() and support() are intrinsic (shape's own frame);
	// callers apply local_position themselves when composing into the solid.
	void set_local_position(const vec3<T> & p) {
		local_position_ = p;
		if (solid_)
			solid_->update_local_bound();
	}
	const vec3<T> & get_local_position() const { return local_position_; }

	void set_box(const aa_box<T> & box) {
		type_ = shape_type::box;
		box_ = box;
		if (solid_)
			solid_->update_local_bound();
	}
	const aa_box<T> & get_box() const { return box_; }

	void set_sphere(const hop::sphere<T> & s) {
		type_ = shape_type::sphere;
		sphere_ = s;
		if (solid_)
			solid_->update_local_bound();
	}
	const hop::sphere<T> & get_sphere() const { return sphere_; }

	void set_capsule(const hop::capsule<T> & c) {
		type_ = shape_type::capsule;
		capsule_ = c;
		if (solid_)
			solid_->update_local_bound();
	}
	const hop::capsule<T> & get_capsule() const { return capsule_; }

	void set_convex_solid(const hop::convex_solid<T> & cs) {
		type_ = shape_type::convex_solid;
		convex_solid_ = cs;
		if (solid_)
			solid_->update_local_bound();
	}
	const hop::convex_solid<T> & get_convex_solid() const { return convex_solid_; }

	void set_traceable(hop::traceable<T> * t) {
		type_ = shape_type::traceable;
		traceable_ = t;
		if (solid_)
			solid_->update_local_bound();
	}
	hop::traceable<T> * get_traceable() const { return traceable_; }

	shape_type get_type() const { return type_; }

	void get_bound(aa_box<T> & box) const {
		switch (type_) {
		case shape_type::box:
			box.set(box_);
			break;
		case shape_type::sphere:
			find_bounding_box(box, sphere_);
			break;
		case shape_type::capsule:
			find_bounding_box(box, capsule_);
			break;
		case shape_type::convex_solid: {
			T epsilon;
			if constexpr (std::is_same_v<T, fixed16>)
				epsilon = fixed16::from_raw(1 << 4);
			else
				epsilon = T(0.0001);

			box.reset();
			bool first_vertex = true;
			auto consider = [&](const vec3<T> & r) {
				if (first_vertex) {
					box.mins = r;
					box.maxs = r;
					first_vertex = false;
				} else {
					box.merge(r);
				}
			};
			if (!convex_solid_.vertices.empty()) {
				for (auto & v : convex_solid_.vertices)
					consider(v);
			} else {
				for_each_convex_solid_vertex(convex_solid_, epsilon, consider);
			}
			break;
		}
		case shape_type::traceable:
			traceable_->get_bound(box);
			break;
		}
	}

private:
	shape_type type_ = shape_type::box;
	aa_box<T> box_;
	hop::sphere<T> sphere_;
	hop::capsule<T> capsule_;
	hop::convex_solid<T> convex_solid_;
	hop::traceable<T> * traceable_ = nullptr;
	vec3<T> local_position_;
	solid<T> * solid_ = nullptr;

	friend class solid<T>;
	friend class simulator<T>;
};

template <typename T> inline void support(vec3<T> & result, const shape<T> & sh, const vec3<T> & d) {
	switch (sh.get_type()) {
	case shape_type::box:
		support(result, sh.get_box(), d);
		break;
	case shape_type::sphere:
		support(result, sh.get_sphere(), d);
		break;
	case shape_type::capsule:
		support(result, sh.get_capsule(), d);
		break;
	case shape_type::convex_solid:
		support(result, sh.get_convex_solid(), d);
		break;
	default:
		result.reset();
		break;
	}
}

} // namespace hop
