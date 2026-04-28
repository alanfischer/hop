#pragma once

#include <hop/math/bounding.h>
#include <hop/math/convex_solid.h>
#include <hop/math/support.h>
#include <hop/traceable.h>
#include <memory>
#include <utility>

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

	shape() : type_(shape_type::box), box_{} {}
	explicit shape(const aa_box<T> & box) : type_(shape_type::box), box_(box) {}
	explicit shape(const hop::sphere<T> & s) : type_(shape_type::sphere), sphere_(s) {}
	explicit shape(const hop::capsule<T> & c) : type_(shape_type::capsule), capsule_(c) {}
	explicit shape(const hop::convex_solid<T> & cs)
	    : type_(shape_type::convex_solid),
	      box_{},
	      convex_solid_(std::make_unique<hop::convex_solid<T>>(cs)) {}
	explicit shape(hop::traceable<T> * t) : type_(shape_type::traceable), traceable_(t) {}

	// shape owns convex_solid_ via unique_ptr — non-copyable, move-only.
	shape(const shape &) = delete;
	shape & operator=(const shape &) = delete;
	shape(shape &&) = default;
	shape & operator=(shape &&) = default;

	void reset_shape() {
		if (solid_) {
			solid_->remove_shape(this->shared_from_this());
			solid_ = nullptr;
		}
		type_ = shape_type::box;
		box_ = {};
		convex_solid_.reset();
		traceable_ = nullptr;
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
		convex_solid_.reset();
		if (solid_)
			solid_->update_local_bound();
	}
	const aa_box<T> & get_box() const { return box_; }

	void set_sphere(const hop::sphere<T> & s) {
		type_ = shape_type::sphere;
		sphere_ = s;
		convex_solid_.reset();
		if (solid_)
			solid_->update_local_bound();
	}
	const hop::sphere<T> & get_sphere() const { return sphere_; }

	void set_capsule(const hop::capsule<T> & c) {
		type_ = shape_type::capsule;
		capsule_ = c;
		convex_solid_.reset();
		if (solid_)
			solid_->update_local_bound();
	}
	const hop::capsule<T> & get_capsule() const { return capsule_; }

	void set_convex_solid(const hop::convex_solid<T> & cs) {
		type_ = shape_type::convex_solid;
		if (convex_solid_)
			*convex_solid_ = cs;
		else
			convex_solid_ = std::make_unique<hop::convex_solid<T>>(cs);
		if (solid_)
			solid_->update_local_bound();
	}
	const hop::convex_solid<T> & get_convex_solid() const { return *convex_solid_; }

	void set_traceable(hop::traceable<T> * t) {
		type_ = shape_type::traceable;
		traceable_ = t;
		convex_solid_.reset();
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
			ensure_vertices(*convex_solid_);
			box.reset();
			if (convex_solid_->vertices.empty())
				break;
			box.mins = convex_solid_->vertices[0];
			box.maxs = convex_solid_->vertices[0];
			for (size_t i = 1; i < convex_solid_->vertices.size(); ++i)
				box.merge(convex_solid_->vertices[i]);
			break;
		}
		case shape_type::traceable:
			traceable_->get_bound(box);
			break;
		}
	}

private:
	// Small per-shape metadata first, then variant payload. type_ tells the
	// rest of the engine which member of the union below is active.
	shape_type type_ = shape_type::box;
	vec3<T> local_position_;
	solid<T> * solid_ = nullptr;

	// Variant payload. The trivially-destructible variants (box / sphere /
	// capsule) plus the externally-owned traceable* share one anonymous-union
	// slot — type_ selects which member is live. convex_solid_ has heap-owned
	// vectors and can't safely live in a union, so it sits behind a unique_ptr
	// allocated lazily when type_ becomes shape_type::convex_solid (and freed
	// when transitioning back to a trivial variant). Saves ~70 B per shape vs.
	// the previous layout where every variant carried full inline storage.
	union {
		aa_box<T>           box_;
		hop::sphere<T>      sphere_;
		hop::capsule<T>     capsule_;
		hop::traceable<T> * traceable_;
	};
	std::unique_ptr<hop::convex_solid<T>> convex_solid_;

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
