#pragma once

#include <hop/math/bounding.h>
#include <hop/math/convex_solid.h>
#include <hop/math/support.h>
#include <hop/traceable.h>
#include <cassert>
#include <memory>
#include <utility>
#include <variant>

namespace hop {

template <typename T> class solid;
template <typename T> class simulator;

// Values are the shape's variant index (see shape::payload_t), so get_type() is a
// plain load of the variant's discriminant.
enum class shape_type {
	box = 0,
	sphere = 1,
	capsule = 2,
	convex_solid = 3,
	traceable = 4,
};

template <typename T> class shape : public std::enable_shared_from_this<shape<T>> {
public:
	using ptr = std::shared_ptr<shape<T>>;
	using tr = scalar_traits<T>;

	shape() : payload_(std::in_place_index<box_i>) {}
	explicit shape(const aa_box<T> & box) : payload_(std::in_place_index<box_i>, box) {}
	explicit shape(const sphere<T> & s) : payload_(std::in_place_index<sphere_i>, s) {}
	explicit shape(const capsule<T> & c) : payload_(std::in_place_index<capsule_i>, c) {}
	explicit shape(const convex_solid<T> & cs)
	    : payload_(std::in_place_index<convex_solid_i>, std::make_unique<convex_solid<T>>(cs)) {}
	// Non-owning: the caller guarantees the traceable outlives this shape. Right
	// for a traceable that is a member of something longer-lived.
	explicit shape(traceable<T> * t) : payload_(std::in_place_index<traceable_i>, traceable_ref { t, nullptr }) {}
	// Owning: the shape keeps the traceable alive for exactly as long as it points
	// at it. Prefer this whenever the traceable is built for one shape — the raw
	// form makes it easy to free the geometry out from under a live shape.
	explicit shape(std::unique_ptr<traceable<T>> t)
	    : payload_(std::in_place_index<traceable_i>, traceable_ref { t.get(), std::move(t) }) {}

	// The payload owns heap geometry via unique_ptr — non-copyable, move-only.
	shape(const shape &) = delete;
	shape & operator=(const shape &) = delete;
	shape(shape &&) = default;
	shape & operator=(shape &&) = default;

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

	// Local rotation: fixed orientation of this shape within its owning solid's
	// frame (the collision-shape-within-body transform). Default identity. Static
	// pose only — composed with the solid's orientation by the narrowphase; no
	// angular dynamics. get_bound()/support() stay intrinsic; callers apply this.
	void set_local_rotation(const mat3<T> & r) {
		local_rotation_ = r;
		if (solid_)
			solid_->update_local_bound();
	}
	const mat3<T> & get_local_rotation() const { return local_rotation_; }

	// Assigning a new alternative destroys the outgoing one (and anything it owns).
	void set_box(const aa_box<T> & box) {
		payload_ = box; // in-place if already a box, so set_box(get_box()) is safe
		changed();
	}
	const aa_box<T> & get_box() const { return alt<box_i>(); }

	void set_sphere(const sphere<T> & s) {
		payload_ = s;
		changed();
	}
	const sphere<T> & get_sphere() const { return alt<sphere_i>(); }

	void set_capsule(const capsule<T> & c) {
		payload_ = c;
		changed();
	}
	const capsule<T> & get_capsule() const { return alt<capsule_i>(); }

	void set_convex_solid(const convex_solid<T> & cs) {
		if (auto * held = std::get_if<convex_solid_i>(&payload_))
			**held = cs; // reuse the allocation
		else
			payload_.template emplace<convex_solid_i>(std::make_unique<convex_solid<T>>(cs));
		changed();
	}
	const convex_solid<T> & get_convex_solid() const { return *alt<convex_solid_i>(); }

	void set_traceable(traceable<T> * t) {
		payload_.template emplace<traceable_i>(traceable_ref { t, nullptr });
		changed();
	}
	void set_traceable(std::unique_ptr<traceable<T>> t) {
		traceable<T> * raw = t.get();
		payload_.template emplace<traceable_i>(traceable_ref { raw, std::move(t) });
		changed();
	}
	traceable<T> * get_traceable() const { return alt<traceable_i>().ptr; }

	// shape_type values are the variant indices; the static_assert below pins the order.
	shape_type get_type() const { return static_cast<shape_type>(payload_.index()); }

	void get_bound(aa_box<T> & box) const {
		switch (get_type()) {
		case shape_type::box:
			box.set(get_box());
			break;
		case shape_type::sphere:
			find_bounding_box(box, get_sphere());
			break;
		case shape_type::capsule:
			find_bounding_box(box, get_capsule());
			break;
		case shape_type::convex_solid: {
			const convex_solid<T> & cs = get_convex_solid();
			ensure_vertices(cs);
			box.reset();
			if (cs.vertices.empty())
				break;
			box.mins = cs.vertices[0];
			box.maxs = cs.vertices[0];
			for (size_t i = 1; i < cs.vertices.size(); ++i)
				box.merge(cs.vertices[i]);
			break;
		}
		case shape_type::traceable:
			get_traceable()->get_bound(box);
			break;
		}
	}

private:
	// A traceable is either borrowed (owned == nullptr) or owned; ptr is always the
	// one to read, so get_traceable() stays a plain load either way.
	struct traceable_ref {
		traceable<T> * ptr;
		std::unique_ptr<traceable<T>> owned;
	};

	// One slot for every kind of geometry; the variant's index is the type tag.
	// convex_solid stays behind a unique_ptr so its vectors don't inflate every
	// shape — the largest inline alternative is capsule.
	using payload_t = std::variant<aa_box<T>, sphere<T>, capsule<T>, std::unique_ptr<convex_solid<T>>, traceable_ref>;
	static constexpr std::size_t box_i = 0, sphere_i = 1, capsule_i = 2, convex_solid_i = 3, traceable_i = 4;

	// Unchecked read of alternative I. Asking for the wrong one is a caller bug: it
	// asserts in debug builds and is assumed impossible otherwise, so a getter reached
	// through a switch on get_type() compiles to a plain load, same as a union member.
	// (A bare *std::get_if keeps an index compare + trap on every call.)
	template <std::size_t I> const std::variant_alternative_t<I, payload_t> & alt() const {
		assert(payload_.index() == I && "shape getter does not match get_type()");
#if defined(_MSC_VER) && !defined(__clang__)
		if (payload_.index() != I) __assume(0);
#else
		if (payload_.index() != I) __builtin_unreachable();
#endif
		return *std::get_if<I>(&payload_);
	}

	void changed() {
		if (solid_)
			solid_->update_local_bound();
	}

	vec3<T> local_position_;
	mat3<T> local_rotation_; // identity by default
	solid<T> * solid_ = nullptr;

	payload_t payload_;

	static_assert(std::size_t(shape_type::box) == box_i && std::size_t(shape_type::sphere) == sphere_i &&
	                  std::size_t(shape_type::capsule) == capsule_i && std::size_t(shape_type::convex_solid) == convex_solid_i &&
	                  std::size_t(shape_type::traceable) == traceable_i,
	              "get_type() returns the variant index as a shape_type");

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

// Support point in the OWNING SOLID's frame: the shape's own support carried through
// its local_rotation and local_position. The support() above is intrinsic by design
// (see the local_position / local_rotation setters), and every caller that forgot to
// compose them has been a bug — reach for this one unless you specifically want the
// raw geometry. Identity pose degenerates to the intrinsic call exactly.
template <typename T> inline void support_in_solid(vec3<T> & result, const shape<T> & sh, const vec3<T> & d) {
	const mat3<T> identity;
	const mat3<T> & R = sh.get_local_rotation();
	if (R != identity) {
		mat3<T> rt;
		transpose(rt, R);
		vec3<T> local_dir;
		mul(local_dir, rt, d);
		vec3<T> local_sup;
		support(local_sup, sh, local_dir);
		mul(result, R, local_sup);
	} else {
		support(result, sh, d);
	}
	add(result, sh.get_local_position());
}

} // namespace hop
