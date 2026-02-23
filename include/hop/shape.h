#pragma once

#include <memory>
#include <hop/math/math_ops.h>
#include <hop/math/convex_solid.h>
#include <hop/math/bounding.h>
#include <hop/traceable.h>

namespace hop {

template<typename T> class solid;
template<typename T> class simulator;

enum class shape_type {
	aa_box      = 1 << 0,
	sphere      = 1 << 1,
	capsule     = 1 << 2,
	convex_solid = 1 << 3,
	traceable   = 1 << 4,
};

template<typename T>
class shape : public std::enable_shared_from_this<shape<T>> {
public:
	using ptr = std::shared_ptr<shape<T>>;
	using tr = scalar_traits<T>;

	shape() = default;
	explicit shape(const aa_box<T>& box) : type_(shape_type::aa_box), aa_box_(box) {}
	explicit shape(const hop::sphere<T>& s) : type_(shape_type::sphere), sphere_(s) {}
	explicit shape(const hop::capsule<T>& c) : type_(shape_type::capsule), capsule_(c) {}
	explicit shape(const hop::convex_solid<T>& cs) : type_(shape_type::convex_solid), convex_solid_(cs) {}
	explicit shape(hop::traceable<T>* t) : type_(shape_type::traceable), traceable_(t) {}

	void reset_shape() {
		if (solid_) {
			solid_->remove_shape(this->shared_from_this());
			solid_ = nullptr;
		}
		aa_box_ = {};
		type_ = shape_type::aa_box;
		sphere_ = {};
		capsule_ = {};
	}

	void set_aa_box(const aa_box<T>& box) {
		type_ = shape_type::aa_box;
		aa_box_ = box;
		if (solid_) solid_->update_local_bound();
	}
	const aa_box<T>& get_aa_box() const { return aa_box_; }

	void set_sphere(const hop::sphere<T>& s) {
		type_ = shape_type::sphere;
		sphere_ = s;
		if (solid_) solid_->update_local_bound();
	}
	const hop::sphere<T>& get_sphere() const { return sphere_; }

	void set_capsule(const hop::capsule<T>& c) {
		type_ = shape_type::capsule;
		capsule_ = c;
		if (solid_) solid_->update_local_bound();
	}
	const hop::capsule<T>& get_capsule() const { return capsule_; }

	void set_convex_solid(const hop::convex_solid<T>& cs) {
		type_ = shape_type::convex_solid;
		convex_solid_ = cs;
		if (solid_) solid_->update_local_bound();
	}
	const hop::convex_solid<T>& get_convex_solid() const { return convex_solid_; }

	void set_traceable(hop::traceable<T>* t) {
		type_ = shape_type::traceable;
		traceable_ = t;
		if (solid_) solid_->update_local_bound();
	}
	hop::traceable<T>* get_traceable() const { return traceable_; }

	shape_type get_type() const { return type_; }

	void get_bound(aa_box<T>& box) const {
		switch (type_) {
			case shape_type::aa_box:
				box.set(aa_box_);
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
				auto& planes = convex_solid_.planes;
				int sz = static_cast<int>(planes.size());
				bool first_vertex = true;
				for (int i = 0; i < sz - 2; ++i) {
					for (int j = i + 1; j < sz - 1; ++j) {
						for (int k = j + 1; k < sz; ++k) {
							vec3<T> r;
							if (get_intersection_of_three_planes(r, planes[i], planes[j], planes[k], epsilon)) {
								bool legal = true;
								for (int l = 0; l < sz; ++l) {
									if (l != i && l != j && l != k) {
										if ((dot(planes[l].normal, r) - planes[l].distance) > epsilon) {
											legal = false;
											break;
										}
									}
								}
								if (legal) {
									if (first_vertex) {
										box.mins = r;
										box.maxs = r;
										first_vertex = false;
									} else {
										box.merge(r);
									}
								}
							}
						}
					}
				}
				break;
			}
			case shape_type::traceable:
				traceable_->get_bound(box);
				break;
		}
	}

private:
	shape_type type_ = shape_type::aa_box;
	aa_box<T> aa_box_;
	hop::sphere<T> sphere_;
	hop::capsule<T> capsule_;
	hop::convex_solid<T> convex_solid_;
	hop::traceable<T>* traceable_ = nullptr;
	solid<T>* solid_ = nullptr;

	friend class solid<T>;
	friend class simulator<T>;
};

} // namespace hop
