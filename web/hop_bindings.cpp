// hop_bindings.cpp — Emscripten/embind wrapper exposing hop physics to JavaScript
// Usage:
//   const sim = new Module.HopSimulator();
//   sim.setGravity(0, 0, -9.81);
//   const solid = sim.addSolid();
//   solid.setMass(1);
//   solid.addShape(Module.HopShape.box(0.5, 0.5, 0.5));
//   solid.setPosition(1, 0, 4);
//   sim.update(0.016);
//   const pos = solid.getPosition(); // {x, y, z}

#include <emscripten/bind.h>
#include <emscripten/val.h>
#include <hop/hop.h>
#include <memory>

static emscripten::val vec3_to_val(const hop::vec3<float>& v) {
	auto obj = emscripten::val::object();
	obj.set("x", v.x);
	obj.set("y", v.y);
	obj.set("z", v.z);
	return obj;
}

class HopShape {
	std::shared_ptr<hop::shape<float>> s_;

public:
	explicit HopShape(std::shared_ptr<hop::shape<float>> s) : s_(std::move(s)) {}

	const std::shared_ptr<hop::shape<float>>& ptr() const { return s_; }

	static HopShape box(float hx, float hy, float hz) {
		return HopShape(std::make_shared<hop::shape<float>>(
		    hop::aa_box<float>(hop::vec3<float>(-hx, -hy, -hz), hop::vec3<float>(hx, hy, hz))));
	}

	static HopShape sphere(float radius) {
		return HopShape(std::make_shared<hop::shape<float>>(hop::sphere<float>(radius)));
	}

	static HopShape capsule(float radius, float dx, float dy, float dz) {
		return HopShape(std::make_shared<hop::shape<float>>(
		    hop::capsule<float>(hop::vec3<float>(), hop::vec3<float>(dx, dy, dz), radius)));
	}

	// Offset within the owning solid's frame. Default is (0,0,0).
	void setLocalPosition(float x, float y, float z) { s_->set_local_position(hop::vec3<float>(x, y, z)); }
	emscripten::val getLocalPosition() const {
		auto obj = emscripten::val::object();
		const auto& lp = s_->get_local_position();
		obj.set("x", lp.x); obj.set("y", lp.y); obj.set("z", lp.z);
		return obj;
	}
};

class HopSolid {
	std::shared_ptr<hop::solid<float>> s_;

public:
	explicit HopSolid(std::shared_ptr<hop::solid<float>> s) : s_(std::move(s)) {}

	const std::shared_ptr<hop::solid<float>>& ptr() const { return s_; }

	void addShape(const HopShape& shape) { s_->add_shape(shape.ptr()); }

	void setMass(float mass) { s_->set_mass(mass); }
	void setInfiniteMass() { s_->set_infinite_mass(); }

	void setPosition(float x, float y, float z) { s_->set_position(hop::vec3<float>(x, y, z)); }
	void setVelocity(float x, float y, float z) { s_->set_velocity(hop::vec3<float>(x, y, z)); }

	emscripten::val getPosition() { return vec3_to_val(s_->get_position()); }
	emscripten::val getVelocity() { return vec3_to_val(s_->get_velocity()); }

	// Rotation (opt-in). Orientation is exchanged as a quaternion {x,y,z,w}.
	// A solid stays non-rotating until given a finite inertia via setInertia.
	void setOrientation(float x, float y, float z, float w) {
		s_->set_orientation_from_quat(hop::quat<float>(x, y, z, w));
	}
	emscripten::val getOrientation() {
		const auto& q = s_->get_orientation_quat();
		auto obj = emscripten::val::object();
		obj.set("x", q.x); obj.set("y", q.y); obj.set("z", q.z); obj.set("w", q.w);
		return obj;
	}
	void setAngularVelocity(float x, float y, float z) { s_->set_angular_velocity(hop::vec3<float>(x, y, z)); }
	emscripten::val getAngularVelocity() { return vec3_to_val(s_->get_angular_velocity()); }
	// Principal-axis diagonal inertia (Ix, Iy, Iz). Non-zero opts the body into
	// dynamic rotation; leave it unset (zero) and the body never spins.
	void setInertia(float x, float y, float z) { s_->set_inertia(hop::vec3<float>(x, y, z)); }
	void addTorque(float x, float y, float z) { s_->add_torque(hop::vec3<float>(x, y, z)); }

	void setCoefficientOfRestitution(float cor) { s_->set_coefficient_of_restitution(cor); }
	// Combine mode follows hop::restitution_combine: 0=average, 1=minimum, 2=multiply, 3=maximum.
	void setRestitutionCombine(int mode) { s_->set_restitution_combine(static_cast<hop::restitution_combine>(mode)); }
	void setCoefficientOfGravity(float cog) { s_->set_coefficient_of_gravity(cog); }
	void setFriction(float static_f, float dynamic_f) {
		s_->set_coefficient_of_static_friction(static_f);
		s_->set_coefficient_of_dynamic_friction(dynamic_f);
	}

	void setCollisionCallback(emscripten::val callback) {
		s_->set_collision_callback([callback](const hop::collision<float>& c) {
			auto obj = emscripten::val::object();
			obj.set("impact", vec3_to_val(c.impact));
			obj.set("point", vec3_to_val(c.point));
			obj.set("normal", vec3_to_val(c.normal));
			obj.set("velocity", vec3_to_val(c.velocity));
			callback(obj);
		});
	}
};

class HopConstraint {
	std::shared_ptr<hop::constraint<float>> c_;

public:
	explicit HopConstraint(std::shared_ptr<hop::constraint<float>> c) : c_(std::move(c)) {}

	void setType(int t) { c_->set_type(static_cast<hop::constraint<float>::type>(t)); }
	void setSpringConstant(float k) { c_->set_spring_constant(k); }
	void setDampingConstant(float d) { c_->set_damping_constant(d); }
	void setRestLength(float r) { c_->set_rest_length(r); }
	void setLocalAnchorA(float x, float y, float z) { c_->set_local_anchor_a(hop::vec3<float>(x, y, z)); }
	void setLocalAnchorB(float x, float y, float z) { c_->set_local_anchor_b(hop::vec3<float>(x, y, z)); }
	void setEndPoint(float x, float y, float z) { c_->set_end_point(hop::vec3<float>(x, y, z)); }

	emscripten::val getEndPoint() { return vec3_to_val(c_->get_end_point()); }
};

class HopSimulator {
	hop::simulator<float> sim_;
	std::vector<std::shared_ptr<hop::solid<float>>> solids_;

public:
	HopSimulator() = default;

	void setGravity(float x, float y, float z) { sim_.set_gravity(hop::vec3<float>(x, y, z)); }
	void update(float dt) { sim_.update(dt); }

	HopSolid addSolid() {
		auto s = std::make_shared<hop::solid<float>>();
		sim_.add_solid(s);
		solids_.push_back(s);
		return HopSolid(s);
	}

	HopConstraint addConstraint(const HopSolid& solid, float x, float y, float z) {
		auto c = std::make_shared<hop::constraint<float>>(solid.ptr(), hop::vec3<float>(x, y, z));
		sim_.add_constraint(c);
		return HopConstraint(c);
	}
};

EMSCRIPTEN_BINDINGS(hop) {
	emscripten::class_<HopShape>("HopShape")
	    .class_function("box", &HopShape::box)
	    .class_function("sphere", &HopShape::sphere)
	    .class_function("capsule", &HopShape::capsule)
	    .function("setLocalPosition", &HopShape::setLocalPosition)
	    .function("getLocalPosition", &HopShape::getLocalPosition);

	emscripten::class_<HopSolid>("HopSolid")
	    .function("addShape", &HopSolid::addShape)
	    .function("setMass", &HopSolid::setMass)
	    .function("setInfiniteMass", &HopSolid::setInfiniteMass)
	    .function("setPosition", &HopSolid::setPosition)
	    .function("setVelocity", &HopSolid::setVelocity)
	    .function("getPosition", &HopSolid::getPosition)
	    .function("getVelocity", &HopSolid::getVelocity)
	    .function("setOrientation", &HopSolid::setOrientation)
	    .function("getOrientation", &HopSolid::getOrientation)
	    .function("setAngularVelocity", &HopSolid::setAngularVelocity)
	    .function("getAngularVelocity", &HopSolid::getAngularVelocity)
	    .function("setInertia", &HopSolid::setInertia)
	    .function("addTorque", &HopSolid::addTorque)
	    .function("setCoefficientOfRestitution", &HopSolid::setCoefficientOfRestitution)
	    .function("setRestitutionCombine", &HopSolid::setRestitutionCombine)
	    .function("setCoefficientOfGravity", &HopSolid::setCoefficientOfGravity)
	    .function("setFriction", &HopSolid::setFriction)
	    .function("setCollisionCallback", &HopSolid::setCollisionCallback);

	emscripten::class_<HopConstraint>("HopConstraint")
	    .function("setType", &HopConstraint::setType)
	    .function("setSpringConstant", &HopConstraint::setSpringConstant)
	    .function("setDampingConstant", &HopConstraint::setDampingConstant)
	    .function("setRestLength", &HopConstraint::setRestLength)
	    .function("setLocalAnchorA", &HopConstraint::setLocalAnchorA)
	    .function("setLocalAnchorB", &HopConstraint::setLocalAnchorB)
	    .function("setEndPoint", &HopConstraint::setEndPoint)
	    .function("getEndPoint", &HopConstraint::getEndPoint);

	emscripten::class_<HopSimulator>("HopSimulator")
	    .constructor<>()
	    .function("setGravity", &HopSimulator::setGravity)
	    .function("update", &HopSimulator::update)
	    .function("addSolid", &HopSimulator::addSolid)
	    .function("addConstraint", &HopSimulator::addConstraint);
}
