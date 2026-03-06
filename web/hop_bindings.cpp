// hop_bindings.cpp — Emscripten/embind wrapper exposing hop physics to JavaScript
// Usage:
//   const sim = new Module.HopSimulator();
//   sim.setGravity(0, 0, -9.81);
//   const solid = sim.addSolid();
//   solid.setMass(1);
//   solid.addShape(Module.HopShape.box(0.5, 0.5, 0.5));
//   solid.setPosition(1, 0, 4);
//   sim.update(16);
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
};

class JsCollisionListener : public hop::collision_listener<float> {
	emscripten::val callback_;

public:
	explicit JsCollisionListener(emscripten::val cb) : callback_(std::move(cb)) {}

	void on_collision(const hop::collision<float>& c) override {
		auto obj = emscripten::val::object();
		obj.set("impact", vec3_to_val(c.impact));
		obj.set("point", vec3_to_val(c.point));
		obj.set("normal", vec3_to_val(c.normal));
		obj.set("velocity", vec3_to_val(c.velocity));
		callback_(obj);
	}
};

class HopSolid {
	std::shared_ptr<hop::solid<float>> s_;
	std::shared_ptr<JsCollisionListener> listener_;

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

	void setCoefficientOfRestitution(float cor) { s_->set_coefficient_of_restitution(cor); }
	void setCoefficientOfRestitutionOverride(bool override) { s_->set_coefficient_of_restitution_override(override); }
	void setCoefficientOfGravity(float cog) { s_->set_coefficient_of_gravity(cog); }
	void setFriction(float static_f, float dynamic_f) {
		s_->set_coefficient_of_static_friction(static_f);
		s_->set_coefficient_of_dynamic_friction(dynamic_f);
	}

	void setCollisionListener(emscripten::val callback) {
		listener_ = std::make_shared<JsCollisionListener>(std::move(callback));
		s_->set_collision_listener(listener_.get());
	}
};

class HopConstraint {
	std::shared_ptr<hop::constraint<float>> c_;

public:
	explicit HopConstraint(std::shared_ptr<hop::constraint<float>> c) : c_(std::move(c)) {}

	void setSpringConstant(float k) { c_->set_spring_constant(k); }
	void setDampingConstant(float d) { c_->set_damping_constant(d); }
	void setDistanceThreshold(float t) { c_->set_distance_threshold(t); }
	void setEndPoint(float x, float y, float z) { c_->set_end_point(hop::vec3<float>(x, y, z)); }

	emscripten::val getEndPoint() { return vec3_to_val(c_->get_end_point()); }
};

class HopSimulator {
	hop::simulator<float> sim_;
	std::vector<std::shared_ptr<hop::solid<float>>> solids_;

public:
	HopSimulator() = default;

	void setGravity(float x, float y, float z) { sim_.set_gravity(hop::vec3<float>(x, y, z)); }
	void update(int dt_ms) { sim_.update(dt_ms, hop::simulator<float>::scope_report_collisions); }

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
	    .class_function("capsule", &HopShape::capsule);

	emscripten::class_<HopSolid>("HopSolid")
	    .function("addShape", &HopSolid::addShape)
	    .function("setMass", &HopSolid::setMass)
	    .function("setInfiniteMass", &HopSolid::setInfiniteMass)
	    .function("setPosition", &HopSolid::setPosition)
	    .function("setVelocity", &HopSolid::setVelocity)
	    .function("getPosition", &HopSolid::getPosition)
	    .function("getVelocity", &HopSolid::getVelocity)
	    .function("setCoefficientOfRestitution", &HopSolid::setCoefficientOfRestitution)
	    .function("setCoefficientOfRestitutionOverride", &HopSolid::setCoefficientOfRestitutionOverride)
	    .function("setCoefficientOfGravity", &HopSolid::setCoefficientOfGravity)
	    .function("setFriction", &HopSolid::setFriction)
	    .function("setCollisionListener", &HopSolid::setCollisionListener);

	emscripten::class_<HopConstraint>("HopConstraint")
	    .function("setSpringConstant", &HopConstraint::setSpringConstant)
	    .function("setDampingConstant", &HopConstraint::setDampingConstant)
	    .function("setDistanceThreshold", &HopConstraint::setDistanceThreshold)
	    .function("setEndPoint", &HopConstraint::setEndPoint)
	    .function("getEndPoint", &HopConstraint::getEndPoint);

	emscripten::class_<HopSimulator>("HopSimulator")
	    .constructor<>()
	    .function("setGravity", &HopSimulator::setGravity)
	    .function("update", &HopSimulator::update)
	    .function("addSolid", &HopSimulator::addSolid)
	    .function("addConstraint", &HopSimulator::addConstraint);
}
