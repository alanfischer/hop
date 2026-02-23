// hop_bindings.cpp — Emscripten/embind wrapper exposing hop physics to JavaScript
// Provides a scalar-argument facade so JS doesn't need to construct vec3 objects.

#include <emscripten/bind.h>
#include <hop/hop.h>
#include <memory>
#include <vector>

class HopSimulator {
	hop::simulator<float> sim_;
	std::vector<std::shared_ptr<hop::solid<float>>> solids_;

public:
	HopSimulator() = default;

	void set_gravity(float x, float y, float z) {
		sim_.set_gravity(hop::vec3<float>(x, y, z));
	}

	void update(int dt_ms) {
		sim_.update(dt_ms);
	}

	// Add a dynamic box with given half-extents. Returns solid index.
	int add_box(float mass, float hx, float hy, float hz) {
		auto s = std::make_shared<hop::solid<float>>();
		s->set_mass(mass);
		s->add_shape(std::make_shared<hop::shape<float>>(hop::aa_box<float>(
			hop::vec3<float>(-hx, -hy, -hz),
			hop::vec3<float>( hx,  hy,  hz)
		)));
		sim_.add_solid(s);
		int id = static_cast<int>(solids_.size());
		solids_.push_back(s);
		return id;
	}

	// Add a dynamic sphere. Returns solid index.
	int add_sphere(float mass, float radius) {
		auto s = std::make_shared<hop::solid<float>>();
		s->set_mass(mass);
		s->add_shape(std::make_shared<hop::shape<float>>(hop::sphere<float>(radius)));
		sim_.add_solid(s);
		int id = static_cast<int>(solids_.size());
		solids_.push_back(s);
		return id;
	}

	// Add a dynamic capsule (from origin to origin+direction, with radius). Returns solid index.
	int add_capsule(float mass, float radius, float dx, float dy, float dz) {
		auto s = std::make_shared<hop::solid<float>>();
		s->set_mass(mass);
		s->add_shape(std::make_shared<hop::shape<float>>(hop::capsule<float>(
			hop::vec3<float>(),
			hop::vec3<float>(dx, dy, dz),
			radius
		)));
		sim_.add_solid(s);
		int id = static_cast<int>(solids_.size());
		solids_.push_back(s);
		return id;
	}

	void set_position(int id, float x, float y, float z) {
		solids_[id]->set_position(hop::vec3<float>(x, y, z));
	}

	void set_velocity(int id, float x, float y, float z) {
		solids_[id]->set_velocity(hop::vec3<float>(x, y, z));
	}

	float get_x(int id) { return solids_[id]->get_position().x; }
	float get_y(int id) { return solids_[id]->get_position().y; }
	float get_z(int id) { return solids_[id]->get_position().z; }

	void set_coefficient_of_restitution(int id, float cor) {
		solids_[id]->set_coefficient_of_restitution(cor);
	}

	void set_coefficient_of_restitution_override(int id, bool override) {
		solids_[id]->set_coefficient_of_restitution_override(override);
	}

	void set_coefficient_of_gravity(int id, float cog) {
		solids_[id]->set_coefficient_of_gravity(cog);
	}

	void set_infinite_mass(int id) {
		solids_[id]->set_infinite_mass();
	}

	void set_friction(int id, float static_f, float dynamic_f) {
		solids_[id]->set_coefficient_of_static_friction(static_f);
		solids_[id]->set_coefficient_of_dynamic_friction(dynamic_f);
	}
};

EMSCRIPTEN_BINDINGS(hop) {
	emscripten::class_<HopSimulator>("HopSimulator")
		.constructor<>()
		.function("setGravity", &HopSimulator::set_gravity)
		.function("update", &HopSimulator::update)
		.function("addBox", &HopSimulator::add_box)
		.function("addSphere", &HopSimulator::add_sphere)
		.function("addCapsule", &HopSimulator::add_capsule)
		.function("setPosition", &HopSimulator::set_position)
		.function("setVelocity", &HopSimulator::set_velocity)
		.function("getX", &HopSimulator::get_x)
		.function("getY", &HopSimulator::get_y)
		.function("getZ", &HopSimulator::get_z)
		.function("setCoefficientOfRestitution", &HopSimulator::set_coefficient_of_restitution)
		.function("setCoefficientOfRestitutionOverride", &HopSimulator::set_coefficient_of_restitution_override)
		.function("setCoefficientOfGravity", &HopSimulator::set_coefficient_of_gravity)
		.function("setInfiniteMass", &HopSimulator::set_infinite_mass)
		.function("setFriction", &HopSimulator::set_friction);
}
