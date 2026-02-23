#pragma once

#include <memory>

namespace hop {

struct fixed16;
template<typename T> struct vec3;
template<typename T> struct aa_box;
template<typename T> struct sphere;
template<typename T> struct capsule;
template<typename T> struct segment;
template<typename T> struct plane;
template<typename T> struct convex_solid;
template<typename T> struct collision;
template<typename T> class collision_listener;
template<typename T> class traceable;
template<typename T> class manager;
template<typename T> class shape;
template<typename T> class solid;
template<typename T> class constraint;
template<typename T> class simulator;

} // namespace hop
