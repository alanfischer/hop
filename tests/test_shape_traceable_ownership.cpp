// A shape can hold its traceable either way: pointing at one somebody else owns,
// or owning it outright.
//
// The raw form is the older contract and stays supported, but it hands out a
// pointer with no lifetime attached — free the traceable while a shape still
// points at it and nothing notices until a trace dereferences freed memory. The
// owning form ties the two together so that cannot be expressed.

#include <cassert>
#include <cstdio>
#include <memory>
#include <hop/hop.h>

using namespace hop;
using T = float;

// Counts its own live instances, so a test can assert on destruction.
static int g_live = 0;

class counting_traceable : public traceable<T> {
public:
	counting_traceable() { ++g_live; }
	~counting_traceable() override { --g_live; }

	void get_bound(aa_box<T> & result) override {
		result.mins.set(-1, -1, -1);
		result.maxs.set(1, 1, 1);
	}
	void trace_segment(collision<T> &, const vec3<T> &, const mat3<T> &, const segment<T> &) override {}
	void trace_solid(collision<T> &, solid<T> *, const vec3<T> &, const mat3<T> &, const segment<T> &, T) override {}
};

static void test_non_owning_shape_leaves_lifetime_to_caller() {
	counting_traceable external;
	assert(g_live == 1);
	{
		shape<T> s(&external);
		assert(s.get_type() == shape_type::traceable);
		assert(s.get_traceable() == &external);
	}
	// The shape is gone; the traceable is not — the caller still owns it.
	assert(g_live == 1);
	printf("  non_owning_shape_leaves_lifetime_to_caller ok\n");
}

static void test_owning_shape_frees_with_the_shape() {
	{
		auto owned = std::make_unique<counting_traceable>();
		counting_traceable * raw = owned.get();
		assert(g_live == 1);
		shape<T> s(std::move(owned));
		assert(s.get_traceable() == raw);
		assert(g_live == 1);
	}
	assert(g_live == 0 && "an owned traceable must die with its shape");
	printf("  owning_shape_frees_with_the_shape ok\n");
}

// Moving must carry ownership across, not double-free or drop it.
static void test_move_carries_ownership() {
	{
		shape<T> a(std::make_unique<counting_traceable>());
		traceable<T> * raw = a.get_traceable();
		assert(g_live == 1);

		shape<T> b(std::move(a));
		assert(b.get_traceable() == raw);
		assert(g_live == 1 && "move must not free the traceable");

		shape<T> c(aa_box<T>(vec3<T>(-1, -1, -1), vec3<T>(1, 1, 1)));
		c = std::move(b);
		assert(c.get_traceable() == raw);
		assert(g_live == 1 && "move-assign must not free the traceable");
	}
	assert(g_live == 0);
	printf("  move_carries_ownership ok\n");
}

// Switching to any other variant releases the owned geometry — the same
// discipline convex_solid_ already follows.
static void test_switching_variant_releases_owned_traceable() {
	shape<T> s(std::make_unique<counting_traceable>());
	assert(g_live == 1);

	s.set_box(aa_box<T>(vec3<T>(-1, -1, -1), vec3<T>(1, 1, 1)));
	assert(g_live == 0 && "set_box must release the owned traceable");
	assert(s.get_type() == shape_type::box);

	s.set_traceable(std::make_unique<counting_traceable>());
	assert(g_live == 1);
	s.set_sphere(sphere<T>(vec3<T>(0, 0, 0), 1));
	assert(g_live == 0 && "set_sphere must release the owned traceable");

	s.set_traceable(std::make_unique<counting_traceable>());
	assert(g_live == 1);
	s.set_capsule(capsule<T>(vec3<T>(0, -1, 0), vec3<T>(0, 1, 0), 1));
	assert(g_live == 0 && "set_capsule must release the owned traceable");

	s.set_traceable(std::make_unique<counting_traceable>());
	assert(g_live == 1);
	convex_solid<T> cs;
	s.set_convex_solid(cs);
	assert(g_live == 0 && "set_convex_solid must release the owned traceable");
	printf("  switching_variant_releases_owned_traceable ok\n");
}

// Replacing an owned traceable with a raw one drops the old geometry; replacing
// it with another owned one does too.
static void test_replacing_a_traceable_releases_the_old_one() {
	counting_traceable external;   // 1 live
	{
		shape<T> s(std::make_unique<counting_traceable>());
		assert(g_live == 2);
		s.set_traceable(&external);
		assert(g_live == 1 && "swapping to a non-owned traceable must free the owned one");
		assert(s.get_traceable() == &external);

		s.set_traceable(std::make_unique<counting_traceable>());
		assert(g_live == 2);
		s.set_traceable(std::make_unique<counting_traceable>());
		assert(g_live == 2 && "replacing an owned traceable must free the previous one");
	}
	assert(g_live == 1);   // only `external` survives the shape
	printf("  replacing_a_traceable_releases_the_old_one ok\n");
}

// A solid holding the shape keeps the geometry alive, and drops it on removal.
static void test_solid_holding_the_shape_keeps_it_alive() {
	{
		auto s = std::make_shared<solid<T>>();
		s->add_shape(std::make_shared<shape<T>>(std::make_unique<counting_traceable>()));
		assert(g_live == 1);
		s->remove_all_shapes();
		assert(g_live == 0 && "removing the shape must release its traceable");
	}
	assert(g_live == 0);
	printf("  solid_holding_the_shape_keeps_it_alive ok\n");
}

int main() {
	printf("test_shape_traceable_ownership\n");
	test_non_owning_shape_leaves_lifetime_to_caller();
	test_owning_shape_frees_with_the_shape();
	test_move_carries_ownership();
	test_switching_variant_releases_owned_traceable();
	test_replacing_a_traceable_releases_the_old_one();
	test_solid_holding_the_shape_keeps_it_alive();
	assert(g_live == 0);
	printf("all shape traceable ownership tests passed\n");
	return 0;
}
