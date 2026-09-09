#pragma once
// A stripped BSP30 blob, built in memory, so the demo needs no map file.
//
// Trimmed from hop-godot's tests/bsp_fixtures.h — only what it takes to author one
// solid box brush. The layout is the real one from hop_bsp_format.h, so the traceable
// parses this exactly as it parses a shipped .bsp.

#include <cstdint>
#include <cstring>
#include <vector>

#include "hop_bsp_format.h"
#include "hop_bsp_traceable.h"

namespace bsp_blob {

using namespace hop_bsp;

struct Builder {
	std::vector<BSPPlane> planes;
	std::vector<BSPNode> nodes;
	std::vector<BSPClipNode> clipnodes;
	std::vector<BSPLeaf> leafs;
	std::vector<BSPModel> models;

	std::vector<uint8_t> build() const {
		std::vector<uint8_t> out(sizeof(BSPHeader), 0);
		BSPHeader hdr {};
		hdr.version = HLBSP_VERSION;
		auto put = [&](int idx, const void *data, size_t sz) {
			if (sz == 0) return;
			hdr.lumps[idx].fileofs = (int32_t)out.size();
			hdr.lumps[idx].filelen = (int32_t)sz;
			const uint8_t *p = (const uint8_t *)data;
			out.insert(out.end(), p, p + sz);
		};
		put(LUMP_PLANES, planes.data(), planes.size() * sizeof(BSPPlane));
		put(LUMP_NODES, nodes.data(), nodes.size() * sizeof(BSPNode));
		put(LUMP_CLIPNODES, clipnodes.data(), clipnodes.size() * sizeof(BSPClipNode));
		put(LUMP_LEAFS, leafs.data(), leafs.size() * sizeof(BSPLeaf));
		put(LUMP_MODELS, models.data(), models.size() * sizeof(BSPModel));
		memcpy(out.data(), &hdr, sizeof(BSPHeader));
		return out;
	}
};

// Six axial planes, one per face, normals pointing +axis as the format wants.
inline int add_box_planes(Builder &b, const double mins[3], const double maxs[3]) {
	const int first = (int)b.planes.size();
	for (int axis = 0; axis < 3; ++axis) {
		for (int hi = 0; hi < 2; ++hi) {
			BSPPlane p {};
			p.normal[axis] = 1.0f;
			p.dist = (float)(hi ? maxs[axis] : mins[axis]);
			p.type = axis;  // axial: hits plane_offset's fast path, as most map planes do
			b.planes.push_back(p);
		}
	}
	return first;
}

// A convex box as a chain of six splitting nodes: each plane sends "outside" to empty
// and "inside" on to the next, and the last one lands in solid.
inline int add_box_brush(Builder &b, const double mins[3], const double maxs[3], bool as_nodes) {
	const int p0 = add_box_planes(b, mins, maxs);
	const int base = as_nodes ? (int)b.nodes.size() : (int)b.clipnodes.size();
	if (as_nodes && b.leafs.empty()) {
		BSPLeaf solid {}; solid.contents = CONTENTS_SOLID;
		BSPLeaf empty {}; empty.contents = CONTENTS_EMPTY;
		b.leafs.push_back(solid);
		b.leafs.push_back(empty);
	}
	const int SOLID_CHILD = as_nodes ? -1 : CONTENTS_SOLID;
	const int EMPTY_CHILD = as_nodes ? -2 : CONTENTS_EMPTY;
	for (int i = 0; i < 6; ++i) {
		const bool upper = (i % 2) == 1;
		const int inward = (i == 5) ? SOLID_CHILD : (base + i + 1);
		const int child0 = upper ? EMPTY_CHILD : inward;
		const int child1 = upper ? inward : EMPTY_CHILD;
		if (as_nodes) {
			BSPNode n {};
			n.planenum = p0 + i;
			n.children[0] = (int16_t)child0;
			n.children[1] = (int16_t)child1;
			b.nodes.push_back(n);
		} else {
			BSPClipNode n {};
			n.planenum = p0 + i;
			n.children[0] = (int16_t)child0;
			n.children[1] = (int16_t)child1;
			b.clipnodes.push_back(n);
		}
	}
	return base;
}

// One model: hull 0 is the box, hulls 1..3 are the same box already expanded by each
// engine hull size, which is exactly what a map compiler bakes. Coordinates are
// GoldSrc units (Z-up), the space the file is authored in.
inline std::vector<uint8_t> make_box_map(const double mins[3], const double maxs[3]) {
	Builder b;
	BSPModel m {};
	for (int i = 0; i < 3; ++i) {
		m.mins[i] = (float)mins[i];
		m.maxs[i] = (float)maxs[i];
	}
	m.headnode[0] = 0;
	add_box_brush(b, mins, maxs, /*as_nodes=*/true);
	for (int h = 1; h < 4; ++h) {
		double emins[3], emaxs[3];
		for (int i = 0; i < 3; ++i) {
			emins[i] = mins[i] - hopbsp::HULL_SIZES[h].maxs[i];
			emaxs[i] = maxs[i] - hopbsp::HULL_SIZES[h].mins[i];
		}
		m.headnode[h] = (int32_t)b.clipnodes.size();
		add_box_brush(b, emins, emaxs, /*as_nodes=*/false);
	}
	b.models.push_back(m);
	return b.build();
}

} // namespace bsp_blob
