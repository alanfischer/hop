#pragma once

#include <cstdint>
#include <cstring>

// The subset of the GoldSrc BSP30 on-disk format the hull trace reads.
//
// This is a *copy* of the collision-relevant structs from goldsrc-godot's
// src/parsers/bsp_parser.h — its layout twin. hop-godot must stay independently
// buildable and testable (its test suite builds with neither Godot nor
// goldsrc-godot), so it cannot include that header. Both sides conform to the
// published, frozen BSP30 spec, so drift is a spec violation rather than a
// private-contract break — but if one is ever touched, touch both.
//
// Deliberately absent: the render/texture lumps (faces, texinfo, edges,
// miptex, lighting). The trace descends planes and node/clipnode children; it
// never looks at a surface.

namespace hop_bsp {

inline constexpr int HLBSP_VERSION = 30;  // Half-Life / GoldSrc
inline constexpr int QBSP_VERSION = 29;   // Quake — same lump layout for everything below

enum BSPLumpType {
	LUMP_ENTITIES = 0,
	LUMP_PLANES,
	LUMP_TEXTURES,
	LUMP_VERTEXES,
	LUMP_VISIBILITY,
	LUMP_NODES,
	LUMP_TEXINFO,
	LUMP_FACES,
	LUMP_LIGHTING,
	LUMP_CLIPNODES,
	LUMP_LEAFS,
	LUMP_MARKSURFACES,
	LUMP_EDGES,
	LUMP_SURFEDGES,
	LUMP_MODELS,
	MAX_BSP_LUMPS
};

#pragma pack(push, 1)

struct BSPLump {
	int32_t fileofs;
	int32_t filelen;
};

struct BSPHeader {
	int32_t version;
	BSPLump lumps[MAX_BSP_LUMPS];
};

struct BSPPlane {
	float normal[3];
	float dist;
	int32_t type;
};

// hull 0 — the point hull. children >= 0 index nodes; children < 0 index
// leafs as -(leaf + 1), and the leaf carries the contents.
struct BSPNode {
	int32_t planenum;
	int16_t children[2];
	int16_t mins[3], maxs[3];
	uint16_t firstface;
	uint16_t numfaces;
};

// hulls 1..3 — the pre-expanded box hulls. children >= 0 index clipnodes;
// children < 0 ARE the contents value directly (no leaf indirection).
struct BSPClipNode {
	int32_t planenum;
	int16_t children[2];
};

struct BSPLeaf {
	int32_t contents;
	int32_t visofs;
	int16_t mins[3], maxs[3];
	uint16_t firstmarksurface;
	uint16_t nummarksurfaces;
	uint8_t ambient_level[4];
};

struct BSPModel {
	float mins[3], maxs[3];
	float origin[3];
	int32_t headnode[4];  // [0] = node tree (point hull), [1..3] = clipnode trees
	int32_t visleafs;
	int32_t firstface, numfaces;
};

#pragma pack(pop)

static_assert(sizeof(BSPLump) == 8, "BSP30 lump layout");
static_assert(sizeof(BSPHeader) == 4 + 15 * 8, "BSP30 header layout");
static_assert(sizeof(BSPPlane) == 20, "BSP30 plane layout");
static_assert(sizeof(BSPNode) == 24, "BSP30 node layout");
static_assert(sizeof(BSPClipNode) == 8, "BSP30 clipnode layout");
static_assert(sizeof(BSPLeaf) == 28, "BSP30 leaf layout");
static_assert(sizeof(BSPModel) == 64, "BSP30 model layout");

inline constexpr int CONTENTS_EMPTY = -1;
inline constexpr int CONTENTS_SOLID = -2;
inline constexpr int CONTENTS_WATER = -3;
inline constexpr int CONTENTS_SLIME = -4;
inline constexpr int CONTENTS_LAVA   = -5;
inline constexpr int CONTENTS_SKY    = -6;

// A non-owning view over a stripped BSP30 blob (see goldsrc-godot's
// GoldSrcBSP::get_bsp_blob — a real BSP file header with only the lumps a
// consumer needs filled in, everything else zeroed). Points straight into the
// caller's bytes; the caller owns them and must outlive the view.
struct BlobView {
	const BSPPlane *planes = nullptr;      int plane_count = 0;
	const BSPNode *nodes = nullptr;        int node_count = 0;
	const BSPClipNode *clipnodes = nullptr; int clipnode_count = 0;
	const BSPLeaf *leafs = nullptr;        int leaf_count = 0;
	const BSPModel *models = nullptr;      int model_count = 0;

	bool valid() const { return plane_count > 0 && model_count > 0; }
};

// Parse a stripped BSP30 blob into lump pointers. Returns false (leaving the
// view empty) on a truncated header, an unrecognised version, or any lump whose
// directory entry runs past the end of the buffer.
inline bool parse_blob(const uint8_t *data, size_t size, BlobView &out) {
	out = BlobView {};
	if (data == nullptr || size < sizeof(BSPHeader)) return false;

	BSPHeader hdr;
	memcpy(&hdr, data, sizeof(BSPHeader));
	if (hdr.version != HLBSP_VERSION && hdr.version != QBSP_VERSION) return false;

	// Resolve one lump to a typed pointer + element count. A zero-length lump is
	// legitimately absent (the blob only carries what its producer needed).
	auto lump = [&](int idx, size_t elem_size, const void **ptr, int *count) -> bool {
		const BSPLump &l = hdr.lumps[idx];
		if (l.filelen == 0) { *ptr = nullptr; *count = 0; return true; }
		if (l.fileofs < 0 || l.filelen < 0) return false;
		if ((size_t)l.fileofs + (size_t)l.filelen > size) return false;
		if ((size_t)l.filelen % elem_size != 0) return false;
		*ptr = data + l.fileofs;
		*count = (int)((size_t)l.filelen / elem_size);
		return true;
	};

	const void *p = nullptr;
	if (!lump(LUMP_PLANES, sizeof(BSPPlane), &p, &out.plane_count)) return false;
	out.planes = (const BSPPlane *)p;
	if (!lump(LUMP_NODES, sizeof(BSPNode), &p, &out.node_count)) return false;
	out.nodes = (const BSPNode *)p;
	if (!lump(LUMP_CLIPNODES, sizeof(BSPClipNode), &p, &out.clipnode_count)) return false;
	out.clipnodes = (const BSPClipNode *)p;
	if (!lump(LUMP_LEAFS, sizeof(BSPLeaf), &p, &out.leaf_count)) return false;
	out.leafs = (const BSPLeaf *)p;
	if (!lump(LUMP_MODELS, sizeof(BSPModel), &p, &out.model_count)) return false;
	out.models = (const BSPModel *)p;

	return out.valid();
}

} // namespace hop_bsp
