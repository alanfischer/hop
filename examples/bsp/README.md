# GoldSrc BSP collision, copied in for the demo

`hop_bsp_format.h` and `hop_bsp_traceable.h` are **verbatim copies** from
[hop-godot](https://github.com/alanfischer/hop-godot)'s `src/`. They are here so
`demo_gib_floor` can put a body on the same BSP hull the game uses, without hop
taking on a dependency or a GoldSrc-specific public API.

They depend on nothing but `<hop/hop.h>` and the standard library — there is no Godot
in them, despite the "godot" in some internal names (that is the *host* space the
traceable converts to and from, which for this demo is the same Y-up metres).

**They are copies. Do not fix bugs here** — fix them in hop-godot and re-copy, or the
two drift and the demo stops reproducing what the game does.

`bsp_blob.h` is trimmed from hop-godot's `tests/bsp_fixtures.h` and builds a stripped
BSP30 blob in memory, so the demo needs no map file on disk.
