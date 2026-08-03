# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

**ring_storage** — a flash parameter-storage library for embedded MCUs (STM32G4/G0/H7, AT32, GD32). All registered KVs are packed into a **single frame** and append-written into a rotating sector region; a monotonic version number distinguishes old/new frames. Designed for whole-snapshot parameter persistence (FOC/PID parameter tables, bootloader metadata) as a replacement for EasyFlash. Detailed design docs live in [README.md](README.md) and [AGENTS.md](AGENTS.md) — read those before editing.

## Where it lives (shared code)

This directory is part of the **shared `public_layer/m_middlewares`** static library in the `stm32_workspace_` workspace. Every firmware project (E1_Master_Power_Manage, E1_Hand_G474, stm32_g474_boot) compiles it via `add_subdirectory(../public_layer/m_middlewares m_middlewares)`; the middleware CMakeLists picks up the dir with `aux_source_directory(.../Third_Party/ring_storage _MID_ring)`. **Edits here affect all sibling projects.**

## Build

No standalone target and no test infrastructure exist for this module. Build it by building a consuming project:

```bash
cd stm32_workspace_/E1_Master_Power_Manage   # or E1_Hand_G474 / stm32_g474_boot
cmake --preset Debug
ninja -C build/Debug
```

Only `ring_storage.c` and `rs_crc32.c` compile into `libm_middlewares`. The platform port lives **outside this directory** — see below.

## Cross-file layout (the non-obvious part)

```
ring_storage.h                              ← public API: error enum, config, KV entry, context struct
ring_storage_port.h                         ← V2 callback API: ring_storage_port_t struct, injected via config.port
ring_storage.c                              ← core: frame serialize/parse, sector scan, round-robin GC, save/load
rs_crc32.{c,h}                              ← CRC32 (poly 0xEDB88320), 8-way unrolled table lookup, static 1KB table
public_layer/device_drivers/hal_flash/ring_storage_port.c  ← the STM32G4 port (NOT in this dir!)
```

The port implementation `ring_storage_port.c` lives in `device_drivers/hal_flash/` and is added to each parent build separately — it is **not** compiled into `m_middlewares` (that's intentional, see the note in `ring_storage_port.h`). It forwards to `hal_flash`, converting absolute addresses to hal_flash's relative offset. Consumers integrate it by `extern`-declaring the five functions and casting them into a `ring_storage_port_t` (see `stm32_g474_boot/service/boot/boot_flash.c`).

### Port layer: V1/V2 mismatch — careful when editing

- `ring_storage_port.h` (V2.0.0) declares **callback structs** returning `int`, documented as "0 成功，负值失败".
- `ring_storage_port.c` still implements **V1-style free functions** returning `ring_storage_error_t` (which uses **positive** error codes).
- `boot_flash.c` bridges the two with casts (`(ring_storage_port_read_t)ring_storage_port_read`).

`ring_storage.c` casts port returns to `ring_storage_error_t` and checks `!= RING_STORAGE_OK`, so positive errors are caught internally. But the V1 impl's positive error codes contradict the V2 contract's "negative = failure" — any *new* consumer that checks `< 0` would misread an error as success. Keep the cast bridge consistent if you touch either side.

## Core mechanism (flows that cross files)

- **Frame = atomic snapshot.** `[magic "RSTF"][version][frame_len][kv_count][header_crc32]` + TLV KV data + `[data_crc32][commit_magic "COMT"]`. `commit_magic` is written **last** and is the atomic commit point — a frame with bad/missing commit_magic is an interrupted write and is skipped. On STM32G4 double-word programming, `data_crc32`+`commit_magic` (8B) is one atomic write.
- **Scan** (`rs_scan_for_latest_frame`, called from `ring_storage_init`): per sector, `rs_probe_header()` validates magic + header CRC + `frame_len` bounds, then footer commit_magic; picks the highest version with `(int32_t)(hdr.version - best_version) > 0` (wrap-safe). No valid frame → `RING_STORAGE_ERROR_NO_VALID_FRAME`, active sector = first, erased if dirty.
- **Save** (`ring_storage_save`): holds the lock for the whole flow (`goto cleanup`). Triggers GC when `est_flash_size > sector_size - write_offset`, then **serializes KVs after GC** — GC stages chunks through the same `frame_buffer`, so ordering is deliberate.
- **Load** (`rs_load_frame`): locks only the 3 Flash I/O segments (header, footer, KV data); CRC computation and KV parsing run **outside the lock** so interrupts aren't masked long enough to drop CAN frames.
- **GC** (`rs_gc_collect`): round-robin target `(active_index + 1) % N`, erase target if dirty (wrap/reclaim), chunk-copy latest frame (chunks aligned to `write_gran`), then switch active. **Lazy erase**: the old sector is *not* erased immediately — its frames stay as history (up to N× sector capacity) until the ring wraps back to it. Power-loss safe: the target is erased before the copy, so the latest frame always has a complete copy elsewhere.
- **`ring_storage_load_version(ctx, version)`** scans all sectors for a specific version (`version == 0` = latest); a GC'd version returns `NO_VALID_FRAME`.

## Key constraints (enforced in code, easy to trip)

- **`value` pointers must be static/global** — `ring_storage_register()` stores the pointer; `save()` dereferences it later. Stack temporaries serialize undefined data.
- **Register all KVs before any save/load** — the table is fixed at runtime (`RING_STORAGE_MAX_KV = 32`, `RING_STORAGE_KEY_MAX = 31`); `register` caches `key_len` to avoid repeated `strlen`.
- **Init validation**: `area_size >= 2 * sector_size`, `area_size % sector_size == 0`, `start_addr % sector_size == 0`, valid `write_gran` (8/32/64/128/256), `frame_buffer_size >= 28` (frame overhead) and `>= write_gran/8` (GC chunk copy needs it).
- **`frame_buffer` is shared** between frame serialization and GC chunk staging — don't reorder save's serialize-after-GC sequence.
- **Sector-empty detection** (`rs_is_sector_empty`) samples head/mid/tail 16 bytes each, not just the head — catches residue data beyond offset 16.

## Conventions & gotchas

- **Error codes are positive** — a deliberate deviation from the workspace convention (`MODULE_OK = 0`, errors negative, per `m_middlewares/README.md` and the `e1-firmware-rules` skill). Don't "normalize" them without updating every caller.
- **Logging is compiled out by default**: `RING_STORAGE_LOG_ENABLE` in `ring_storage.c` (and `RING_STORAGE_PORT_LOG_ENABLE` in the port) is `0`, so even error logs are no-ops. Flip to `1` to debug.
- **Style** follows the `e1-firmware-rules` skill: WebKit braces (Allman for functions, K&R for control flow), 4-space indent, 100-col, snake_case + `module_` prefix, `_t`/`_cb_t` suffixes, Chinese Doxygen comments. Logging uses the middleware `log.h` with tag `"ring_storage"`.
- **Packed structs**: `rs_header_t`/`rs_footer_t` are `__attribute__((packed))` — the on-flash layout depends on it; don't reorder fields without updating `RS_FRAME_OVERHEAD`/scan logic.
- **Port `lock` should nest** (ref-counted, as `hal_flash_lock` is) and use `BASEPRI` masking rather than `PRIMASK`, so high-priority interrupts (e.g. FOC) keep running.

## Consumers

- `stm32_g474_boot/service/boot/boot_flash.{c,h}` — bootloader metadata (`boot_metadata_t`) persisted in a 16KB region via ring_storage; the canonical integration example (port bridging + config).
