# Firmware / Software Compatibility

## Overview

On startup, `zynqMotorController` reads two global registers from the FPGA and
validates them against a compile-time compatibility table in `zynqMotorDriver.cpp`.
If the firmware does not match any entry the driver throws `std::runtime_error` and
the IOC fails to start, preventing silent misbehaviour with a mismatched bitfile.

---

## Register Layout

Both registers are at fixed global offsets regardless of the motor axis count:

| Offset | Name     | Width  | Description                                             |
|--------|----------|--------|---------------------------------------------------------|
| 0x00   | DEVICE   | 32-bit | `[31:16]` device type, `[15:0]` subdevice type          |
| 0x04   | VERSION  | 32-bit | `[31:24]` major, `[23:16]` minor, `[15:0]` build number |
| 0x08   | GIT_HASH | 32-bit | `[31]` dirty flag, `[27:0]` lower 7 hex digits of `git rev-parse --short HEAD` |

---

## Device and Subdevice Codes

Defined in `firmware/src/pkg/generic_pkg.sv`.

**Device codes:**

| Code   | Name            | Description               |
|--------|-----------------|---------------------------|
| 0x0006 | `MOTION_KRIA`   | Kria-based stepper controller |

**Subdevice codes:**

| Code   | Name                              | Description                             |
|--------|-----------------------------------|-----------------------------------------|
| 0x0000 | `SUBMOTION_KR260_DRV8434A_4CH`    | KR260, DRV8434A, 4-axis                 |
| 0x0001 | `SUBMOTION_KR260_DRV8825_4CH`     | KR260, DRV8825, 4-axis                  |
| 0x0002 | `SUBMOTION_KRIA_DRV8434A_8CH`     | Custom Kria carrier, DRV8434A, 8-axis   |
| 0x0003 | `SUBMOTION_KRIA_DRV8825_8CH`      | Custom Kria carrier, DRV8825, 8-axis    |

The DEVICE register word is: `(device_code << 16) | subdevice_code`.  
Example: `MOTION_KRIA` + `SUBMOTION_KR260_DRV8434A_4CH` → `0x00060000`.

---

## IOC Startup Log

On controller init the driver logs:

```
zynqMotorDriver: DEVICE=0x00060000  VERSION=1.0.0  GIT=ab3f7c2
zynqMotorDriver: DEVICE=0x00060000  VERSION=1.0.0  GIT=ab3f7c2-dirty
```

The `-dirty` suffix means the firmware was built from an uncommitted working tree.
The git hash is **informational only** — the compatibility check uses only `DEVICE` and `VERSION`.
See `firmware/doc/how-to-compile.md` ("Git Hash Register Build Setup") for how the hash is embedded at build time.

---

## Versioning Policy

| Field   | When to change                                                                 |
|---------|--------------------------------------------------------------------------------|
| `major` | Breaking change — register moved, bit field redefined, or semantics changed. Software **must** be updated before the new bitfile is deployed. |
| `minor` | Backward-compatible addition — new register or status bit that old software can safely ignore. Raise `maxMinor` in the table. |
| `build` | Internal build counter. Not part of the ABI contract; ignored by the check.    |

---

## Current Compatibility Table

Defined in `zynqMotorDriver.cpp` (`kFwCompatRules[]`).

| DEVICE word | Major | Min Minor | Max Minor | Description                        |
|-------------|-------|-----------|-----------|------------------------------------|
| 0x00060000  |   1   |     0     |    255    | MOTION_KRIA / KR260 DRV8434A 4CH   |

---

## How to Update the Table

### Firmware bumps `minor` (additive, software unchanged)

Raise `maxMinor` in the matching row:

```cpp
{ 0x00060000u,  1,  0,  2,  "MOTION_KRIA / KR260 DRV8434A 4CH" },
```

### Firmware bumps `major` (breaking change)

Add a new row for the new major, then update software as needed.  
Keep the old row during a transition period if older bitfiles are still in the field:

```cpp
{ 0x00060000u,  1,  0,  255,  "MOTION_KRIA / KR260 DRV8434A 4CH (ABI v1)" },
{ 0x00060000u,  2,  0,  255,  "MOTION_KRIA / KR260 DRV8434A 4CH (ABI v2)" },
```

Remove the old row once support for it is dropped.

### New subdevice introduced

Add a row with the new DEVICE word:

```cpp
{ 0x00060002u,  1,  0,  255,  "MOTION_KRIA / Kria custom DRV8434A 8CH" },
```

---

## Release Tagging Convention

Tag both the firmware and software repositories with the same label, e.g.:

```
mc-2026.04.02
```

Include in the release notes:
- Firmware commit hash and/or bitfile checksum
- Software commit hash
- Compatible firmware DEVICE word(s) and VERSION range(s)
