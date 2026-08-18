Welcome to the Klipper project!

[![Klipper](docs/img/klipper-logo-small.png)](https://www.klipper3d.org/)

https://www.klipper3d.org/

The Klipper firmware controls 3d-Printers. It combines the power of a
general purpose computer with one or more micro-controllers. See the
[features document](https://www.klipper3d.org/Features.html) for more
information on why you should use the Klipper software.

Start by [installing Klipper software](https://www.klipper3d.org/Installation.html).

Klipper software is Free Software. See the [license](COPYING) or read
the [documentation](https://www.klipper3d.org/Overview.html). We
depend on the generous support from our
[sponsors](https://www.klipper3d.org/Sponsors.html).

---

## Axis Backlash Compensation

This fork adds **backlash compensation** for X, Y, and Z axes — a feature
that corrects for mechanical play in the drivetrain (gear teeth, lead screws,
belts) by applying a position offset when an axis changes direction.

### The Problem

Every mechanical drivetrain has some amount of **backlash** — the gap between
mating gear teeth or the looseness in a lead screw. When the motor reverses
direction, it moves but the toolhead doesn't until the slack is taken up.
This causes:

- **Inaccurate dimensions** on parts with direction changes
- **Ringing artifacts** at corners where axes reverse
- **Poor fit** on assemblies requiring tight tolerances

### How It Works

The compensation is inspired by **Pressure Advance**, which compensates for
filament compression delay by pushing filament earlier. Backlash compensation
applies the same principle to axis motion:

1. **Detect direction changes** at move boundaries by comparing consecutive
   move displacements
2. **Apply a position offset** of `±backlash/2` — the motor leads the tool
   by half the backlash distance
3. The mechanical play is absorbed during the direction change, so the tool
   starts moving in the new direction immediately

```
Without backlash compensation:
  Motor:  0 → 10 → [reverses, 0.2mm lost] → 9.8
  Tool:   0 → 10 → [stuck while motor reverses] → 9.8

With backlash compensation (backlash=0.2mm):
  Motor:  0 → 10.1 → [reverses, offset absorbs play] → 9.9
  Tool:   0 → 10 → [immediate reverse] → 9.9
```

### Configuration

Add to your `printer.cfg`:

```ini
[backlash]
backlash_x: 0.15    # mm of mechanical backlash on X axis
backlash_y: 0.08    # mm on Y axis
backlash_z: 0.02    # mm on Z axis
```

Measure backlash with a dial indicator against the toolhead while manually
reversing each axis.

### G-code Commands

```
SET_BACKLASH X=0.12 Y=0.08 Z=0.02    # Adjust at runtime
GET_BACKLASH                           # Query current values
```

### Compatibility

Works with **all kinematics types**: Cartesian, CoreXY, CoreXZ, Delta,
Deltesian, Polar, and more. The implementation uses a wrapper pattern
(similar to Input Shaper) that intercepts the position calculation callback
without modifying the underlying kinematics code.

### Implementation

| File | Purpose |
|------|---------|
| `klippy/chelper/kin_backlash.c` | Core C logic — direction detection and position offset |
| `klippy/extras/backlash.py` | Python module — config parsing, G-code commands, stepper wrapping |
| `klippy/chelper/__init__.py` | FFI bindings for the C backlash functions |

See [BACKLASH_PROJECT.md](BACKLASH_PROJECT.md) for detailed architecture
documentation.
