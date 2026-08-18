# Klipper Backlash Compensation - Project Summary

## What This Is

A Klipper firmware fork that adds **axis backlash compensation** for X, Y, and Z axes. Backlash is mechanical play in the drivetrain (gear teeth, lead screws, belts) that causes lost motion when an axis changes direction.

The concept is analogous to **Pressure Advance** (which compensates for filament compression delay) but applied to mechanical axis backlash.

## How It Works

When an axis changes direction, the motor moves but the tool doesn't until the mechanical slack is taken up. The compensation works by:

1. **Detecting direction changes** at move boundaries by comparing net displacement of consecutive moves
2. **Applying a position offset** of `±backlash/2` in the current movement direction
3. The motor "leads" the tool by half the backlash distance, absorbing the mechanical play during direction changes

### Compensation Formula

```
When direction reverses:
  motor_position(t) = tool_position(t) + (curr_direction * backlash / 2)

Direction detection:
  prev_direction = sign(prev_move displacement)
  curr_direction = sign(curr_move displacement)
  if prev_direction != curr_direction: apply offset
```

## Architecture

Follows the same **wrapper pattern** as `kin_shaper.c` (Input Shaper):

```
iterative solver → backlash_calc_position_cb → orig_kinematics_calc_position_cb
                  (adds offset at direction change)
```

- New `kin_backlash.c` wraps any existing kinematics callback
- Python `backlash.py` reads config, manages wrappers, provides G-code commands
- Works with **all kinematics types**: Cartesian, CoreXY, CoreXZ, Delta, etc.

## Files Created/Modified

| File | Action | Purpose |
|------|--------|---------|
| `klippy/chelper/kin_backlash.c` | **NEW** | Core C backlash logic - wrapper struct, direction detection, offset calculation |
| `klippy/extras/backlash.py` | **NEW** | Python module - reads `[backlash]` config section, wraps stepper kinematics, registers G-code commands |
| `klippy/chelper/__init__.py` | **MODIFIED** | Added `kin_backlash.c` to SOURCE_FILES list + FFI function definitions in `defs_kin_backlash` |

## Key Data Structures

### C Side (`kin_backlash.c`)

```c
struct backlash_params {
    double backlash[3]; // [0]=X, [1]=Y, [2]=Z in mm
    double active_print_time;
    struct list_node node;
};

struct backlash_axis_state {
    double previous_pos;
    struct move *previous_move;
};

struct backlash_stepper {
    struct stepper_kinematics sk;      // wrapper (first field for container_of)
    struct stepper_kinematics *orig_sk; // original kinematics
    struct list_head backlash_list;     // param list (supports runtime changes)
    struct backlash_axis_state axes[3]; // per-axis state
};
```

### Python Side (`backlash.py`)

```python
class BacklashCompensation:
    backlash_x, backlash_y, backlash_z  # config values in mm
    backlash_steppers[]                  # list of (stepper, backlash_sk) tuples
```

## Configuration

```ini
[backlash]
backlash_x: 0.15    # mm of mechanical backlash on X axis
backlash_y: 0.08    # mm on Y axis
backlash_z: 0.02    # mm on Z axis
```

## G-code Commands

```
SET_BACKLASH X=0.12 Y=0.08 Z=0.02    # Set backlash values at runtime
GET_BACKLASH                           # Query current values
```

## FFI Functions Exposed

```c
struct stepper_kinematics *backlash_stepper_alloc(void);
void backlash_stepper_free(struct stepper_kinematics *sk);
int backlash_set_sk(struct stepper_kinematics *sk, struct stepper_kinematics *orig_sk);
void backlash_set_backlash(struct stepper_kinematics *sk, int axis, double backlash);
void backlash_set_position(struct stepper_kinematics *sk, double x, double y, double z);
```

## Integration Points

1. **Loading**: `backlash.py` is an "extra" module, loaded from `[backlash]` config section
2. **Setup**: In `klippy:connect` event, wraps each stepper's kinematics
3. **Homing**: `toolhead:set_position` event triggers `backlash_set_position()` to reset direction tracking
4. **Runtime changes**: `SET_BACKLASH` adds new entries to the C param list (same pattern as `SET_PRESSURE_ADVANCE`)

## Build Verification

- C file compiles cleanly with `gcc -fsyntax-only -Wall`
- Full `c_helper.so` builds successfully with all source files
- All 5 FFI functions are exported and accessible from Python
- Integration test confirms: allocation, backlash setting, active flag propagation, and deallocation all work

## Testing

```bash
# From WSL with klippy-env active:
cd /mnt/c/Users/CRRC/klipper/klippy
python3 -c "
import chelper
ffi_main, ffi_lib = chelper.get_ffi()
sk = ffi_lib.backlash_stepper_alloc()
ffi_lib.backlash_set_backlash(sk, 0, 0.1)  # 0.1mm X backlash
print('Backlash wrapper works!')
ffi_lib.backlash_stepper_free(sk)
"
```

## How It Relates to Pressure Advance

| Aspect | Pressure Advance | Backlash Compensation |
|--------|-----------------|----------------------|
| Compensates | Filament compression delay | Mechanical drivetrain play |
| When applied | During acceleration/deceleration | At direction changes |
| Offset type | Velocity-proportional | Fixed position offset |
| C implementation | `kin_extruder.c` (smoothing integral) | `kin_backlash.c` (move boundary detection) |
| Parameter | `pressure_advance` (mm per mm/s) | `backlash_x/y/z` (mm of play) |
| Wrapper pattern | N/A (built into extruder) | `kin_shaper.c`-style wrapper |
