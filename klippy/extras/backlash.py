# Backlash compensation support
#
# Copyright (C) 2025  Klipper Backlash Contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Backlash compensation corrects for mechanical play in the drivetrain
# (gear teeth, lead screws, belts) by applying a position offset when
# the axis direction changes. This is analogous to how Pressure Advance
# compensates for filament compression, but for axis motion.
#
# Configuration:
#   [backlash]
#   backlash_x: 0.0    # mm of mechanical backlash on X axis
#   backlash_y: 0.0    # mm of mechanical backlash on Y axis
#   backlash_z: 0.0    # mm of mechanical backlash on Z axis
#
# G-code commands:
#   SET_BACKLASH [X=<value>] [Y=<value>] [Z=<value>]
#   GET_BACKLASH

import chelper


class BacklashCompensation:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        # Read backlash values from config
        self.backlash_x = config.getfloat('backlash_x', 0., minval=0.)
        self.backlash_y = config.getfloat('backlash_y', 0., minval=0.)
        self.backlash_z = config.getfloat('backlash_z', 0., minval=0.)
        # Track wrapper kinematics objects
        self.backlash_steppers = []  # list of (stepper, backlash_sk) tuples
        self.orig_stepper_kinematics = []  # original sk objects (for reference)
        # Register event handlers
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
        self.printer.register_event_handler("toolhead:set_position",
                                            self._handle_set_position)
        # Register G-code commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command("SET_BACKLASH", self.cmd_SET_BACKLASH,
                               desc=self.cmd_SET_BACKLASH_help)
        gcode.register_command("GET_BACKLASH", self.cmd_GET_BACKLASH,
                               desc=self.cmd_GET_BACKLASH_help)
    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self._setup_backlash_wrappers()
    def _handle_set_position(self):
        # Reset backlash state after homing/position reset
        ffi_main, ffi_lib = chelper.get_ffi()
        toolhead = self.toolhead
        pos = toolhead.get_position()
        for stepper, backlash_sk in self.backlash_steppers:
            ffi_lib.backlash_set_position(backlash_sk,
                                          pos[0], pos[1], pos[2])
    def _setup_backlash_wrappers(self):
        ffi_main, ffi_lib = chelper.get_ffi()
        kin = self.toolhead.get_kinematics()
        # Check if any backlash is configured
        if (self.backlash_x <= 0. and self.backlash_y <= 0.
                and self.backlash_z <= 0.):
            return
        for s in kin.get_steppers():
            if s.get_trapq() is None:
                continue
            # Get the original kinematics
            orig_sk = s.get_stepper_kinematics()
            # Allocate backlash wrapper
            backlash_sk = ffi_main.gc(ffi_lib.backlash_stepper_alloc(),
                                      ffi_lib.backlash_stepper_free)
            # Connect wrapper to original kinematics
            res = ffi_lib.backlash_set_sk(backlash_sk, orig_sk)
            if res < 0:
                continue
            # Set backlash values for each active axis
            if s.is_active_axis('x') and self.backlash_x > 0.:
                ffi_lib.backlash_set_backlash(backlash_sk, 0, self.backlash_x)
            if s.is_active_axis('y') and self.backlash_y > 0.:
                ffi_lib.backlash_set_backlash(backlash_sk, 1, self.backlash_y)
            if s.is_active_axis('z') and self.backlash_z > 0.:
                ffi_lib.backlash_set_backlash(backlash_sk, 2, self.backlash_z)
            # Replace the stepper's kinematics with the wrapper
            s.set_stepper_kinematics(backlash_sk)
            self.backlash_steppers.append((s, backlash_sk))
            self.orig_stepper_kinematics.append(orig_sk)
    def _update_backlash(self, backlash_x=None, backlash_y=None,
                         backlash_z=None):
        ffi_main, ffi_lib = chelper.get_ffi()
        if backlash_x is not None:
            self.backlash_x = backlash_x
        if backlash_y is not None:
            self.backlash_y = backlash_y
        if backlash_z is not None:
            self.backlash_z = backlash_z
        for stepper, backlash_sk in self.backlash_steppers:
            if stepper.is_active_axis('x'):
                ffi_lib.backlash_set_backlash(backlash_sk, 0, self.backlash_x)
            if stepper.is_active_axis('y'):
                ffi_lib.backlash_set_backlash(backlash_sk, 1, self.backlash_y)
            if stepper.is_active_axis('z'):
                ffi_lib.backlash_set_backlash(backlash_sk, 2, self.backlash_z)
    def get_status(self, eventtime):
        return {
            'backlash_x': self.backlash_x,
            'backlash_y': self.backlash_y,
            'backlash_z': self.backlash_z,
        }
    cmd_SET_BACKLASH_help = "Set backlash compensation parameters"
    def cmd_SET_BACKLASH(self, gcmd):
        backlash_x = gcmd.get_float('X', self.backlash_x, minval=0.)
        backlash_y = gcmd.get_float('Y', self.backlash_y, minval=0.)
        backlash_z = gcmd.get_float('Z', self.backlash_z, minval=0.)
        self._update_backlash(backlash_x, backlash_y, backlash_z)
        msg = ("backlash_x: %.6f\n"
               "backlash_y: %.6f\n"
               "backlash_z: %.6f"
               % (backlash_x, backlash_y, backlash_z))
        self.printer.set_rollover_info(self.name, "%s: %s" % (self.name, msg))
        gcmd.respond_info(msg, log=False)
    cmd_GET_BACKLASH_help = "Get current backlash compensation parameters"
    def cmd_GET_BACKLASH(self, gcmd):
        msg = ("backlash_x: %.6f\n"
               "backlash_y: %.6f\n"
               "backlash_z: %.6f"
               % (self.backlash_x, self.backlash_y, self.backlash_z))
        gcmd.respond_info(msg, log=False)


def load_config(config):
    return BacklashCompensation(config)
