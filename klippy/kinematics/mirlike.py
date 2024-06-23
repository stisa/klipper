# Code for handling the kinematics of a mir-like robot, where 4 motors
# in a hybrid corexy setup control xyz. See also https://github.com/Apsu/Mir
#
#
# Copyright (C) 2024  Silvio Tisato <silvio@tisato.me>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math
import stepper


class MIRlikeKinematics:
    def __init__(self, toolhead, config):
        # Setup axis rails
        steppers = [
            stepper.PrinterRail(config.getsection("stepper_" + n)) for n in "abcd"
        ]
        self.rails = steppers[:3]

        # X uses A and B
        self.rails[0].get_endstops()[0][0].add_stepper(steppers[1].get_steppers()[0])
        # Y uses C and D
        self.rails[1].get_endstops()[0][0].add_stepper(steppers[2].get_steppers()[0])
        self.rails[1].get_endstops()[0][0].add_stepper(steppers[3].get_steppers()[0])
        
        # Z uses everything
        self.rails[2].get_endstops()[0][0].add_stepper(steppers[1].get_steppers()[0])
        self.rails[2].get_endstops()[0][0].add_stepper(steppers[0].get_steppers()[0])
        self.rails[2].get_endstops()[0][0].add_stepper(steppers[3].get_steppers()[0])

        self.rails[0].setup_itersolve("corexz_stepper_alloc", b"-") 
        self.rails[1].setup_itersolve("coreyz_stepper_alloc", b"-")
        self.rails[2].setup_itersolve("cartesian_stepper_alloc", b"-")

        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        config.get_printer().register_event_handler(
            "stepper_enable:motor_off", self._motor_off
        )
        # Setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            "max_z_velocity", max_velocity, above=0.0, maxval=max_velocity
        )
        self.max_z_accel = config.getfloat(
            "max_z_accel", max_accel, above=0.0, maxval=max_accel
        )
        self.limits = [(1.0, -1.0)] * 3
        ranges = [r.get_range() for r in self.rails]
        self.axes_min = toolhead.Coord(*[r[0] for r in ranges], e=0.0)
        self.axes_max = toolhead.Coord(*[r[1] for r in ranges], e=0.0)

    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]

    def calc_position(self, stepper_positions):
        pos = [stepper_positions[rail.get_name()] for rail in self.rails]
        print(pos) # I don't get this part. What about duplicates?
        # X = 1/2 (a-b), where a and b are the two opposite motors
        # Y = 1/2 (c-d), where c and d are the two remaining opposite motors
        # Z = 1/4 (a+b+c+d)
        return [
            0.5 * (pos[0] - pos[1]),
            0.5 * (pos[2] - pos[3]),
            0.25 * (pos[1] + pos[2] + pos[2] + pos[3]),
        ]

    def set_position(self, newpos, homing_axes):
        for i, rail in enumerate(self.rails):
            rail.set_position(newpos)
            if i in homing_axes:
                self.limits[i] = rail.get_range()

    def note_z_not_homed(self):
        # Helper for Safe Z Home
        # TODO: is this per axis or per stepper?
        self.limits[2] = (1.0, -1.0)

    def home(self, homing_state):
        # Each axis is homed independently and in order
        # TODO: maybe home z first?
        for axis in homing_state.get_axes():
            rail = self.rails[axis]
            # Determine movement
            position_min, position_max = rail.get_range()
            hi = rail.get_homing_info()
            homepos = [None, None, None, None]
            homepos[axis] = hi.position_endstop
            forcepos = list(homepos)
            if hi.positive_dir:
                forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
            else:
                forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
            # Perform homing
            homing_state.home_rails([rail], forcepos, homepos)

    def _motor_off(self, print_time):
        self.limits = [(1.0, -1.0)] * 3  # CHECKME: are limits per axis or rail?

    def _check_endstops(self, move):
        end_pos = move.end_pos
        for i in (0, 1, 2):
            if move.axes_d[i] and (
                end_pos[i] < self.limits[i][0] or end_pos[i] > self.limits[i][1]
            ):
                if self.limits[i][0] > self.limits[i][1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()

    def check_move(self, move):
        limits = self.limits
        xpos, ypos = move.end_pos[:2]
        if (
            xpos < limits[0][0]
            or xpos > limits[0][1]
            or ypos < limits[1][0]
            or ypos > limits[1][1]
        ):
            self._check_endstops(move)
        if not move.axes_d[2]:
            # Normal XY move - use defaults
            return
        # Move with Z - update velocity and accel for Z axis
        self._check_endstops(move)
        z_ratio = move.move_d / abs(move.axes_d[2])
        move.limit_speed(self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)

    def get_status(self, eventtime):
        axes = [a for a, (l, h) in zip("xyz", self.limits) if l <= h]
        return {
            "homed_axes": "".join(axes),
            "axis_minimum": self.axes_min,
            "axis_maximum": self.axes_max,
        }


def load_kinematics(toolhead, config):
    return MIRlikeKinematics(toolhead, config)
