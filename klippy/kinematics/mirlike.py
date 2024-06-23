# Code for handling the kinematics of a mir-like robot, where 4 motors
# in a hybrid corexy setup control xyz. See also https://github.com/Apsu/Mir
#
#
# Copyright (C) 2024  Silvio Tisato <silvio@tisato.me>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math, collections
import stepper, extras.homing

class MIRlikeKinematics:
    def __init__(self, toolhead, config):
        # Setup axis rails
        printer = config.get_printer()
        ppins = printer.lookup_object('pins')
            
        stepper_configs = [config.getsection("stepper_" + n) for n in "abcd"]
        self.steppers = []
        self.endstops = []
        self.homing_infos = []
        self.ranges = []
        self.endstop_map = {}
        
        for c in stepper_configs:
            # add stepper
            self.steppers.append(stepper.PrinterStepper(c))
            # config endstop
            endstop_pin = c.get('endstop_pin')
            pin_params = ppins.parse_pin(endstop_pin, True, True)
            # Normalize pin name
            pin_name = "%s:%s" % (pin_params['chip_name'], pin_params['pin'])
            # register the endstop
            mcu_endstop = ppins.setup_pin('endstop', endstop_pin)
            self.endstop_map[pin_name] = {'endstop': mcu_endstop,
                                        'invert': pin_params['invert'],
                                        'pullup': pin_params['pullup']}
            name = self.steppers[-1].get_name(short=True)
            self.endstops.append((mcu_endstop, name))
            query_endstops = printer.load_object(c, 'query_endstops')
            query_endstops.register_endstop(mcu_endstop, name)

            self.endstops[-1][0].add_stepper(self.steppers[-1])
            pos_endstop = c.getfloat('position_endstop')
            self.ranges.append((c.getfloat('position_min'),c.getfloat('position_max')))
            # what a mess... TODO: make my own axis class
            homing_speed = c.getfloat('homing_speed', 5.0, above=0.)
            second_homing_speed = c.getfloat(
                'second_homing_speed', homing_speed/2., above=0.)
            homing_retract_speed = c.getfloat(
                'homing_retract_speed', homing_speed, above=0.)
            homing_retract_dist = c.getfloat(
                'homing_retract_dist', 5., minval=0.)
            homing_positive_dir = c.getboolean(
                'homing_positive_dir', None)
            if homing_positive_dir is None:
                axis_len = self.ranges[-1][1] - self.ranges[-1][0]
                if pos_endstop <= self.ranges[-1][0] + axis_len / 4.:
                    homing_positive_dir = False
                elif pos_endstop >= self.ranges[-1][1] - axis_len / 4.:
                    homing_positive_dir = True
                else:
                    raise c.error(
                        "Unable to infer homing_positive_dir in section '%s'"
                        % (c.get_name(),))
                c.getboolean('homing_positive_dir', homing_positive_dir) # TODO: weird?
            elif ((homing_positive_dir
                and pos_endstop == self.ranges[-1][1])
                or (not homing_positive_dir
                    and pos_endstop == self.ranges[-1][0])):
                raise c.error(
                    "Invalid homing_positive_dir / position_endstop in '%s'"
                    % (c.get_name(),))
            self.homing_infos.append(collections.namedtuple('homing_info', [
            'speed', 'position_endstop', 'retract_speed', 'retract_dist',
            'positive_dir', 'second_homing_speed'])(
                homing_speed, pos_endstop,
                homing_retract_speed, homing_retract_dist,
                homing_positive_dir, second_homing_speed))
        # X uses A and B
        # Y uses C and D
        # Z uses everything
        
        #self.rails[2].add_extra_stepper(steppers[0]) #a
        #self.rails[2].add_extra_stepper(steppers[2]) #b
        #self.rails[2].add_extra_stepper(steppers[3]) #d
        
        self.steppers[0].setup_itersolve('corexz_stepper_alloc', b'-')
        self.steppers[1].setup_itersolve('corexz_stepper_alloc', b'+')
        self.steppers[2].setup_itersolve('coreyz_stepper_alloc', b'-')
        self.steppers[3].setup_itersolve('coreyz_stepper_alloc', b'+')
        
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
        self.limits = [(1.0, -1.0)] * 4
        # TODO:
        #ranges = [r.get_range() for r in self.rails]
        #self.axes_min = toolhead.Coord(*[r[0] for r in ranges], e=0.0)
        #self.axes_max = toolhead.Coord(*[r[1] for r in ranges], e=0.0)

    def get_steppers(self):
        return self.steppers

    def calc_position(self, stepper_positions):
        pos = [stepper_positions[s.get_name()] for s in self.get_steppers()]
        logging.error("calcpos-------------------")
        logging.error(pos) # I don't get this part. What about duplicates?
        # X = 1/2 (a-b), where a and b are the two opposite motors
        # Y = 1/2 (c-d), where c and d are the two remaining opposite motors
        # Z = 1/4 (a+b+c+d)
        return [
            0.5 * (pos[0] - pos[1]),
            0.5 * (pos[2] - pos[3]),
            0.25 * (pos[1] + pos[2] + pos[2] + pos[3]),
        ]

    def set_position(self, newpos, homing_axes):
        for stepper in self.steppers:
            stepper.set_position(newpos)
 
        #for i, rail in enumerate(self.rails):
        #    rail.set_position(newpos)
        #    if i in homing_axes:
        #        self.limits[i] = rail.get_range()

    def note_z_not_homed(self):
        # Helper for Safe Z Home
        # TODO: is this per axis or per stepper?
        self.limits[2] = (1.0, -1.0)
    
    def home(self, homing_state):
        # Each axis is homed independently or all together?
        logging.error("homing NOT IMPLEMENTED---------")
        logging.error(homing_state.get_axes())
        logging.error("---------homing")
        # TODO - homing not implemented
        homing_state.set_axes([0, 1, 2])
        homing_state.set_homed_position([170.,170., 170.])
        self.limits = self.ranges # TODO: limits should be one per axis, not per motor?

    def _motor_off(self, print_time):
        self.limits = [(1.0, -1.0)] * 4  # CHECKME: are limits per axis or rail?

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
            "axis_minimum": 0, # TODO: self.axes_min,
            "axis_maximum": 270 # TODO: self.axes_max,
        }


def load_kinematics(toolhead, config):
    return MIRlikeKinematics(toolhead, config)
