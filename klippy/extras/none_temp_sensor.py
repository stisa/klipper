# A fake sensor that reports 0 deg C at all times
#
# Copyright (C) 2022  Silvio Tisato <silvio@tisato.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

REPORT_TIME = .1

class NONE:
    def __init__(self,config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
    
        self.temp = 0.
        self.min_temp = self.max_temp = self.range_switching_error = 0.
        self.max_sample_time = None
        self.sample_timer = None
        self.printer.add_object("none " + self.name, self)

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp

    def setup_callback(self, cb):
        self._callback = cb

    def get_report_time_delta(self):
        return REPORT_TIME


def load_config(config):
    # Register sensor
    pheaters = config.get_printer().load_object(config, "heaters")
    pheaters.add_sensor_factory("NONE", NONE)
