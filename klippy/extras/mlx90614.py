# MLX90614 I2C-based Contactless Temperature Sensing
#
# Copyright (C) 2025 Pascal Wistinghausen <pascal@concepts3d.ca>
# based on the Klipper AHT10 Module by Scott Mudge <mail@scottmudge.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from . import bus

######################################################################
# Compatible Sensors:
#       MLX90614ESF-BCC -    Tested on RP2040
######################################################################

MLX_I2C_ADDR = 0x5A

MLX_REGS = {
    'TA': 0x06,
    'TOBJ1': 0x07,
    'TOBJ2': 0x08
}


class MLX90614:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        self.i2c = bus.MCU_I2C_from_config(
            config, default_addr=MLX_I2C_ADDR, default_speed=100000)
        self.report_time = config.getint('mlx90614_report_time',30,minval=1)
        self.datasource = config.getchoice('mlx90614_datasource',["ambient","object1","object2","objectaverage"])
        self.temp = self.min_temp = self.max_temp = 0.
        self.sample_timer = None
        self.printer.add_object("mlx90614 " + self.name, self)
        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)

        self.printer.register_event_handler("klippy:shutdown",
                                            self.handle_shutdown)

    def handle_connect(self):
        self.sample_timer = self.reactor.register_timer(self._sample_mlx, self.reactor.NOW)

    def handle_shutdown(self):
        if self.sample_timer is not None:
            self.reactor.unregister_timer(self.sample_timer)
            self.sample_timer = None


    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp

    def setup_callback(self, cb):
        self._callback = cb

    def get_report_time_delta(self):
        return self.report_time

    def _read_float_from_ram(self, addr):

        result = self.i2c.i2c_read([addr],3)
        if result is None:
            logging.warning("mlx90614: received data from i2c_read is None")
            return 0

        data = bytearray(result['response'])
        raw = ((data[1]) << 8) | data[0]
        temp_kelvin = raw * 0.02
        temp_celsius = temp_kelvin - 273.15

        return temp_celsius



    def _make_measurement(self):

        try:

            readaddr = 0
            if self.datasource == "ambient":
                readaddr = MLX_REGS["TA"]
            elif self.datasource == "object1" or self.datasource == "objectaverage":
                readaddr = MLX_REGS["TOBJ1"]
            elif self.datasource == "object2":
                readaddr = MLX_REGS["TOBJ2"]

            measurement = self._read_float_from_ram(readaddr)

            if self.datasource == "objectaverage":
                measurement += self._read_float_from_ram(MLX_REGS["TOBJ2"])
                measurement /= 2

        except Exception as e:
            logging.exception("mlx90614: exception encountered" +
                              " reading data: %s"%str(e))
            return False

        self.temp = measurement

        return True


    def _sample_mlx(self, eventtime):
        if not self._make_measurement():
            self.temp = .0
            return self.reactor.NEVER

        if self.temp < self.min_temp or self.temp > self.max_temp:
            self.printer.invoke_shutdown(
                "MLX90614 temperature %0.1f outside range of %0.1f:%.01f"
                % (self.temp, self.min_temp, self.max_temp))

        measured_time = self.reactor.monotonic()
        print_time = self.i2c.get_mcu().estimated_print_time(measured_time)
        self._callback(print_time, self.temp)
        return measured_time + self.report_time

    def get_status(self, eventtime):
        return {
            'temperature': round(self.temp, 2),
        }


def load_config(config):
    # Register sensor
    pheater = config.get_printer().lookup_object("heaters")
    pheater.add_sensor_factory("MLX90614", MLX90614)
