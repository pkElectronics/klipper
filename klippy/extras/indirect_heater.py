

KELVIN_TO_CELSIUS = -273.15

class IndirectHeater:

    def __init__(self,config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()


        pheaters = self.printer.load_object(config, 'heaters')
        self.sensor = pheaters.setup_sensor(config)
        self.min_temp = config.getfloat('min_temp', KELVIN_TO_CELSIUS,
                                        minval=KELVIN_TO_CELSIUS)
        self.max_temp = config.getfloat('max_temp', 99999999.9,
                                        above=self.min_temp)
        self.sensor.setup_minmax(self.min_temp, self.max_temp)
        self.sensor.setup_callback(self.temperature_callback)
        pheaters.register_sensor(config, self)

        self.last_temp = 0.
        self.measured_min = 99999999.
        self.measured_max = 0.

        self.target_temp = 0

        self.heater_name = config.get("heater_name")
        self.heater_obj = self.printer.lookup_object(self.heater_name)

        algos = {'watermark': ControlBangBang, 'pid': ControlPID}
        algo = config.getchoice('control', algos)
        self.control = algo(self, config)

        self.indirect_mode = config.getchoice("indirect_control_mode",{"delta","full"})

        if self.indirect_mode == "delta":
            self.indirect_control_delta = config.getfloat("indirect_control_delta",0, minval=0)

        self.indirect_control_tmax = config.getfloat("indirect_control_tmax",100,minval=0)

        #self.indirect_control_cycle_time = config.getint("indirect_control_cycle_time",1,minval=0,maxval=20)



    def temperature_callback(self, read_time, temp):
        self.last_temp = temp
        if temp:
            self.measured_min = min(self.measured_min, temp)
            self.measured_max = max(self.measured_max, temp)

        self.control.temperature_update(read_time, temp, self.target_temp, self.get_setpoint_baseline_temp())


    def get_setpoint_baseline_temp(self):

        if self.indirect_mode == "delta":
            indirect_target = min(self.target_temp + self.indirect_control_delta, self.indirect_control_tmax)
        else:
            indirect_target = self.indirect_control_tmax

        return indirect_target


######################################################################
# Bang-bang control algo
######################################################################

class ControlBangBang:
    def __init__(self, heater, config):
        self.heater = heater
        self.heater_max_power = heater
        self.max_delta = config.getfloat('max_delta', 2.0, above=0.)
        self.heating = False

    def temperature_update(self, read_time, temp, target_temp, setpoint_baseline_temp):
        if self.heating and temp >= target_temp+self.max_delta:
            self.heating = False
        elif not self.heating and temp <= target_temp-self.max_delta:
            self.heating = True

        if self.heating:
            self.heater.heater_obj.set_temp(setpoint_baseline_temp)
        else:
            self.heater.heater_obj.set_temp(0.)

    def check_busy(self, eventtime, smoothed_temp, target_temp):
        return smoothed_temp < target_temp-self.max_delta


######################################################################
# Proportional Integral Derivative (PID) control algo
######################################################################

PID_SETTLE_DELTA = 1.
PID_SETTLE_SLOPE = .1
AMBIENT_TEMP = 25.
PID_PARAM_BASE = 255.

class ControlPID:
    def __init__(self, heater, config):
        self.heater = heater
        self.Kp = config.getfloat('pid_Kp') / PID_PARAM_BASE
        self.Ki = config.getfloat('pid_Ki') / PID_PARAM_BASE
        self.Kd = config.getfloat('pid_Kd') / PID_PARAM_BASE
        self.min_deriv_time = heater.get_smooth_time()
        self.temp_integ_max = 0.
        if self.Ki:
            self.temp_integ_max = 1 / self.Ki
        self.prev_temp = AMBIENT_TEMP
        self.prev_temp_time = 0.
        self.prev_temp_deriv = 0.
        self.prev_temp_integ = 0.

    def temperature_update(self, read_time, temp, target_temp, setpoint_baseline_temp):
        time_diff = read_time - self.prev_temp_time
        # Calculate change of temperature
        temp_diff = temp - self.prev_temp
        if time_diff >= self.min_deriv_time:
            temp_deriv = temp_diff / time_diff
        else:
            temp_deriv = (self.prev_temp_deriv * (self.min_deriv_time-time_diff)
                          + temp_diff) / self.min_deriv_time
        # Calculate accumulated temperature "error"
        temp_err = target_temp - temp
        temp_integ = self.prev_temp_integ + temp_err * time_diff
        temp_integ = max(0., min(self.temp_integ_max, temp_integ))
        # Calculate output
        co = self.Kp*temp_err + self.Ki*temp_integ - self.Kd*temp_deriv
        #logging.debug("pid: %f@%.3f -> diff=%f deriv=%f err=%f integ=%f co=%d",
        #    temp, read_time, temp_diff, temp_deriv, temp_err, temp_integ, co)
        bounded_co = max(0., min(1.0, co))
        self.heater.heater_obj.set_temp(setpoint_baseline_temp * bounded_co)
        # Store state for next measurement
        self.prev_temp = temp
        self.prev_temp_time = read_time
        self.prev_temp_deriv = temp_deriv
        if co == bounded_co:
            self.prev_temp_integ = temp_integ

    def check_busy(self, eventtime, smoothed_temp, target_temp):
        temp_diff = target_temp - smoothed_temp
        return (abs(temp_diff) > PID_SETTLE_DELTA
                or abs(self.prev_temp_deriv) > PID_SETTLE_SLOPE)


def load_config(config):
    return IndirectHeater(config)