"""
Driver for the Renogy Rover Solar Controller using the Modbus RTU protocol
"""

import minimalmodbus
from enum import Enum, unique

minimalmodbus.BAUDRATE = 9600
minimalmodbus.TIMEOUT = 0.5


BATTERY_TYPE = {
    1: 'open',
    2: 'sealed',
    3: 'gel',
    4: 'lithium',
    5: 'self-customized'
}

CHARGING_STATE = {
    0: 'deactivated',
    1: 'activated',
    2: 'mppt',
    3: 'equalizing',
    4: 'boost',
    5: 'floating',
    6: 'current limiting'
}

@unique
class LoadMode(Enum):
    SOLE_LIGHT_CONTROL = 0x00
    MANUAL = 0x0F
    DELAY_1_HR = 0x01
    DELAY_2_HR = 0x02
    DELAY_3_HR = 0x03
    DELAY_4_HR = 0x04
    DELAY_5_HR = 0x05
    DELAY_6_HR = 0x06
    DELAY_7_HR = 0x07
    DELAY_8_HR = 0x08
    DELAY_9_HR = 0x09
    DELAY_10_HR = 0x0A
    DELAY_11_HR = 0x0B
    DELAY_12_HR = 0x0C
    DELAY_13_HR = 0x0D
    DELAY_14_HR = 0x0E
    DEBUG = 0x10
    ALWAYS_ON = 0x11


class RenogyRover(minimalmodbus.Instrument):
    """
    Communicates using the Modbus RTU protocol (via provided USB<->RS232 cable)
    """

    def __init__(self, portname, slaveaddress):
        minimalmodbus.Instrument.__init__(self, portname, slaveaddress)

    def model(self):
        """
        Read the controller's model information
        """
        return self.read_string(12, numberOfRegisters=8)

    def system_voltage_current(self):
        """
        Read the controler's system voltage and current
        Returns a tuple of (voltage, current)
        """
        register = self.read_register(10)
        amps = register & 0x00ff
        voltage = register >> 8
        return (voltage, amps)

    def version(self):
        """
        Read the controler's software and hardware version information
        Returns a tuple of (software version, hardware version)
        """
        registers = self.read_registers(20, 4)
        soft_major = registers[0] & 0x00ff
        soft_minor = registers[1] >> 8
        soft_patch = registers[1] & 0x00ff
        hard_major = registers[2] & 0x00ff
        hard_minor = registers[3] >> 8
        hard_patch = registers[3] & 0x00ff
        software_version = 'V{}.{}.{}'.format(soft_major, soft_minor, soft_patch)
        hardware_version = 'V{}.{}.{}'.format(hard_major, hard_minor, hard_patch)
        return (software_version, hardware_version)

    def serial_number(self):
        """
        Read the controller's serial number
        """
        registers = self.read_registers(24, 2)
        return '{}{}'.format(registers[0], registers[1])

    def battery_percentage(self):
        """
        Read the battery percentage
        """
        return self.read_register(256) & 0x00ff

    def battery_voltage(self):
        """
        Read the battery voltage
        """
        return self.read_register(257, numberOfDecimals=1)

    def battery_temperature(self):
        """
        Read the battery surface temperature
        """
        register = self.read_register(259)
        battery_temp_bits = register & 0x00ff
        temp_value = battery_temp_bits & 0x0ff
        sign = battery_temp_bits >> 7
        battery_temp = -(temp_value - 128) if sign == 1 else temp_value
        return battery_temp


    def controller_temperature(self):
        """
        Read the controller temperature
        """
        register = self.read_register(259)
        controller_temp_bits = register >> 8
        temp_value = controller_temp_bits & 0x0ff
        sign = controller_temp_bits >> 7
        controller_temp = -(temp_value - 128) if sign == 1 else temp_value
        return controller_temp

    def load_voltage(self):
        """
        Read load (raspberrypi) voltage
        """
        return self.read_register(260, numberOfDecimals=1)

    def load_current(self):
        """
        Read load (raspberrypi) current
        """
        return self.read_register(261, numberOfDecimals=2)

    def load_power(self):
        """
        Read load (raspberrypi) power
        """
        return self.read_register(262)

    def solar_voltage(self):
        """
        Read solar voltage
        """
        return self.read_register(263, numberOfDecimals=1)

    def solar_current(self):
        """
        Read solar current
        """
        return self.read_register(264, numberOfDecimals=2)

    def solar_power(self):
        """
        Read solar power
        """
        return self.read_register(265)

    def charging_amp_hours_today(self):
        """
        Read charging amp hours for the current day
        """
        return self.read_register(273)

    def discharging_amp_hours_today(self):
        """
        Read discharging amp hours for the current day
        """
        return self.read_register(274)

    def power_generation_today(self):
        return self.read_register(275)

    def charging_status(self):
        return self.read_register(288) & 0x00ff

    def charging_status_label(self):
        return CHARGING_STATE.get(self.charging_status())

    def battery_capacity(self):
        return self.read_register(57346)

    def voltage_setting(self):
        register = self.read_register(57347)
        setting = register >> 8
        recognized_voltage = register & 0x00ff
        return (setting, recognized_voltage)

    def battery_type(self):
        register = self.read_register(57348)
        return BATTERY_TYPE.get(register)

    def load_mode(self):
        register = self.read_register(registeraddress=0xE01D)
        return LoadMode(register)

    def set_load_mode(self, load_mode: LoadMode):
        print("setting load mode to: ", load_mode.value)
        self.write_register(registeraddress=0xE01D, value=load_mode.value, functioncode=0x06)

    def set_load_on(self):
        self.write_register(registeraddress=0x010A, value=0x1, functioncode=0x06)

    def set_load_off(self):
        self.write_register(registeraddress=0x010A, value=0x0, functioncode=0x06)


if __name__ == "__main__":
    rover = RenogyRover('/dev/ttyUSB0', 1)
    rover.set_load_mode(load_mode=LoadMode.ALWAYS_ON)
    rover.set_load_off()
    print('Model: ', rover.model())
    print('Battery %: ', rover.battery_percentage())
    print('Battery Type: ', rover.battery_type())
    print('Battery Capacity: ', rover.battery_capacity())
    print('Battery Voltage: ', rover.battery_voltage())
    battery_temp = rover.battery_temperature()
    print('Battery Temperature: ', battery_temp, battery_temp * 1.8 + 32)
    controller_temp = rover.controller_temperature()
    print('Controller Temperature: ', controller_temp, controller_temp * 1.8 + 32)
    print('Load Voltage: ', rover.load_voltage())
    print('Load Current: ', rover.load_current())
    print('Load Power: ', rover.load_power())
    print('Load Mode: ', rover.load_mode())
    print('Charging Status: ', rover.charging_status_label())
    print('Solar Voltage: ', rover.solar_voltage())
    print('Solar Current: ', rover.solar_current())
    print('Solar Power: ', rover.solar_power())
    print('Power Generated Today (kilowatt hours): ', rover.power_generation_today())
    print('Charging Amp/Hours Today: ', rover.charging_amp_hours_today())
    print('Discharging Amp/Hours Today: ', rover.discharging_amp_hours_today())
