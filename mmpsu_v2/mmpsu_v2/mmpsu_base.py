"""Classes for communicating with MMPSU v2"""

from enum import IntEnum
from abc import ABC, abstractmethod, abstractproperty
from typing import Any, Dict, List
import struct
from dataclasses import dataclass

MMPSU_UART_BAUD = 115200

START_BYTE          = 0xA5
HDDR_SIZE           = 4
CMD_IND             = 1
PAYLOAD_LEN_L_IND   = 2
PAYLOAD_LEN_H_IND   = 3
CRC_SIZE            = 2

#==============================================================================
@dataclass
class FieldData:
    """Holds data about a MMSPU field/register"""
    name: str
    reg: int
    field_type: type
    value: Any
    description: str = ''
    units: str = ''
    writable: bool = False

    def pack_into(self, dst: bytearray, value: Any, offset: int) -> int:
        """Pack value and this object's register number into the given 
        bytearray and offset depending on this field's type"""
        fmt_str = ''
        if self.field_type == int:
            fmt_str = '<Bi'
        elif self.field_type == float:
            fmt_str = 'Bf'
        elif self.field_type == bool:
            fmt_str = 'B?xxx'
        elif self.field_type == bytes or self.field_type == bytearray:
            fmt_str = f'B{len(value)}s'
        else:
            # unrecognized type, do nothing
            return 0
        struct.pack_into(fmt_str, dst, offset, self.reg, value)
        return struct.calcsize(fmt_str)

    def unpack_from(self, src: bytearray, offset: int) -> Any:
        """Unpack value from the given bytearray and offset, depending on this 
        field's type. Note we need to be pointing at the value, not the reg
        number."""
        fmt_str = ''
        if self.field_type == int:
            fmt_str = '<i'
        elif self.field_type == float:
            fmt_str = 'f'
        elif self.field_type == bool:
            fmt_str = '?xxx'
        elif self.field_type == bytes or self.field_type == bytearray:
            fmt_str = f'4s'
        else:
            # unrecognized type, do nothing
            return 0

        retval = struct.unpack_from(fmt_str, src, offset)
        return retval[0]

#==============================================================================
class MmpsuV2Commands(IntEnum):
    """Command set/msg types"""
    CMD_NONE = 0
    CMD_READ = 1
    CMD_WRITE = 2
    CMD_READ_ALL = 3
    CMD_TEST_COMMS = 4
    CMD_ERROR = 5

#==============================================================================
class MmpsuV2ErrorCodes(IntEnum):
    """Command set/msg types"""
    ERR_NONE = 0
    ERR_CRC = 1
    ERR_UNKNOWN_CMD = 2

MMPSU_ERR_CODES = {
    MmpsuV2ErrorCodes.ERR_NONE: "",
    MmpsuV2ErrorCodes.ERR_CRC: "ERR_CRC",
    MmpsuV2ErrorCodes.ERR_UNKNOWN_CMD: "ERR_UNKNOWN_CMD"
}

#==============================================================================
class MmpsuV2Alarms(IntEnum):
    """Alarms of things that can be wrong"""
    ALARM_NONE              = 0
    ALARM_IOUT_OVERCURRENT  = 1
    ALARM_PHASE_OVERCURRENT = (1 << 1)
    ALARM_PHASE_OVERTEMP    = (1 << 2)
    ALARM_VIN_UNDERVOLT     = (1 << 3)
    ALARM_5P0_OVERCURRENT   = (1 << 4)
    ALARM_5P0_UNDERVOLT     = (1 << 5)
    ALARM_12P0_OVERCURRENT  = (1 << 6)
    ALARM_12P0_UNDERVOLT    = (1 << 7)

MMPSU_ALARM_CODES = {
    MmpsuV2Alarms.ALARM_NONE: "", # print nothing
    MmpsuV2Alarms.ALARM_IOUT_OVERCURRENT: "IOUT_OVERCURRENT",
    MmpsuV2Alarms.ALARM_PHASE_OVERCURRENT: "PHASE_OVERCURRENT",
    MmpsuV2Alarms.ALARM_PHASE_OVERTEMP: "PHASE_OVERTEMP",
    MmpsuV2Alarms.ALARM_VIN_UNDERVOLT: "VIN_UNDERVOLT",
    MmpsuV2Alarms.ALARM_5P0_OVERCURRENT: "5P0_OVERCURRENT",
    MmpsuV2Alarms.ALARM_5P0_UNDERVOLT: "5P0_UNDERVOLT",
    MmpsuV2Alarms.ALARM_12P0_OVERCURRENT: "12P0_OVERCURRENT",
    MmpsuV2Alarms.ALARM_12P0_UNDERVOLT: "12P0_UNDERVOLT"
}

#==============================================================================
class MmpsuV2RegMap(IntEnum):
    """Register mapping for MMPSU v2"""
    OUTPUT_ENABLED = 0
    VOUT_MEASURED = 1
    VOUT_SETPOINT = 2
    PHASES_PRESENT = 3
    PHASES_ENABLED = 4
    PHASE_A_DUTY_CYCLE = 5
    PHASE_A_CURRENT = 6
    PHASE_A_CURRENT_LIMIT = 7
    PHASE_A_TEMP = 8
    PHASE_B_DUTY_CYCLE = 9
    PHASE_B_CURRENT = 10
    PHASE_B_CURRENT_LIMIT = 11
    PHASE_B_TEMP = 12
    PHASE_C_DUTY_CYCLE = 13
    PHASE_C_CURRENT = 14
    PHASE_C_CURRENT_LIMIT = 15
    PHASE_C_TEMP = 16
    PHASE_D_DUTY_CYCLE = 17
    PHASE_D_CURRENT = 18
    PHASE_D_CURRENT_LIMIT = 19
    PHASE_D_TEMP = 20
    PHASE_E_DUTY_CYCLE = 21
    PHASE_E_CURRENT = 22
    PHASE_E_CURRENT_LIMIT = 23
    PHASE_E_TEMP = 24
    PHASE_F_DUTY_CYCLE = 25
    PHASE_F_CURRENT = 26
    PHASE_F_CURRENT_LIMIT = 27
    PHASE_F_TEMP = 28
    PHASES_IN_OVERTEMP = 29
    DEBUG_MODE = 30
    DEVELOPER_MODE = 31
    SYSTEM_STATE = 32
    VOLTAGE_KP = 33
    VOLTAGE_KI = 34
    CURRENT_KP = 35
    CURRENT_KI = 36
    COMMS_ERROR_COUNT = 37
    MANUAL_MODE = 38
    PHASE_COUNT_REQUESTED = 39
    V_IN = 40
    I_IN = 41
    V_12P0 = 42
    I_12P0 = 43
    V_5P0 = 44
    I_5P0 = 45
    V_IN_UV_THRESH = 46
    OVERTEMP_THRESH = 47
    ALARMS = 48


#==============================================================================
MMPSU_V2_REGS = [
    FieldData('OUTPUT_ENABLED', MmpsuV2RegMap.OUTPUT_ENABLED, bool, True, description='Enable or disable the output.', writable=True),
    FieldData('VOUT_MEASURED', MmpsuV2RegMap.VOUT_MEASURED, int, 0, description='Measured output voltage in millivolts.', units='mV'),
    FieldData('VOUT_SETPOINT', MmpsuV2RegMap.VOUT_SETPOINT, int, 12000, description='Output voltage setpoint in millivolts.', units='mV', writable=True),
    FieldData('PHASES_PRESENT', MmpsuV2RegMap.PHASES_PRESENT, int, 0, description='Number of phases inserted on power up.'),
    FieldData('PHASES_ENABLED', MmpsuV2RegMap.PHASES_ENABLED, int, 0, description='Number of phases currently enabled.'),
    FieldData('PHASE_A_DUTY_CYCLE', MmpsuV2RegMap.PHASE_A_DUTY_CYCLE, int, 0, description='Phase A duty cycle in counts.'),
    FieldData('PHASE_A_CURRENT', MmpsuV2RegMap.PHASE_A_CURRENT, int, 0, description='Phase A current in milliamps.', units='mA'),
    FieldData('PHASE_A_CURRENT_LIMIT', MmpsuV2RegMap.PHASE_A_CURRENT_LIMIT, int, 30, description='Phase A DC current limit in milliamps.', units='mA', writable=True),
    FieldData('PHASE_B_DUTY_CYCLE', MmpsuV2RegMap.PHASE_B_DUTY_CYCLE, int, 0, description='Phase B duty cycle in counts.'),
    FieldData('PHASE_A_TEMP', MmpsuV2RegMap.PHASE_A_TEMP, float, 0, description='Phase A approximate temperature in degrees C', units='°C'),
    FieldData('PHASE_B_CURRENT', MmpsuV2RegMap.PHASE_B_CURRENT, int, 0, description='Phase B current in milliamps.', units='mA'),
    FieldData('PHASE_B_CURRENT_LIMIT', MmpsuV2RegMap.PHASE_B_CURRENT_LIMIT, int, 30, description='Phase B DC current limit in milliamps.', units='mA', writable=True),
    FieldData('PHASE_B_TEMP', MmpsuV2RegMap.PHASE_B_TEMP, float, 0, description='Phase B approximate temperature in degrees C', units='°C'),
    FieldData('PHASE_C_DUTY_CYCLE', MmpsuV2RegMap.PHASE_C_DUTY_CYCLE, int, 0, description='Phase C duty cycle in counts.'),
    FieldData('PHASE_C_CURRENT', MmpsuV2RegMap.PHASE_C_CURRENT, int, 0, description='Phase C current in milliamps.', units='mA'),
    FieldData('PHASE_C_CURRENT_LIMIT', MmpsuV2RegMap.PHASE_C_CURRENT_LIMIT, int, 30, description='Phase C DC current limit in milliamps.', units='mA', writable=True),
    FieldData('PHASE_C_TEMP', MmpsuV2RegMap.PHASE_C_TEMP, float, 0, description='Phase C approximate temperature in degrees C', units='°C'),
    FieldData('PHASE_D_DUTY_CYCLE', MmpsuV2RegMap.PHASE_D_DUTY_CYCLE, int, 0, description='Phase D duty cycle in counts.'),
    FieldData('PHASE_D_CURRENT', MmpsuV2RegMap.PHASE_D_CURRENT, int, 0, description='Phase D current in milliamps.', units='mA'),
    FieldData('PHASE_D_CURRENT_LIMIT', MmpsuV2RegMap.PHASE_D_CURRENT_LIMIT, int, 30, description='Phase D DC current limit in milliamps.', units='mA', writable=True),
    FieldData('PHASE_D_TEMP', MmpsuV2RegMap.PHASE_D_TEMP, float, 0, description='Phase D approximate temperature in degrees C', units='°C'),
    FieldData('PHASE_E_DUTY_CYCLE', MmpsuV2RegMap.PHASE_E_DUTY_CYCLE, int, 0, description='Phase E duty cycle in counts.'),
    FieldData('PHASE_E_CURRENT', MmpsuV2RegMap.PHASE_E_CURRENT, int, 0, description='Phase E current in milliamps.', units='mA'),
    FieldData('PHASE_E_CURRENT_LIMIT', MmpsuV2RegMap.PHASE_E_CURRENT_LIMIT, int, 30, description='Phase E DC current limit in milliamps.', units='mA', writable=True),
    FieldData('PHASE_E_TEMP', MmpsuV2RegMap.PHASE_E_TEMP, float, 0, description='Phase E approximate temperature in degrees C', units='°C'),
    FieldData('PHASE_F_DUTY_CYCLE', MmpsuV2RegMap.PHASE_F_DUTY_CYCLE, int, 0, description='Phase F duty cycle in counts.'),
    FieldData('PHASE_F_CURRENT', MmpsuV2RegMap.PHASE_F_CURRENT, int, 0, description='Phase F current in milliamps.', units='mA'),
    FieldData('PHASE_F_CURRENT_LIMIT', MmpsuV2RegMap.PHASE_F_CURRENT_LIMIT, int, 30, description='Phase F DC current limit in milliamps.', units='mA', writable=True),
    FieldData('PHASE_F_TEMP', MmpsuV2RegMap.PHASE_F_TEMP, float, 0, description='Phase F approximate temperature in degrees C', units='°C'),
    FieldData('PHASES_IN_OVERTEMP', MmpsuV2RegMap.PHASES_IN_OVERTEMP, int, 0, description='Bit-field indicating which fields are in overtemp'),
    FieldData('DEBUG_MODE', MmpsuV2RegMap.DEBUG_MODE, int, 0, description='Put the MMPSU into debug mode.', writable=True),
    FieldData('DEVELOPER_MODE', MmpsuV2RegMap.DEVELOPER_MODE, bool, False, description='Put the MMPSU into developer mode.', writable=True),
    FieldData('SYSTEM_STATE', MmpsuV2RegMap.SYSTEM_STATE, int, 0, description='Power system FSM state.'),
    FieldData('VOLTAGE_KP', MmpsuV2RegMap.VOLTAGE_KP, int, 0, description='Proportional control coefficient for voltage control.'),
    FieldData('VOLTAGE_KI', MmpsuV2RegMap.VOLTAGE_KI, int, 0, description='Integral control coefficient for voltage control.'),
    FieldData('CURRENT_KP', MmpsuV2RegMap.CURRENT_KP, int, 0, description='Proportional control coefficient for current balancing.'),
    FieldData('CURRENT_KI', MmpsuV2RegMap.CURRENT_KI, int, 0, description='Integral control coefficient for current balancing.'),
    FieldData('COMMS_ERROR_COUNT', MmpsuV2RegMap.COMMS_ERROR_COUNT, int, 0, description='Count of communication link errors, user resettable.', writable=True),
    FieldData('MANUAL_MODE', MmpsuV2RegMap.MANUAL_MODE, bool, False, description='Put the MMPSU into manual control mode.', writable=True),
    FieldData('PHASE_COUNT_REQUESTED', MmpsuV2RegMap.PHASE_COUNT_REQUESTED, int, 0, description='Requested number of phases to be operating (manual mode).', writable=True),
    FieldData('V_IN', MmpsuV2RegMap.V_IN, int, 0, description='Input voltage in millivolts.', units='mV'),
    FieldData('I_IN', MmpsuV2RegMap.I_IN, int, 0, description='Input current in milliamps.', units='mA'),
    FieldData('V_12P0', MmpsuV2RegMap.V_12P0, int, 0, description='Auxilliary 12V rail voltage in millivolts.', units='mV'),
    FieldData('I_12P0', MmpsuV2RegMap.I_12P0, int, 0, description='Auxilliary 12V rail current in milliamps.', units='mA'),
    FieldData('V_5P0', MmpsuV2RegMap.V_5P0, int, 0, description='Auxilliary 5V rail voltage in millivolts.', units='mV'),
    FieldData('I_5P0', MmpsuV2RegMap.I_5P0, int, 0, description='Auxilliary 5V rail current in milliamps.', units='mA'),
    FieldData('V_IN_UV_THRESH', MmpsuV2RegMap.V_IN_UV_THRESH, int, 24000, description='Threshold in millivolts for firing the Vin undervolt alarm.', units='mV'),
    FieldData('OVERTEMP_THRESH', MmpsuV2RegMap.OVERTEMP_THRESH, float, 85.0, description='Threshold temperature in degrees C at which to trip the overtemp', units='Celcius'),
    FieldData('ALARMS', MmpsuV2RegMap.ALARMS, int, 0, description='Alarms that have fired, clears on read')
]

#==============================================================================
class MmpsuV2Base(ABC):
    """A base/abstract class for controlling the MMPSU v2"""

    def __init__(self, regmap: List[FieldData]):
        self._regmap = {field_data.name: field_data for field_data in regmap}
        self._regmap_by_num = {field_data.reg: field_data for field_data in regmap}

    @abstractmethod
    def read_fields(self, fields: List[str]) -> Dict[str, Any]:
        """Read a list of fields"""

    @abstractmethod
    def write_fields(self, field_values: Dict[str, Any]) -> bool:
        """Write a dictionary of fields"""

    def read_field(self, field: str) -> Any:
        """Read a single field"""
        return self.read_fields([field])[field]

    def write_field(self, field: str, field_value: Any) -> bool:
        """Write a single field, return True on success"""
        return self.write_fields({field: field_value})


#==============================================================================
# HELPER FUNCTIONS
#==============================================================================
def get_payload_size(packet: bytearray | bytes) -> int:
    """Extract the payload length from the given packet. You can actually 
    pass in just a header."""
    return packet[PAYLOAD_LEN_L_IND] | (packet[PAYLOAD_LEN_H_IND] << 8)

def get_packet_size(packet: bytearray | bytes) -> int:
    """Extract the expected packet size from the given packet based on the 
    header. You can actually pass in just a header"""
    return get_payload_size(packet) + HDDR_SIZE + CRC_SIZE

def set_packet_size(packet: bytearray | bytes, length: int):
    """Pack the packet size into the header of the given packet"""
    packet[PAYLOAD_LEN_L_IND] = length & 0xFF
    packet[PAYLOAD_LEN_H_IND] = (length >> 8) & 0xFF

def get_checksum(packet: bytearray | bytes) -> int:
    """Extract the checksum from the packet"""
    ind = HDDR_SIZE + get_payload_size(packet)
    return (packet[ind] << 8) | packet[ind + 1]

def get_alarm_string(alarms: int) -> str:
    """Construct a string from the alarms present."""
    alarm_strs = []
    for i in range(0, len(MmpsuV2Alarms) - 2):
        ind = (1 << i) & alarms
        if ind != 0:
            alarm_strs.append(MMPSU_ALARM_CODES[ind])
    return ' | '.join(alarm_strs)