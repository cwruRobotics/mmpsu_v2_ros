import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType
from mmpsu_v2 import mmpsu_v2_uart
from mmpsu_v2 import mmpsu_base
from std_srvs.srv import SetBool

from mmpsu_v2_interfaces.srv import SetFloat32

from mmpsu_v2_interfaces.msg import MmpsuCoreTelem, MmpsuAuxTelem


class MmpsuV2Node(Node):
    """MMPSU v2 ROS 2 node"""

    PHASE_MAPPING = {0: "A", 1: "B", 2: "C", 3: "D", 4: "E", 5: "F"}

    # fields that need to be pulled for a core telemetry message
    CORE_TELEM_FIELDS = [
        "OUTPUT_ENABLED",
        "VOUT_MEASURED",
        "VOUT_SETPOINT",
        "PHASES_PRESENT",
        "PHASES_ENABLED",
        "PHASE_A_DUTY_CYCLE",
        "PHASE_A_CURRENT",
        "PHASE_A_CURRENT_LIMIT",
        "PHASE_A_TEMP",
        "PHASE_B_DUTY_CYCLE",
        "PHASE_B_CURRENT",
        "PHASE_B_CURRENT_LIMIT",
        "PHASE_B_TEMP",
        "PHASE_C_DUTY_CYCLE",
        "PHASE_C_CURRENT",
        "PHASE_C_CURRENT_LIMIT",
        "PHASE_C_TEMP",
        "PHASE_D_DUTY_CYCLE",
        "PHASE_D_CURRENT",
        "PHASE_D_CURRENT_LIMIT",
        "PHASE_D_TEMP",
        "PHASE_E_DUTY_CYCLE",
        "PHASE_E_CURRENT",
        "PHASE_E_CURRENT_LIMIT",
        "PHASE_E_TEMP",
        "PHASE_F_DUTY_CYCLE",
        "PHASE_F_CURRENT",
        "PHASE_F_CURRENT_LIMIT",
        "PHASE_F_TEMP",
        "PHASES_IN_OVERTEMP",
        "ALARMS",
    ]

    # fields that need to be pulled for an aux telemetry message
    AUX_TELEM_FIELDS = [
        "COMMS_ERROR_COUNT",
        "V_IN",
        "I_IN",
        "V_12P0",
        "I_12P0",
        "V_5P0",
        "I_5P0",
    ]

    def __init__(self):
        super().__init__("mmpsu_v2")
        # publishers
        self.core_telem_pub = self.create_publisher(MmpsuCoreTelem, "mmpsu_core", 10)
        self.aux_telem_pub = self.create_publisher(MmpsuAuxTelem, "mmpsu_aux", 10)

        # services
        self.output_enable = self.create_service(
            SetBool, "set_output_enabled", self.set_output_enabled_callback
        )
        self.vout_set_srv = self.create_service(
            SetFloat32, "set_vout", self.set_output_voltage_callback
        )
        self.iout_limit_srv = self.create_service(
            SetFloat32, "set_iout_limit", self.set_iout_limit_callack
        )
        self.manual_mode_srv = self.create_service(
            SetBool, "set_manual_mode", self.set_manual_mode_callback
        )
        self.phase_count_srv = self.create_service(
            SetFloat32, "set_phase_count", self.set_phase_count_callback
        )

        # parameter things
        core_telem_param_desc = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description="Period for polling the core telemetry fields.",
        )
        uart_path_param_desc = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING, description="UART device path."
        )
        aux_telem_param_desc = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description="Period for polling the auxilliary telemetry fields.",
        )

        self.declare_parameter("core_telem_period", None, core_telem_param_desc)
        self.declare_parameter("uart_path", None, uart_path_param_desc)
        self.declare_parameter("aux_telem_period", None, aux_telem_param_desc)

        try:
            self.core_status_period = (
                self.get_parameter("core_telem_period").get_parameter_value().double_value
            )
        except ParameterNotDeclaredException:
            self.core_status_period = 0.1

        try:
            self.uart_path = self.get_parameter("uart_path").get_parameter_value().string_value
        except ParameterNotDeclaredException:
            self.uart_path = "/dev/ttyS0"

        try:
            self.aux_telem_period = (
                self.get_parameter("aux_telem_period").get_parameter_value().double_value
            )
        except ParameterNotDeclaredException:
            self.aux_telem_period = 0.5

        self._mmpsu = mmpsu_v2_uart.MmpsuV2Uart(self.uart_path, self.get_logger())

        if self._mmpsu.test_comms():
            self.get_logger().info("MMPSU test comms succeeded.")
        else:
            self.get_logger().warning("MMPSU test comms failed.")

        # timers
        self.core_timer = self.create_timer(self.core_status_period, self.status_timer_callback)
        self.aux_timer = self.create_timer(self.aux_telem_period, self.aux_telem_timer_callback)

    def status_timer_callback(self):
        msg = MmpsuCoreTelem()

        fields = self._mmpsu.read_fields(self.CORE_TELEM_FIELDS)

        try:
            msg.output_enabled = fields["OUTPUT_ENABLED"]
            msg.vout_measured = float(fields["VOUT_MEASURED"]) / 1000.0
            msg.vout_setpoint = float(fields["VOUT_SETPOINT"]) / 1000.0

            iout_total = 0.0

            phases_present = fields["PHASES_PRESENT"]
            phases_enabled = fields["PHASES_ENABLED"]
            phases_overtemped = fields["PHASES_IN_OVERTEMP"]
            for phase_ind in range(0, 6):
                present = bool((phases_present >> phase_ind) & 1)
                msg.phase_present[phase_ind] = present
                if present:
                    msg.phase_enabled[phase_ind] = bool((phases_enabled >> phase_ind) & 1)
                    msg.phase_overtemp[phase_ind] = bool((phases_overtemped >> phase_ind) & 1)
                    letter = self.PHASE_MAPPING[phase_ind]
                    msg.phase_duty_cycle[phase_ind] = (
                        float(fields[f"PHASE_{letter}_DUTY_CYCLE"]) / 5440.0
                    )
                    iout_phase = float(fields[f"PHASE_{letter}_CURRENT"]) / 1000.0
                    msg.phase_current[phase_ind] = iout_phase
                    iout_total += iout_phase
                    msg.phase_current_limit[phase_ind] = (
                        float(fields[f"PHASE_{letter}_CURRENT_LIMIT"]) / 1000.0
                    )
                    msg.phase_temp[phase_ind] = fields[f"PHASE_{letter}_TEMP"]
            msg.iout_measured = iout_total
            msg.alarms = mmpsu_base.get_alarm_string(fields["ALARMS"])

        except KeyError:
            self._logger.warning("Not all core telemetry fields read.")

        self.core_telem_pub.publish(msg)

    def aux_telem_timer_callback(self):
        msg = MmpsuAuxTelem()

        fields = self._mmpsu.read_fields(self.AUX_TELEM_FIELDS)
        try:
            msg.comms_err_count = fields["COMMS_ERROR_COUNT"]
            msg.vin_voltage = float(fields["V_IN"]) / 1000.0
            msg.vin_current = float(fields["I_IN"]) / 1000.0
            msg.v5p0_voltage = float(fields["V_5P0"]) / 1000.0
            msg.v5p0_current = float(fields["I_5P0"]) / 1000.0
            msg.v12p0_voltage = float(fields["V_12P0"]) / 1000.0
            msg.v12p0_current = float(fields["I_12P0"]) / 1000.0
            msg.rx_crc_err_count = self._mmpsu.rx_crc_err_count
            msg.mmpsu_crc_err_count = self._mmpsu.mmpsu_crc_err_count
        except KeyError:
            self._logger.warning("Not all auxilliary telemetry fields read.")

        self.aux_telem_pub.publish(msg)

    def set_output_enabled_callback(self, request, response):
        response.success = self._mmpsu.write_field("OUTPUT_ENABLED", request.data)
        return response

    def set_output_voltage_callback(self, request, response):
        setpt = self._clamp(request.setpoint, self._mmpsu.MIN_VOUT, self._mmpsu.MAX_VOUT)
        response.success = self._mmpsu.write_field("VOUT_SETPOINT", int(setpt * 1000.0))
        return response

    def set_iout_limit_callack(self, req, resp):
        resp.success = True
        phases_enab = self._mmpsu.read_field("PHASES_ENABLED")
        phases_to_set = []
        for i in range(6):
            if bool((phases_enab >> i) & 1):
                phases_to_set.append(f"PHASE_{self.PHASE_MAPPING[i]}_CURRENT_LIMIT")

        per_phase_ilim = self._clamp(
            req.setpoint / len(phases_to_set),
            self._mmpsu.MIN_PHASE_CURR_LIMIT,
            self._mmpsu.MAX_PHASE_CURR_LIMIT,
        )

        field_dict = {}
        for phase_field in phases_to_set:
            field_dict[phase_field] = int(per_phase_ilim * 1000.0)

        resp.success = self._mmpsu.write_fields(field_dict)

        return resp

    def set_manual_mode_callback(self, req, resp):
        """Callback for the set_manual_mode_srv service."""
        resp.success = self._mmpsu.write_field("MANUAL_MODE", req.data)
        return resp

    def set_phase_count_callback(self, req, resp):
        """Callback for the set_phase_count_srv service."""
        count = self._clamp(int(req.setpoint), 1, 6)
        resp.success = self._mmpsu.write_field("PHASE_COUNT_REQUESTED", count)
        return resp

    def _clamp(self, value, min_val, max_val):
        if value >= min_val:
            if value <= max_val:
                return value
            else:
                return max_val
        else:
            return min_val


def main(args=None):
    rclpy.init(args=args)

    mmpsu_node = MmpsuV2Node()

    rclpy.spin(mmpsu_node)

    mmpsu_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
