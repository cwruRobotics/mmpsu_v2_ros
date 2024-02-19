# MMPSU v2 ROS Package

## Services
| Service Name              | Service Type          | Description   |
|---------------------------|-----------------------|---------------|
|`mmpsu/set_output_enabled` | `std_srvs SetBool`    | True enables the main output, false disables. |
|`mmpsu/set_vout`           | `mmpsu_v2 SetFloat32` | Sets the main output voltage. Clamped to between 0.0 and 24.0 |
|`mmpsu/set_iout_limit`     | `mmpsu_v2 SetFloat32` | Sets the total main output current limit. It does this by actually dividing the limit across the number of present phases. |

## Topics
| Topic Name                | Data Type             | Description   |
|---------------------------|-----------------------|---------------|
|`mmpsu/mmpsu_status`       | `mmpsu_v2 MmpsuStatus`| See [MmpsuStatus](#mmpsustatus) documentation below |


## Message and Service Types

### `MmpsuStatus`
This message type provides many telemetry data of interest (and many of no interest too!).

| Field name            | Type          | Description   |
|-----------------------|---------------|---------------|
| `output_enabled`      | `bool`        | Reports whether the output is actually enabled. |
| `vout_measured`       | `float32`     | Reports the measured value of the main output in volts. |
| `vout_setpoint`       | `float32`     | Reports the target (setpoint) of the main output in volts. |
| `iout_measured`       | `float32`     | Reports the total current measured out of the main output. Actually a sum of each enabled phase's current. |
| `phase_present`       | `bool[6]`     | A list of which phases are inserted/present (determined at startup, hot swapping cards is strongly discouraged). |
| `phase_enabled`       | `bool[6]`     | A list of which present phases are enabled right now. (changes with dynamic load conditions). |
| `phase_duty_cycle`    | `float32[6]`  | A list of each present phase's PWM duty cycle, ranging from 0.0 ~ 1.0 |
| `phase_current`       | `float32[6]`  | A list of each present phase's measured current (they may not be sharing 100% equally) |
| `phase_current_limit` | `float32[6]`  | A list of each present phase's programmed current limit |
| `phase_temp`          | `float32[6]`  | A list of each present phase's measured temperature in degrees C. -float('inf') for non-present phases. |
| `phase_overtemp`      | `float32[6]`  | A list of which present phase's are disabled due to overtemp. |
| `comms_err_count`     | `int32`       | A running count of comms errors we've encountered. |
| `vin_voltage`         | `float32`     | Measured voltage in volts at the power input to the MMPSU |
| `vin_current`         | `float32`     | Measured current in amps into the MMPSU's power input |
| `v5p0_voltage`        | `float32`     | Measured voltage in volts of the aux. 5V output. |
| `v5p0_current`        | `float32`     | Measured current in amps flowing out of the 5V aux output. |
| `v12p0_voltage`       | `float32`     | Measured voltage in volts of the aux 12V output. |
| `v12p0_current`       | `float32`     | Measured current in amps flowing out of the 12V aux output. |
| `rx_crc_err_count`    | `int32`       | Count of how many CRC errors we've encountered on packets from MMPSU==>us. Resets on reach transmission |
| `mmpsu_crc_err_count` | `int32`       | Count of how many CRC errors the MMPSU has reported to have encountered. Resets on reach transmission |
| `alarms`              | `string`      | A human readable string of alarm codes raised by MMPSU. Clears on each time slice. Will often be empty. |
