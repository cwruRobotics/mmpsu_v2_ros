# MMPSU v2p0 ROS2 package

## `mmpsu_v2`
Actual node code. Nodes available are the following:
* `mmpsu_v2_node`
* `mmpsu_debug_pane`

### `mmpsu_v2_node` Node
This node does the actual communication with the MMPSU over UART using pyserial and offers several topics and services (see sections below). 

#### Topics:
* `mmpsu/mmpsu_core`: Core telemetry messages of type `mmpsu_v2_interfaces.msg.MmpsuCoreTelem`.
* `mmpsu/mmpsu_aux`: Auxilliary telemetry messages of type `mmpsu_v2_interfaces.msg.MmpsuAuxTelem`.

#### Services:
* `mmpsu/set_output_enabled`: enable or disable the output (`std_srvs.srv.SetBool`)
* `mmpsu/set_vout`: set the voltage setpoint of the output (`mmpsu_v2_interfaces.srv.SetFloat32`)
* `mmpsu/set_iout_limit`: set the total output current limit (`mmpsu_v2_interfaces.srv.SetFloat32`)

### `mmpsu_debug_pane` Node
A Tkinter-based GUI that allows basic control and debugging of MMPSU. It requires the `mmpsu_v2_node` to be running to actually control the hardware.

### Parameters, launch files, and config files

The `mmpsu_v2_node` needs several parameters to be set in order to function correctly. In this repo we have an example launch file `mmpsu_v2/launch/mmpsu_v2_launch.py` which calls on a config file called `mmpsu_v2_config.yaml` in the `mmpsu_v2/config/` directory. The chief parameter you very likely need to change is the `uart_path` parameter. The value required will depend on what UART peripheral you have available and choose to use for MMPSU comms.


## `mmpsu_v2_interfaces`
Defines message and service types.

### Message types:
* `MmpsuAuxTelem`: Less important telemetry than the core telemetry.
* `MmpsuCoreTelem`: More important telemetry than the auxilliary telemetry.

### Service types:
* `SetFloat32`: Allows setting of a float32 type, analagous to the standard service Set- types.