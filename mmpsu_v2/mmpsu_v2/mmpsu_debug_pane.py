import rclpy
from rclpy.node import Node

from std_srvs.srv import SetBool
from mmpsu_v2_interfaces.srv import SetFloat32
from mmpsu_v2_interfaces.msg import MmpsuCoreTelem, MmpsuAuxTelem

import tkinter as tk


class MmpsuDebugPane(Node):

    def __init__(self):
        super().__init__("mmpsu_v2")

        self.w = tk.Tk()
        self.w.title = "MMPSU v2 Control & Debug"
        self.futures = {"set_vout": None, "set_output_enabled": None}

        # create the gui
        self.setup_widgets()

        # timer to update the gui
        self.core_timer = self.create_timer(0.025, self.gui_update_callback)

        # subscribe to mmpsu topics
        self.core_telem_sub = self.create_subscription(
            MmpsuCoreTelem, "mmpsu/mmpsu_core", self.core_telem_callback, 10
        )
        self.aux_telem_sub = self.create_subscription(
            MmpsuAuxTelem, "mmpsu/mmpsu_aux", self.aux_telem_callback, 10
        )

        # clients to control mmpsu
        self.set_vout_cli = self.create_client(SetFloat32, "mmpsu/set_vout")
        while not self.set_vout_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("MMPSU set_vout service not available. Waiting again...")

        self.set_output_enab_cli = self.create_client(SetBool, "mmpsu/set_output_enabled")
        while not self.set_output_enab_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "MMPSU set_output_enabled service not available. Waiting again..."
            )

        self.set_manual_mode_cli = self.create_client(SetBool, "mmpsu/set_manual_mode")
        while not self.set_manual_mode_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("MMPSU set_manual_mode service not available. Waiting again...")

        self.set_phase_count_cli = self.create_client(SetFloat32, "mmpsu/set_phase_count")
        while not self.set_phase_count_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("MMPSU set_phase_count service not available. Waiting again...")

    def setup_widgets(self):
        pass
        # tk.StringVar(self.w)
        self.ctrl_frame = tk.LabelFrame(self.w, text="Control")
        self.data_frame = tk.LabelFrame(self.w, text="Overall Data")
        self.phase_frame = tk.LabelFrame(self.w, text="Phase Data")

        grid_row = 0

        # OUTPUT ENABLE/DISABLE BUTTON
        self.output_state_strvar = tk.StringVar()
        tk.Label(self.ctrl_frame, text="ENABLE OUTPUT:").grid(row=grid_row, column=0)
        self.output_state_btn = tk.Button(
            self.ctrl_frame, textvariable=self.output_state_strvar, command=self.toggle_output_state
        )
        self.output_state_btn.grid(row=grid_row, column=1)
        grid_row += 1

        # VOUT SETPT
        self.vout_setpt_strvar = tk.StringVar()
        tk.Label(self.ctrl_frame, text="VOUT SETPOINT:").grid(row=grid_row, column=0)
        tk.Entry(self.ctrl_frame, textvariable=self.vout_setpt_strvar).grid(row=grid_row, column=1)
        tk.Button(self.ctrl_frame, text="SET", command=self.set_vout_setpt).grid(
            row=grid_row, column=2
        )
        grid_row += 1

        # IOUT LIMIT
        self.iout_limit_strvar = tk.StringVar()
        tk.Label(self.ctrl_frame, text="IOUT LIMIT:").grid(row=grid_row, column=0)
        tk.Entry(self.ctrl_frame, textvariable=self.iout_limit_strvar).grid(row=grid_row, column=1)
        tk.Button(self.ctrl_frame, text="SET", command=self.set_iout_limit).grid(
            row=grid_row, column=2
        )
        grid_row += 1

        # MANUAL MODE
        self.manual_mode_intvar = tk.IntVar()
        self.manual_mode_chkbtn = tk.Checkbutton(
            self.ctrl_frame,
            text="Manual Mode",
            variable=self.manual_mode_intvar,
            onvalue=1,
            offval=0,
            command=self.set_manual_mode,
        )
        self.manual_mode_chkbtn.grid(row=grid_row, column=0)
        grid_row += 1

        self.phase_count_strvar = tk.StringVar()
        tk.Label(self.ctrl_frame, text="PHASE COUNT:").grid(row=grid_row, column=0)
        tk.Entry(self.ctrl_frame, textvariable=self.phase_count_strvar).grid(row=grid_row, column=1)
        tk.Button(self.ctrl_frame, text="SET", command=self.set_phase_count).grid(
            row=grid_row, column=2
        )

        # === DATA FRAME ===
        grid_row = 0
        # VOUT MEASURED
        self.vout_strvar = tk.StringVar()
        tk.Label(self.data_frame, text="VOUT MEASURED:").grid(row=grid_row, column=0)
        tk.Entry(self.data_frame, textvariable=self.vout_strvar).grid(row=grid_row, column=1)
        grid_row += 1

        # IOUT MEASURED
        self.iout_strvar = tk.StringVar()
        tk.Label(self.data_frame, text="IOUT MEASURED:").grid(row=grid_row, column=0)
        tk.Entry(self.data_frame, textvariable=self.iout_strvar).grid(row=grid_row, column=1)
        grid_row += 1

        # VIN MEASURED
        self.vin_strvar = tk.StringVar()
        tk.Label(self.data_frame, text="VIN MEASURED:").grid(row=grid_row, column=0)
        tk.Entry(self.data_frame, textvariable=self.vin_strvar).grid(row=grid_row, column=1)
        grid_row += 1

        # IIN MEASURED
        self.iin_strvar = tk.StringVar()
        tk.Label(self.data_frame, text="IIN MEASURED:").grid(row=grid_row, column=0)
        tk.Entry(self.data_frame, textvariable=self.iin_strvar).grid(row=grid_row, column=1)
        grid_row += 1

        # AUX 5P0 VOLTAGE
        self.aux_5p0_v_strvar = tk.StringVar()
        tk.Label(self.data_frame, text="AUX. 5V VOLTAGE:").grid(row=grid_row, column=0)
        tk.Entry(self.data_frame, textvariable=self.aux_5p0_v_strvar).grid(row=grid_row, column=1)
        grid_row += 1

        # AUX 5P0 CURRENT
        self.aux_5p0_i_strvar = tk.StringVar()
        tk.Label(self.data_frame, text="AUX. 5V CURRENT:").grid(row=grid_row, column=0)
        tk.Entry(self.data_frame, textvariable=self.aux_5p0_i_strvar).grid(row=grid_row, column=1)
        grid_row += 1

        # AUX 12P0 VOLTAGE
        self.aux_12p0_v_strvar = tk.StringVar()
        tk.Label(self.data_frame, text="AUX. 12V VOLTAGE:").grid(row=grid_row, column=0)
        tk.Entry(self.data_frame, textvariable=self.aux_12p0_v_strvar).grid(row=grid_row, column=1)
        grid_row += 1

        # AUX 12P0 CURRENT
        self.aux_12p0_i_strvar = tk.StringVar()
        tk.Label(self.data_frame, text="AUX. 12V CURRENT:").grid(row=grid_row, column=0)
        tk.Entry(self.data_frame, textvariable=self.aux_12p0_i_strvar).grid(row=grid_row, column=1)
        grid_row += 1

        # === PHASE FRAME ===
        self.phase_present_labels = []
        self.phase_enabled_labels = []
        self.phase_current_strvars = [tk.StringVar() for i in range(6)]
        self.phase_duty_cycle_strvars = [tk.StringVar() for i in range(6)]
        self.phase_temp_strvars = [tk.StringVar() for i in range(6)]
        self.phase_frames = [tk.LabelFrame(self.phase_frame) for i in range(6)]

        grid_col = 0
        mapping = ["A", "B", "C", "D", "E", "F"]
        for frame, i_var, duty_var, temp_var, ph_letter in zip(
            self.phase_frames,
            self.phase_current_strvars,
            self.phase_duty_cycle_strvars,
            self.phase_temp_strvars,
            mapping,
        ):
            frame["text"] = f"PHASE {ph_letter}"
            row = 0
            present_label = tk.Label(frame, text="ABSENT", fg="gray")
            present_label.grid(row=row, column=0)
            self.phase_present_labels.append(present_label)
            row += 1
            enabled_label = tk.Label(frame, text="DISABLED", fg="gray")
            enabled_label.grid(row=row, column=0)
            self.phase_enabled_labels.append(enabled_label)
            row += 1
            tk.Label(frame, text="I:").grid(row=row, column=0)
            tk.Label(frame, textvariable=i_var).grid(row=row, column=1)
            row += 1
            tk.Label(frame, text="Duty Cycle:").grid(row=row, column=0)
            tk.Label(frame, textvariable=duty_var).grid(row=row, column=1)
            row += 1
            tk.Label(frame, text="Temp:").grid(row=row, column=0)
            tk.Label(frame, textvariable=temp_var).grid(row=row, column=1)

            frame.grid(row=0, column=grid_col)
            grid_col += 1

        self.ctrl_frame.grid(row=0, column=0)
        self.data_frame.grid(row=1, column=0)
        self.phase_frame.grid(row=2, column=0)

    def core_telem_callback(self, msg):
        self.output_state_strvar.set("ENABLED" if msg.output_enabled else "DISABLED")
        self.vout_strvar.set("{:.2f} V".format(msg.vout_measured))
        self.iout_strvar.set("{:.2f} A".format(msg.iout_measured))

        for ph in range(6):
            if msg.phase_present[ph]:
                self.phase_present_labels[ph]["text"] = "PRESENT"
                self.phase_present_labels[ph]["fg"] = "green"
                self.phase_current_strvars[ph].set("{:.2f} A".format(msg.phase_current[ph]))
                self.phase_duty_cycle_strvars[ph].set("{:.3f}".format(msg.phase_duty_cycle[ph]))
                self.phase_temp_strvars[ph].set("{:.2f} °C".format(msg.phase_temp[ph]))

            else:
                self.phase_present_labels[ph]["text"] = "ABSENT"
                self.phase_present_labels[ph]["fg"] = "gray"
                self.phase_current_strvars[ph].set("--- A")
                self.phase_duty_cycle_strvars[ph].set("---")
                self.phase_temp_strvars[ph].set("--- °C")

            if msg.phase_enabled[ph]:
                self.phase_enabled_labels[ph]["text"] = "ENABLED"
                self.phase_enabled_labels[ph]["fg"] = "green"
            else:
                self.phase_enabled_labels[ph]["text"] = "DISABLED"
                self.phase_enabled_labels[ph]["fg"] = "gray"

    def aux_telem_callback(self, msg):
        self.vin_strvar.set(f"{msg.vin_voltage:.2f} V")
        self.iin_strvar.set(f"{msg.vin_current:.2f} A")
        self.aux_5p0_v_strvar.set(f"{msg.v5p0_voltage:.2f} V")
        self.aux_5p0_i_strvar.set(f"{msg.v5p0_current:.2f} A")
        self.aux_12p0_v_strvar.set(f"{msg.v12p0_voltage:.2f} V")
        self.aux_12p0_i_strvar.set(f"{msg.v12p0_current:.2f} A")

    def toggle_output_state(self):
        pass

    def set_vout_setpt(self):
        try:
            req = SetFloat32.Request()
            req.setpoint = float(self.vout_setpt_strvar.get())
            self.futures["vout_setpt"] = self.set_vout_cli.call_async(req)

        except ValueError:
            self.get_logger().warn("set_vout_setpt was unsuccessful.")

    def set_manual_mode(self):
        try:
            req = SetBool.Request()
            req.data = bool(self.manual_mode_intvar.get())
            self.futures["manual_mode"] = self.set_manual_mode_cli.call_async(req)

        except ValueError:
            self.get_logger().warn("set_manual_mode was unsuccessful.")

    def set_iout_limit(self):
        pass

    def set_phase_count(self):
        try:
            req = SetFloat32.Request()
            req.setpoint = float(self.phase_count_strvar.get())
            self.futures["phase_count"] = self.set_phase_count_cli.call_async(req)

        except ValueError:
            self.get_logger().warn("set_phase_count was unsuccessful.")

    def gui_update_callback(self):
        self.w.update_idletasks()
        self.w.update()


def main(args=None):
    rclpy.init(args=args)

    mmpsu_pane = MmpsuDebugPane()

    rclpy.spin(mmpsu_pane)

    mmpsu_pane.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
