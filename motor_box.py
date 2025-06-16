import sys
import os
import typing

import gi
gi.require_version('Gtk', '4.0')
gi.require_version('Adw', '1')
from gi.repository import Gtk, Adw, GLib, GObject

sys.path.append(
    os.path.abspath(os.path.join(
        os.path.dirname(__file__),
        os.path.pardir
    ))
)
import motor.base_motor as base_motor

class MotorControls(Adw.PreferencesGroup):
    def __init__(
            self,
            motor: base_motor.Motor,
            get_position_callback: typing.Callable,
            get_direction_callback: typing.Callable,
            set_direction_callback: typing.Callable,
            get_step_size_callback: typing.Callable,
            set_step_size_callback: typing.Callable,
            get_acceleration_callback: typing.Callable,
            set_acceleration_callback: typing.Callable,
            get_max_velocity_callback: typing.Callable,
            set_max_velocity_callback: typing.Callable
    ) -> None:
        super().__init__(title='Motor Controls')
        self.get_position_callback = get_position_callback
        self.get_direction_callback = get_direction_callback
        self.set_direction_callback = set_direction_callback
        self.get_step_size_callback = get_step_size_callback
        self.set_step_size_callback = set_step_size_callback
        self.get_acceleration_callback = get_acceleration_callback
        self.set_acceleration_callback = set_acceleration_callback
        self.get_max_velocity_callback = get_max_velocity_callback
        self.set_max_velocity_callback = set_max_velocity_callback

        self.motor = motor
        self.manual_motor_control = False

        # enable controls
        enable_controls_row = Adw.ActionRow(title='Enable motor controls')
        self.add(child=enable_controls_row)

        self.enable_controls_switch = Gtk.Switch(valign=Gtk.Align.CENTER)
        self.enable_controls_switch.connect('notify::active', self.on_enable_controls)
        enable_controls_row.add_suffix(widget=self.enable_controls_switch)
        enable_controls_row.set_activatable_widget(widget=self.enable_controls_switch)

        # position
        position_row = Adw.ActionRow(title='Position')
        self.add(child=position_row)

        self.position_label = Gtk.Label(label=f'{self.get_position_callback():.3f}')
        position_row.add_suffix(widget=self.position_label)

        # direction
        direction_row = Adw.ActionRow(title='Direction')
        self.add(child=direction_row)

        self.direction_label = Gtk.Label(label=self.get_direction_callback().name)
        direction_row.add_suffix(widget=self.direction_label)

        # step size
        step_size_row = Adw.ActionRow(title='Step size')
        self.add(child=step_size_row)

        self.step_size_entry = Gtk.Entry(
            text=self.get_step_size_callback(),
            valign=Gtk.Align.CENTER,
            sensitive=False
        )
        step_size_row.add_suffix(widget=self.step_size_entry)
        self.step_size_entry.connect('activate', self.on_set_step_size)

        # acceleration
        acceleration_row = Adw.ActionRow(title='Acceleration')
        self.add(child=acceleration_row)

        self.acceleration_entry = Gtk.Entry(
            text=get_acceleration_callback(),
            placeholder_text = 'Max value: 20',
            valign=Gtk.Align.CENTER,
            sensitive=False
        )
        acceleration_row.add_suffix(widget=self.acceleration_entry)
        self.acceleration_entry.connect('activate', self.on_set_acceleration)

        # max velocity
        max_velocity_row = Adw.ActionRow(title='Max velocity')
        self.add(child=max_velocity_row)

        self.max_velocity_entry = Gtk.Entry(
            text=self.get_max_velocity_callback(),
            placeholder_text = 'Max value: 25',
            valign=Gtk.Align.CENTER,
            sensitive=False
        )
        max_velocity_row.add_suffix(widget=self.max_velocity_entry)
        self.max_velocity_entry.connect('activate', self.on_set_max_velocity)

        # rotation row
        rotation_row = Adw.ActionRow()
        self.add(child=rotation_row)

        ## control box
        control_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
        control_box.set_valign(Gtk.Align.CENTER)
        control_box.set_halign(Gtk.Align.CENTER)
        control_box.set_hexpand(True)
        rotation_row.set_child(child=control_box)

        ### control grid
        control_grid = Gtk.Grid(
            column_spacing=6,
            row_spacing=6
        )
        control_box.append(child=control_grid)

        #### jog ccw
        self.jog_ccw_button = Gtk.Button(
            label='Jog CCW',
            valign=Gtk.Align.CENTER,
            sensitive=False
        )
        control_grid.attach(
            child=self.jog_ccw_button,
            column=1,
            row=1,
            width=1,
            height=1
        )
        self.jog_ccw_button.connect('clicked', self.on_jog_motor_ccw)

        #### ccw
        self.ccw_button = Gtk.Button(
            label='CCW',
            valign=Gtk.Align.CENTER,
            sensitive=False
        )
        control_grid.attach(
            child=self.ccw_button,
            column=2,
            row=1,
            width=1,
            height=1
        )
        self.ccw_button.connect('clicked', self.on_rotate_motor_ccw)

        #### rotate to 0
        self.rot_0_button = Gtk.Button(
            label='0',
            valign=Gtk.Align.CENTER,
            sensitive=False
        )
        control_grid.attach(
            child=self.rot_0_button,
            column=3,
            row=1,
            width=1,
            height=1
        )
        self.rot_0_button.connect('clicked', self.on_rotate_motor_0)

        #### cw
        self.cw_button = Gtk.Button(
            label='CW',
            valign=Gtk.Align.CENTER,
            sensitive=False
        )
        control_grid.attach(
            child=self.cw_button,
            column=4,
            row=1,
            width=1,
            height=1
        )
        self.cw_button.connect('clicked', self.on_rotate_motor_cw)

        #### jog cw
        self.jog_cw_button = Gtk.Button(
            label='Jog CW',
            valign=Gtk.Align.CENTER,
            sensitive=False
        )
        control_grid.attach(
            child=self.jog_cw_button,
            column=5,
            row=1,
            width=1,
            height=1
        )
        self.jog_cw_button.connect('clicked', self.on_jog_motor_cw)

        #### stop
        self.stop_button = Gtk.Button(
            label='Stop',
            valign=Gtk.Align.CENTER,
            sensitive=False
        )
        control_grid.attach(self.stop_button, 1, 2, 5, 1)
        self.stop_button.connect('clicked', self.on_stop_motor)

        GLib.timeout_add(
            interval=100,
            function=self.update_motor_info
        )

    def on_enable_controls(
            self,
            switch: Gtk.Switch,
            gparam: GObject.GParamSpec
    ) -> None:
        self.manual_motor_control = not self.manual_motor_control
        self.step_size_entry.set_sensitive(sensitive=switch.get_active())
        self.acceleration_entry.set_sensitive(sensitive=switch.get_active())
        self.max_velocity_entry.set_sensitive(sensitive=switch.get_active())
        self.ccw_button.set_sensitive(sensitive=switch.get_active())
        self.rot_0_button.set_sensitive(sensitive=switch.get_active())
        self.cw_button.set_sensitive(sensitive=switch.get_active())
        self.jog_ccw_button.set_sensitive(sensitive=switch.get_active())
        self.jog_cw_button.set_sensitive(sensitive=switch.get_active())
        self.stop_button.set_sensitive(sensitive=switch.get_active())

    def update_motor_info(self) -> bool:
        self.position_label.set_text(str=f'{self.get_position_callback():.3f}')
        self.direction_label.set_text(str=self.get_direction_callback().name)
        # self.acceleration_entry.set_text(text=f'{self.get_acceleration_callback():.3f}')
        # self.max_velocity_entry.set_text(text=f'{self.get_max_velocity_callback():.3f}')
        return True
    
    def on_set_step_size(self, entry: Gtk.Entry):
        try:
            value = abs(float(entry.get_text()))
        except:
            print(f'Invalid entry: {entry.get_text()}')
        else:
            self.set_step_size_callback(value=value)

    def on_set_acceleration(self, entry: Gtk.Entry):
        try:
            value = abs(float(entry.get_text()))
        except:
            print(f'Invalid entry: {entry.get_text()}')
        else:
            if value > 20:
                print(f'Invalid entry: {entry.get_text()}')
            else:
                self.set_acceleration_callback(value=value)

    def on_set_max_velocity(self, entry: Gtk.Entry):
        try:
            value = abs(float(entry.get_text()))
        except:
            print(f'Invalid entry: {entry.get_text()}')
        else:
            if value > 25:
                print(f'Invalid entry: {entry.get_text()}')
            else:
                self.set_max_velocity_callback(value=value)

    def on_stop_motor(self, button: Gtk.Button) -> None:
        self.motor.stop()

    def on_rotate_motor_ccw(self, button: Gtk.Button) -> None:
        self.motor.threaded_move_by(
            angle=-self.get_step_size_callback(),
            acceleration=self.get_acceleration_callback(),
            max_velocity=self.get_max_velocity_callback()
        )

    def on_rotate_motor_0(self, button: Gtk.Button) -> None:
        self.motor.threaded_move_to(
            position=0,
            acceleration=self.get_acceleration_callback(),
            max_velocity=self.get_max_velocity_callback()
        )

    def on_rotate_motor_cw(self, button: Gtk.Button) -> None:
        self.motor.threaded_move_by(
            angle=self.get_step_size_callback(),
            acceleration=self.get_acceleration_callback(),
            max_velocity=self.get_max_velocity_callback()
        )

    def on_jog_motor_ccw(self, button: Gtk.Button) -> None:
        self.motor.jog(
            direction=base_motor.MotorDirection.BACKWARD,
            acceleration=self.get_acceleration_callback(),
            max_velocity=self.get_max_velocity_callback()
        )

    def on_jog_motor_cw(self, button: Gtk.Button) -> None:
        self.motor.jog(
            direction=base_motor.MotorDirection.FORWARD,
            acceleration=self.get_acceleration_callback(),
            max_velocity=self.get_max_velocity_callback()
        )


class DeviceInfoGroup(Adw.PreferencesGroup):
    def __init__(
            self,
            get_device_info_callback: typing.Callable
    ) -> None:
        super().__init__()
        self.set_title(title='Motor Info')
        device_info: base_motor.DeviceInfo = get_device_info_callback()

        # serial number
        serial_no_row = Adw.ActionRow(title='Serial number')
        self.add(child=serial_no_row)

        self.serial_no_label = Gtk.Label(label=device_info.serial_number)
        serial_no_row.add_suffix(widget=self.serial_no_label)

        # model number
        model_no_row = Adw.ActionRow(title='Model number')
        self.add(child=model_no_row)

        self.model_no_label = Gtk.Label(label=device_info.model)
        model_no_row.add_suffix(widget=self.model_no_label)

        # firmware
        fw_ver_row = Adw.ActionRow(title='Firmware version')
        self.add(child=fw_ver_row)

        self.fw_ver_label = Gtk.Label(label=device_info.firmware_version)
        fw_ver_row.add_suffix(widget=self.fw_ver_label)

        # device_name
        device_name_row = Adw.ActionRow(title='Device name')
        self.add(child=device_name_row)

        self.device_name_label = Gtk.Label(label=device_info.device_name)
        device_name_row.add_suffix(widget=self.device_name_label)

class MotorControlPage(Adw.PreferencesPage):
    def __init__(
            self,
            # motor: thorlabs_motor.Motor | remote_motor.Motor
            motor: base_motor.Motor
    ) -> None:
        super().__init__()
        self.motor = motor

        self.motor_controls_group = MotorControls(
            motor=motor,
            get_position_callback=self.get_motor_position,
            get_direction_callback=self.get_motor_direction,
            set_direction_callback=self.set_motor_direction,
            get_step_size_callback=self.get_motor_step_size,
            set_step_size_callback=self.set_motor_step_size,
            get_acceleration_callback=self.get_motor_acceleration,
            set_acceleration_callback=self.set_motor_acceleration,
            get_max_velocity_callback=self.get_motor_max_velocity,
            set_max_velocity_callback=self.set_motor_max_velocity
        )
        self.add(self.motor_controls_group)

        device_info_group = DeviceInfoGroup(
            get_device_info_callback=self.get_device_info
        )
        self.add(device_info_group)

    def get_device_info(self) -> base_motor.DeviceInfo:
        return self.motor.device_info
    
    def get_motor_position(self) -> float:
        return self.motor.position
    
    def get_motor_direction(self) -> base_motor.MotorDirection:
        return self.motor.direction

    def set_motor_direction(self, value: base_motor.MotorDirection) -> None:
        self.motor.direction = value

    def get_motor_step_size(self) -> float:
        return self.motor.step_size

    def set_motor_step_size(self, value: float) -> None:
        self.motor.step_size = value

    def get_motor_acceleration(self) -> float:
        return self.motor.acceleration

    def set_motor_acceleration(self, value: float) -> None:
        self.motor.acceleration = value
    
    def get_motor_max_velocity(self) -> float:
        return self.motor.max_velocity

    def set_motor_max_velocity(self, value: float) -> None:
        self.motor.max_velocity = value