import typing
import threading

import gi
gi.require_version('Gtk', '4.0')
gi.require_version('Adw', '1')
from gi.repository import Gtk, Adw, GLib, Gio

from motor.base_motor import Motor


GUI_UPDATE_INTERVAL_MS = 50


class MotorPage(Adw.NavigationPage):
    def __init__(
            self,
            motor: Motor,
            on_back_button: typing.Callable
    ) -> None:
        super().__init__(
            title=f'Motor Controller: {motor.device_info.serial_number}'
        )
        self.on_back_button = on_back_button

        self._motor = motor
        self._command_running = False
        self._update_timer_id: typing.Optional[int] = None

        self._build_ui()

        self._update_timer_id = GLib.timeout_add(
            GUI_UPDATE_INTERVAL_MS,
            self._update_display,
        )

    def _disconnect_motor(self) -> None:
        try:
            self._motor.disconnect()
        except Exception as error:
            print(f'Could not disconnect motor: {error}')

    def _update_display(self) -> bool:
        """Copy the cached motor state into the GUI."""
        self.position_label.set_label(
            f'{self._motor.position:.3f}°'
        )

        if self._motor.is_moving:
            self.movement_label.set_label('Moving')
            self.movement_label.remove_css_class(
                'success'
            )
            self.movement_label.add_css_class(
                css_class='accent'
            )
        else:
            self.movement_label.set_label(str='Stopped')
            self.movement_label.remove_css_class(
                css_class='accent'
            )
            self.movement_label.add_css_class(
                css_class='success'
            )

        error = self._motor.tracking_error

        if error is None:
            self.connection_label.set_label(
                str='Connected'
            )
            self.connection_label.remove_css_class(
                css_class='error'
            )
            self.connection_label.add_css_class(
                css_class='success'
            )
        else:
            self.connection_label.set_label(
                str=str(error)
            )
            self.connection_label.remove_css_class(
                css_class='success'
            )
            self.connection_label.add_css_class(
                css_class='error'
            )

        return GLib.SOURCE_CONTINUE

    def _build_ui(self) -> None:
        self.toast_overlay = Adw.ToastOverlay()
        main_box = Gtk.Box(
            orientation=Gtk.Orientation.VERTICAL
        )
        self.set_child(child=main_box)

        header_bar = Adw.HeaderBar()
        main_box.append(child=header_bar)
        window_title = Adw.WindowTitle(
            title='Motor Controller',
            subtitle=f'{self._motor.device_info.serial_number}'
        )
        header_bar.set_title_widget(
            title_widget=window_title
        )

        self.back_button = Gtk.Button(
            label='Back',
            icon_name='carousel-arrow-previous-symbolic',
            tooltip_markup='Return to motor selection'
        )
        self.back_button.connect(
            'clicked',
            self._on_back_button_clicked
        )
        header_bar.pack_start(child=self.back_button)

        # menu button
        menu = Gio.Menu.new()
        menu.append('Help', 'app.help')
        menu.append('About', 'app.about')
        popover = Gtk.PopoverMenu()
        popover.set_menu_model(menu)

        menu_button = Gtk.MenuButton(
            icon_name='open-menu-symbolic',
            popover=popover
        )
        header_bar.pack_end(child=menu_button)

        self.stop_button = Gtk.Button(
            label='Stop',
            tooltip_text='Stop the motor',
        )
        self.stop_button.add_css_class(
            css_class='destructive-action'
        )
        self.stop_button.connect(
            'clicked',
            self._on_stop_clicked,
        )
        header_bar.pack_end(child=self.stop_button)

        scrolled_window = Gtk.ScrolledWindow(
            hscrollbar_policy=Gtk.PolicyType.NEVER,
            vexpand=True,
        )
        main_box.append(child=scrolled_window)

        clamp = Adw.Clamp(
            maximum_size=600,
            tightening_threshold=400,
        )
        scrolled_window.set_child(child=clamp)

        content = Gtk.Box(
            orientation=Gtk.Orientation.VERTICAL,
            spacing=24,
            margin_top=24,
            margin_bottom=24,
            margin_start=18,
            margin_end=18,
        )
        clamp.set_child(child=content)

        self._create_status_group(content=content)
        self._create_relative_movement_group(content=content)
        self._create_absolute_movement_group(content=content)
        self._create_settings_group(content=content)
        self._create_device_info_group(content=content)

    def _create_status_group(
        self,
        content: Gtk.Box,
    ) -> None:
        group = Adw.PreferencesGroup(
            title='Motor Status',
        )

        self.position_row = Adw.ActionRow(
            title='Position',
            subtitle='Current angular position',
        )

        self.position_label = Gtk.Label(
            label='—',
            xalign=1,
        )
        self.position_label.add_css_class(css_class='title-2')
        self.position_label.add_css_class(css_class='numeric')

        self.position_row.add_suffix(
            widget=self.position_label
        )
        group.add(child=self.position_row)

        self.movement_row = Adw.ActionRow(
            title='Movement',
        )

        self.movement_label = Gtk.Label(
            label='Unknown',
            xalign=1,
        )

        self.movement_row.add_suffix(
            widget=self.movement_label
        )
        group.add(child=self.movement_row)

        self.connection_row = Adw.ActionRow(
            title='Tracking',
        )

        self.connection_label = Gtk.Label(
            label='Connected',
            xalign=1,
        )

        self.connection_row.add_suffix(
            widget=self.connection_label
        )
        group.add(child=self.connection_row)

        content.append(child=group)

    def _create_relative_movement_group(
        self,
        content: Gtk.Box,
    ) -> None:
        group = Adw.PreferencesGroup(
            title='Relative Movement',
            description='Rotate relative to the current position'
        )

        self.relative_angle_row = Adw.SpinRow.new_with_range(
            0.01,
            360.0,
            0.1,
        )
        self.relative_angle_row.set_title(
            title='Angle'
        )
        self.relative_angle_row.set_subtitle(
            subtitle='Degrees'
        )
        self.relative_angle_row.set_digits(digits=2)
        self.relative_angle_row.set_value(value=15.0)

        group.add(child=self.relative_angle_row)

        button_box = Gtk.Box(
            orientation=Gtk.Orientation.HORIZONTAL,
            spacing=12,
            homogeneous=True,
            margin_top=12,
        )

        anticlockwise_button = Gtk.Button(
            label='Anticlockwise',
            icon_name='object-rotate-left-symbolic',
        )
        anticlockwise_button.connect(
            'clicked',
            self._on_relative_move_clicked,
            -1.0,
        )

        clockwise_button = Gtk.Button(
            label='Clockwise',
            icon_name='object-rotate-right-symbolic',
        )
        clockwise_button.add_css_class(
            css_class='suggested-action'
        )
        clockwise_button.connect(
            'clicked',
            self._on_relative_move_clicked,
            1.0,
        )

        button_box.append(child=anticlockwise_button)
        button_box.append(child=clockwise_button)

        group.add(child=button_box)
        content.append(child=group)

    def _create_absolute_movement_group(
        self,
        content: Gtk.Box,
    ) -> None:
        group = Adw.PreferencesGroup(
            title='Absolute Movement',
            description='Move to an absolute angular position',
        )

        self.target_position_row = Adw.SpinRow.new_with_range(
            -3600.0,
            3600.0,
            0.1,
        )
        self.target_position_row.set_title(
            title='Target Position'
        )
        self.target_position_row.set_subtitle(
            subtitle='Degrees'
        )
        self.target_position_row.set_digits(digits=2)
        self.target_position_row.set_value(value=0)

        group.add(child=self.target_position_row)

        move_button = Gtk.Button(
            label='Move to Position',
            halign=Gtk.Align.END,
            margin_top=12,
        )
        move_button.add_css_class(
            css_class='suggested-action'
        )
        move_button.connect(
            'clicked',
            self._on_move_to_clicked,
        )

        group.add(child=move_button)
        content.append(child=group)

    def _create_settings_group(
        self,
        content: Gtk.Box,
    ) -> None:
        group = Adw.PreferencesGroup(
            title='Movement Settings',
        )

        self.acceleration_row = Adw.SpinRow.new_with_range(
            0.01,
            1000.0,
            0.1,
        )
        self.acceleration_row.set_title(
            title='Acceleration'
        )
        self.acceleration_row.set_subtitle(
            subtitle='Degrees/s²'
        )
        self.acceleration_row.set_digits(digits=2)
        self.acceleration_row.set_value(
            value=self._motor.acceleration
        )

        group.add(child=self.acceleration_row)

        self.velocity_row = Adw.SpinRow.new_with_range(
            0.01,
            1000.0,
            0.1,
        )
        self.velocity_row.set_title(
            title='Maximum Velocity'
        )
        self.velocity_row.set_subtitle(
            subtitle='Degrees/s'
        )
        self.velocity_row.set_digits(digits=2)
        self.velocity_row.set_value(
            value=self._motor.max_velocity
        )

        group.add(child=self.velocity_row)

        apply_button = Gtk.Button(
            label='Apply settings',
            halign=Gtk.Align.END,
            margin_top=12,
        )
        apply_button.connect(
            'clicked',
            self._on_apply_settings_clicked,
        )

        group.add(child=apply_button)
        content.append(child=group)
    
    def _create_device_info_group(
            self,
            content: Gtk.Box
    ) -> None:
        group = Adw.PreferencesGroup(
            title='Device Information',
        )

        self.device_name_row = Adw.ActionRow(
            title='Name'
        )
        self.device_name_label = Gtk.Label(
            label=self._motor.device_info.device_name
        )
        self.device_name_row.add_suffix(widget=self.device_name_label)
        group.add(child=self.device_name_row)

        self.model_row = Adw.ActionRow(
            title='Model'
        )
        self.model_label = Gtk.Label(
            label=self._motor.device_info.model
        )
        self.model_row.add_suffix(widget=self.model_label)
        group.add(child=self.model_row)

        self.serial_number_row = Adw.ActionRow(
            title='Serial Number'
        )
        self.serial_number_label = Gtk.Label(
            label=self._motor.device_info.serial_number
        )
        self.serial_number_row.add_suffix(widget=self.serial_number_label)
        group.add(child=self.serial_number_row)

        self.firmware_version_row = Adw.ActionRow(
            title='Firmware Version'
        )
        self.firmware_version_label = Gtk.Label(
            label=self._motor.device_info.device_name
        )
        self.firmware_version_row.add_suffix(widget=self.firmware_version_label)
        group.add(child=self.firmware_version_row)

        content.append(child=group)

    def _show_toast(
        self,
        message: str,
    ) -> None:
        self.toast_overlay.add_toast(
            toast=Adw.Toast(
                title=message,
                timeout=3,
            )
        )

    def _run_motor_command(
        self,
        command: typing.Callable[[], None],
        *,
        success_message: typing.Optional[str] = None,
    ) -> None:
        """Run a synchronous motor command outside GTK's thread."""
        if self._command_running:
            self._show_toast(
                message='Another motor command is being submitted'
            )
            return

        self._command_running = True

        def worker() -> None:
            try:
                command()
            except Exception as error:
                GLib.idle_add(
                    self._command_failed,
                    error,
                )
            else:
                GLib.idle_add(
                    self._command_finished,
                    success_message,
                )

        threading.Thread(
            target=worker,
            name='motor-command',
            daemon=True,
        ).start()

    def _command_finished(
        self,
        message: typing.Optional[str],
    ) -> bool:
        self._command_running = False

        if message is not None:
            self._show_toast(message=message)

        return GLib.SOURCE_REMOVE

    def _command_failed(
        self,
        error: Exception,
    ) -> bool:
        self._command_running = False

        self._show_toast(
            message=f'Motor command failed: {error}'
        )

        return GLib.SOURCE_REMOVE

    def _on_back_button_clicked(self, button: Gtk.Button) -> None:
        self._disconnect_motor()
        self.on_back_button()

    def _on_stop_clicked(
        self,
        _button: Gtk.Button,
    ) -> None:
        self._run_motor_command(
            self._motor.stop,
        )

    def _on_relative_move_clicked(
        self,
        _button: Gtk.Button,
        direction: float,
    ) -> None:
        angle = (
            self.relative_angle_row.get_value()
            * direction
        )

        self._run_motor_command(
            lambda: self._motor.move_by(
                angle=angle,
            )
        )

    def _on_move_to_clicked(
        self,
        _button: Gtk.Button,
    ) -> None:
        target = self.target_position_row.get_value()

        acceleration = self.acceleration_row.get_value()
        max_velocity = self.velocity_row.get_value()

        self._run_motor_command(
            lambda: self._motor.move_to(
                position=target,
                acceleration=acceleration,
                max_velocity=max_velocity,
            )
        )

    def _on_apply_settings_clicked(
        self,
        _button: Gtk.Button,
    ) -> None:
        acceleration = self.acceleration_row.get_value()
        max_velocity = self.velocity_row.get_value()

        self._run_motor_command(
            lambda: self._motor.update_settings(
                acceleration=acceleration,
                max_velocity=max_velocity,
            ),
            success_message='Settings applied',
        )