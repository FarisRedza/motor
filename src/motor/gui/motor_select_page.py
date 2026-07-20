import typing

import gi
gi.require_version('Gtk', '4.0')
gi.require_version('Adw', '1')
from gi.repository import Gtk, Adw, Gio

from motor.base_motor import Motor
from motor import dummy_motor, k10cr2_motor, thorlabs_motor, standa_motor


class MotorSelectPage(Adw.NavigationPage):
    def __init__(
            self,
            on_motor_selected: typing.Callable
    ) -> None:
        super().__init__(title='Select Motor')
        self.on_motor_selected = on_motor_selected

        self._build_ui()

    def on_connect_local_motor(
            self,
            button: Gtk.Button,
            motor: tuple[str, str]
    ) -> None:
        selected_motor = self.get_selected_motor(motor=motor)
        self.on_motor_selected(motor=selected_motor)

    def get_selected_motor(self, motor: tuple[str, str]) -> Motor:
        sn = motor[0]
        model = motor[1]

        match model:
            case 'Dummy Motor':
                return dummy_motor.DummyMotor()

            case 'Kinesis K10CR1 Rotary Stage':
                return thorlabs_motor.ThorlabsMotor(
                    serial_number=sn,
                )
            
            case 'Kinesis K10CR2 Rotary Stage':
                return k10cr2_motor.K10CR2Motor(
                    serial_number=sn
                )
            
            case 'XIMC':
                return standa_motor.StandaMotor(
                    serial_number=sn
                )

            case _:
                raise ValueError(f'Unknown motor type: {model}')

    def _build_ui(self) -> None:
        main_box = Gtk.Box(
            orientation=Gtk.Orientation.VERTICAL
        )
        self.set_child(child=main_box)

        header_bar = Adw.HeaderBar()
        main_box.append(child=header_bar)

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

        margin_v = 24
        margin_h = 18
        content = Gtk.Box(
            orientation=Gtk.Orientation.VERTICAL,
            spacing=margin_v,
            margin_top=margin_v,
            margin_bottom=margin_v,
            margin_start=margin_h,
            margin_end=margin_h,
        )
        main_box.append(child=content)

        self._create_local_devices_group(content=content)
        self._create_remote_connection_group(content=content)

    def _create_local_devices_group(self, content: Gtk.Box) -> None:
        group = Adw.PreferencesGroup(
            title='Local Devices'
        )
        content.append(child=group)
        
        local_devices = []

        local_devices.extend(dummy_motor.list_dummy_motors())

        if thorlabs_motor.is_available():
            local_devices.extend(thorlabs_motor.list_kinesis_motors())

        if k10cr2_motor.is_available():
            local_devices.extend(k10cr2_motor.list_k10cr2_motors())

        if standa_motor.is_available():
            local_devices.extend(standa_motor.list_standa_motors())


        for device in local_devices:
            device_row = Adw.ActionRow(
                title=device[1],
                subtitle=f'Serial number: {device[0]}'
            )
            group.add(child=device_row)

            connect_device_button = Gtk.Button(
                label='Connect',
                icon_name='carousel-arrow-next-symbolic',
                css_classes=['flat'],
                valign=Gtk.Align.CENTER
            )
            connect_device_button.connect(
                'clicked',
                lambda button,
                d=device: self.on_connect_local_motor(
                    button=button,
                    motor=d
                )
            )
            device_row.add_suffix(widget=connect_device_button)
            device_row.set_activatable_widget(
                widget=connect_device_button
            )

    def _create_remote_connection_group(self, content: Gtk.Box) -> None:
        group = Adw.PreferencesGroup(
            title='Remote Connection'
        )
        content.append(child=group)

        connect_button = Gtk.Button(
            label='Connect',
            valign=Gtk.Align.CENTER
        )
        group.set_header_suffix(suffix=connect_button)

        host_row = Adw.ActionRow(title='Host')
        group.add(child=host_row)
        host_entry = Gtk.Entry(
            text='127.0.0.1',
            valign=Gtk.Align.CENTER
        )
        host_row.add_suffix(widget=host_entry)

        port_row = Adw.ActionRow(title='Port')
        group.add(child=port_row)
        port_entry = Gtk.Entry(
            text='5002',
            valign=Gtk.Align.CENTER
        )
        port_row.add_suffix(widget=port_entry)