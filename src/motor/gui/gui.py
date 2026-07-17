import sys
import typing
import socket

import gi
gi.require_version('Gtk', '4.0')
gi.require_version('Adw', '1')
from gi.repository import Gtk, Adw, Gio

from motor.gui.gui_widget import MotorControlPage

# TODO: need to handle these imports better, should work for now
THORLABS = 0
ELLIPTEC = 0
K10CR2 = 0
STANDA = 0

from motor import remote_motor

try:
    from motor import thorlabs_motor
except:
    pass
else:
    THORLABS = 1

try:
    from motor import elliptec_motor
except:
    pass
else:
    ELLIPTEC = 1

try:
    from motor import k10cr2_motor
except:
    pass
else:
    K10CR2 = 1

try:
    from motor import standa_motor
except:
    pass
else:
    STANDA = 1


class DeviceListGroup(Adw.PreferencesGroup):
    def __init__(
            self,
            title,
            devices: list[tuple[str, str]],
            set_device_callback: typing.Callable,
            remote: bool = False
    ) -> None:
        super().__init__(title=title)
        self.set_device_callback = set_device_callback
        self.remote = remote

        if len(devices) == 0:
            no_devices_row = Adw.ActionRow(
                child=Gtk.Label(
                    label='No devices found',
                    valign=Gtk.Align.CENTER,
                    vexpand=True
                )
            )
            self.add(child=no_devices_row)

        else:
            for d in devices:
                device_row = Adw.ActionRow(
                    title=d[1],
                    subtitle=f'Serial number: {d[0]}'
                )
                self.add(child=device_row)
                connect_device_button = Gtk.Button(
                    label='Connect',
                    icon_name='carousel-arrow-next-symbolic',
                    css_classes=['flat'],
                    valign=Gtk.Align.CENTER
                )
                connect_device_button.connect(
                    'clicked',
                    lambda button,
                    d=d: self.on_connect_device(
                        button=button,
                        device=d
                    )
                )
                device_row.add_suffix(widget=connect_device_button)
                device_row.set_activatable_widget(
                    widget=connect_device_button
                )

    def on_connect_device(self, button: Gtk.Button, device: tuple[str, str]) -> None:
        self.set_device_callback(
            device=device,
            remote=self.remote
        )


class RemoteConnectionGroup(Adw.PreferencesGroup):
    def __init__(
            self,
            set_host_callback: typing.Callable,
            set_port_callback: typing.Callable,
            set_sock_callback: typing.Callable,
            server_connect_callback: typing.Callable
        ) -> None:
        super().__init__(title='Remote Connection')
        self.set_host_callback = set_host_callback
        self.set_port_callback = set_port_callback
        self.set_sock_callback = set_sock_callback
        self.server_connect_callback = server_connect_callback

        # host
        self.host_row = Adw.ActionRow(title='Host')
        self.add(child=self.host_row)
        host_entry = Gtk.Entry(
            text='127.0.0.1',
            valign=Gtk.Align.CENTER
        )
        host_entry.connect(
            'activate',
            self.on_set_host
        )
        self.host_row.add_suffix(
            widget=host_entry
        )
        # port
        self.port_row = Adw.ActionRow(title='Port')
        self.add(child=self.port_row)
        port_entry = Gtk.Entry(
            text='5002',
            valign=Gtk.Align.CENTER
        )
        port_entry.connect(
            'activate',
            self.on_set_port
        )
        self.port_row.add_suffix(
            widget=port_entry
        )

        # connect
        connect_button = Gtk.Button(
            label='Connect',
            valign=Gtk.Align.CENTER
        )
        connect_button.connect(
            'clicked',
            self.on_server_connect
        )
        self.set_header_suffix(suffix=connect_button)

    def on_set_host(self, entry: Gtk.Entry) -> None:
        self.set_host_callback(host=entry.get_text())

    def on_set_port(self, entry: Gtk.Entry) -> None:
        try:    
            port = int(entry.get_text())
        except:
            print(f'Invalid entry: {entry.get_text()}')
        else:
            self.set_port_callback(port=port)

    def on_server_connect(self, button: Gtk.Button) -> None:
        self.server_connect_callback()


class MainWindow(Adw.ApplicationWindow):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.set_title(title='Motor Controller')
        self.set_default_size(width=600, height=500)
        self.set_size_request(width=450, height=150)
        self.connect('close-request', self.on_close_request)

        self.host = '127.0.0.1'
        self.port = 5002
        self._sock: socket.socket | None = None

        # main box
        main_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        self.set_content(content=main_box)

        ## header_bar
        try:
            self.header_bar = Gtk.HeaderBar(
                use_native_controls=True
            )
        except:
            self.header_bar = Gtk.HeaderBar()
        main_box.append(child=self.header_bar)

        self.return_button = Gtk.Button(
            label='Return',
            icon_name='carousel-arrow-previous-symbolic',
        )
        self.return_button.connect(
            'clicked',
            self.unset_device
        )

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
        self.header_bar.pack_end(child=menu_button)

        self.main_stack = Gtk.Stack(
            transition_type=Gtk.StackTransitionType.CROSSFADE
        )
        main_box.append(child=self.main_stack)

        self.device_select_page = Adw.PreferencesPage()
        self.main_stack.add_child(child=self.device_select_page)
        self.main_stack.set_visible_child(child=self.device_select_page)

        local_devices = []
        if THORLABS == 1:
            local_devices.extend(thorlabs_motor.list_thorlabs_motors())
        if ELLIPTEC == 1:
            local_devices.extend(elliptec_motor.list_elliptec_motors())
        if K10CR2 == 1:
            local_devices.extend(k10cr2_motor.list_k10cr2_motors())
        if STANDA == 1:
            local_devices.extend(standa_motor.list_standa_motors())


        local_devices_group = DeviceListGroup(
            title='Local Devices',
            devices=local_devices,
            set_device_callback=self.set_device
        )
        self.device_select_page.add(group=local_devices_group)

        self.remote_connection_group = RemoteConnectionGroup(
            set_host_callback=self.set_host,
            set_port_callback=self.set_port,
            set_sock_callback=self.set_sock,
            server_connect_callback=self.server_connect
        )
        self.device_select_page.add(group=self.remote_connection_group)

    def server_connect(self) -> None:
        sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_STREAM
        )
        sock.settimeout(5)
        sock.connect((self.host, self.port))
        self._sock = sock

        remote_devices = remote_motor.list_thorlabs_motors(
            sock=self._sock
        )

        self.device_select_page.remove(group=self.remote_connection_group)
        self.device_select_page.add(
            group=DeviceListGroup(
                title='Remote Devices',
                devices=remote_devices,
                set_device_callback=self.set_device,
                remote=True
            )
        )

    def set_device(self, device: tuple[str, str], remote: bool = False) -> None:
        if not remote:
            match device[1]:
                case 'Thorlabs' | 'Kinesis K10CR1 Rotary Stage':
                    if THORLABS == 1:
                        self.motor_page = MotorControlPage(
                            motor=thorlabs_motor.ThorlabsMotor(
                                serial_number=device[0]
                            )
                        )
                    else:
                        raise ValueError('Thorlabs motor support is not available. Please install the required dependencies.')
                case 'Kinesis K10CR2 Rotary Stage':
                    if K10CR2 == 1:
                        self.motor_page = MotorControlPage(
                            motor=k10cr2_motor.ThorlabsMotor(
                                serial_number=device[0]
                            )
                        )
                    else:
                        raise ValueError('K10CR2 motor support is not available. Please install the required dependencies.')
                case 'Elliptec':
                    if ELLIPTEC == 1:
                        self.motor_page = MotorControlPage(
                            motor=elliptec_motor.ElliptecMotor(
                                serial_number=device[0]
                            )
                        )
                    else:
                        raise ValueError('Elliptec motor support is not available. Please install the required dependencies.')
                case 'XIMC':
                    if STANDA == 1:
                        self.motor_page = MotorControlPage(
                            motor=standa_motor.StandaMotor(
                                serial_number=device[0]
                            )
                        )
                case _:
                    raise NotImplementedError(f'Unsupported motor: {device[1]}')
        else:
            self.motor_page = MotorControlPage(
                motor=remote_motor.RemoteMotor(
                    serial_number=device[0],
                    sock=self._sock
                )
            )
        self.main_stack.add_child(child=self.motor_page)
        self.main_stack.set_visible_child(child=self.motor_page)
        self.main_stack.remove(child=self.device_select_page)
        self.header_bar.pack_start(child=self.return_button)

    def unset_device(self, button: Gtk.Button) -> None:
        self.motor_page.motor.disconnect()
        self.main_stack.add_child(child=self.device_select_page)
        self.main_stack.set_visible_child(child=self.device_select_page)
        self.main_stack.remove(child=self.motor_page)
        self.header_bar.remove(child=self.return_button)

    def on_close_request(self, window: Adw.ApplicationWindow) -> bool:
        return False
    
    def get_host(self) -> str:
        return self.host
    
    def set_host(self, host: str) -> None:
        self.host = host

    def get_port(self) -> int:
        return self.port

    def set_port(self, port: int) -> None:
        self.port = port

    def set_sock(self, sock: socket.socket) -> None:
        self._sock = sock


class App(Adw.Application):
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.connect('activate', self.on_activate)

        help_action = Gio.SimpleAction.new(
            name='help',
            parameter_type=None
        )
        help_action.connect('activate', self.on_help)
        self.add_action(action=help_action)

        about_action = Gio.SimpleAction.new(
            name='about',
            parameter_type=None
        )
        about_action.connect('activate', self.on_about)
        self.add_action(action=about_action)

    def on_activate(self, app) -> None:
        self.win = MainWindow(application=app)
        self.win.present()

    def on_help(self, action: Gio.SimpleAction, param: None) -> None:
        help_dialog = Gtk.MessageDialog(
            transient_for=self.get_active_window(),
            modal=True,
            visible=True,
            buttons=Gtk.ButtonsType.OK,
            text='Help',
            secondary_text='Select a motor from "Local Devices" or connect to a remote motor server using "Remote Connection", and then click "Enable motor controls" to begin.'
        )
        help_dialog.connect(
            'response',
            lambda dialog, response: dialog.destroy()
        )

    def on_about(self, action: Gio.SimpleAction, param: None) -> None:
        about_dialog = Gtk.AboutDialog(
            transient_for=self.get_active_window(),
            modal=True,
            visible=True,
            program_name='Motor Controller',
            version='0.1',
            logo_icon_name='object-rotate-right-symbolic',
            website='https://github.com/FarisRedza/motor',
            website_label='GitHub',
            authors=['Faris Redza']
        )


def main() -> None:
    app = App(application_id='com.github.FarisRedza.MotorController')
    try:
        app.run(sys.argv)
    except Exception as e:
        print('App crashed with an exception:', e)

if __name__ == '__main__':
    main()