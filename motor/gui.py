import sys
import typing
import socket

import gi
gi.require_version('Gtk', '4.0')
gi.require_version('Adw', '1')
from gi.repository import Gtk, Adw

from . import gui_widget
from . import thorlabs_motor
from . import remote_motor

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
                    valign=Gtk.Align.CENTER
                )
                connect_device_button.connect(
                    'clicked',
                    lambda button,
                    serial_number=d[0]: self.on_connect_device(
                        button=button,
                        serial_number=serial_number
                    )
                )
                device_row.add_suffix(widget=connect_device_button)

    def on_connect_device(self, button: Gtk.Button, serial_number: str) -> None:
        self.set_device_callback(
            serial_number=serial_number,
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
        self.connect_row = Adw.ActionRow()
        self.add(child=self.connect_row)
        connect_button = Gtk.Button(
            label='Connect',
            valign=Gtk.Align.CENTER
        )
        connect_button.connect(
            'clicked',
            self.on_server_connect
        )
        self.connect_row.set_child(
            child=connect_button
        )

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
            header_bar = Gtk.HeaderBar(
                use_native_controls=True
            )
        except:
            header_bar = Gtk.HeaderBar()
        main_box.append(child=header_bar)

        self.main_stack = Gtk.Stack(
            transition_type=Gtk.StackTransitionType.CROSSFADE
        )
        main_box.append(child=self.main_stack)

        self.device_select_page = Adw.PreferencesPage()
        self.main_stack.add_child(child=self.device_select_page)
        self.main_stack.set_visible_child(child=self.device_select_page)

        local_devices = thorlabs_motor.list_thorlabs_motors()
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

    def set_device(self, serial_number: str, remote: bool = False) -> None:
        if not remote:
            self.motor_page = gui_widget.MotorControlPage(
                motor=thorlabs_motor.Motor(
                    serial_number=serial_number
                )
            )
        else:
            self.motor_page = gui_widget.MotorControlPage(
                motor=remote_motor.RemoteMotor(
                    serial_number=serial_number,
                    sock=self._sock
                )
            )
        self.main_stack.add_child(child=self.motor_page)
        self.main_stack.set_visible_child(child=self.motor_page)

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

    def on_activate(self, app) -> None:
        self.win = MainWindow(application=app)
        self.win.present()

if __name__ == '__main__':
    app = App(application_id='com.github.FarisRedza.MotorController')
    try:
        app.run(sys.argv)
    except Exception as e:
        print('App crashed with an exception:', e)