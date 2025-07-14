import sys

import gi
gi.require_version('Gtk', '4.0')
gi.require_version('Adw', '1')
from gi.repository import Gtk, Adw

from . import gui_widget
from . import remote_motor

class MainWindow(Adw.ApplicationWindow):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.set_title(title='Motor Controller')
        self.set_default_size(width=400, height=350)
        self.set_size_request(width=400, height=350)
        self.connect('close-request', self.on_close_request)

        # main box
        main_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        self.set_content(content=main_box)

        ## header_bar
        header_bar = Adw.HeaderBar()
        main_box.append(child=header_bar)

        self.main_stack = Gtk.Stack(
            transition_type=Gtk.StackTransitionType.CROSSFADE
        )
        main_box.append(child=self.main_stack)

        motors = remote_motor.list_motors(
            host=remote_motor.server_host,
            port=remote_motor.server_port
        )
        if len(motors) == 0:
            main_box.append(
                child=Gtk.Label(
                    label='No motors found',
                    valign=Gtk.Align.CENTER,
                    vexpand=True
                )
            )
        else:
            ## add motor
            add_motor_page = Adw.PreferencesPage()
            self.main_stack.add_titled(
                child=add_motor_page,
                name='add motor',
                title='add motor'
            )
            add_motor_group = Adw.PreferencesGroup(title='Motors')
            add_motor_page.add(add_motor_group)
            for i in motors:
                add_motor_row = Adw.ActionRow(
                    title=i.device_info.device_name,
                    subtitle=i.device_info.serial_number
                )
                add_motor_group.add(add_motor_row)
                add_motor_button = Gtk.Button(
                    label='Connect',
                    valign=Gtk.Align.CENTER
                )
                add_motor_button.connect(
                    'clicked',
                    lambda widget,
                    serial_number=i.device_info.serial_number: self.on_add_motor(
                        widget,
                        serial_number
                    )
                )
                add_motor_row.add_suffix(widget=add_motor_button)

        ## add motor
        add_motor_button = Gtk.Button(
            label='Add motor',
            valign=Gtk.Align.CENTER,
            halign=Gtk.Align.CENTER
        )
        add_motor_button.connect('clicked', self.on_add_motor)

    def on_add_motor(self, button: Gtk.Button, serial_number) -> None:
        self.motor_control_box = gui_widget.MotorControlPage(
            motor=remote_motor.Motor(
                host=remote_motor.server_host,
                port=remote_motor.server_port,
                serial_number=serial_number
            )
        )
        self.main_stack.add_titled(
            child=self.motor_control_box,
            name='Test',
            title='test'
        )
        self.main_stack.set_visible_child(child=self.motor_control_box)
        pass

    def on_close_request(self, window: Adw.ApplicationWindow) -> bool:
        return False

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