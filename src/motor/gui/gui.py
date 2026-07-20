import typing
import threading

import gi
gi.require_version('Gtk', '4.0')
gi.require_version('Adw', '1')
from gi.repository import Gtk, Adw, Gio, GLib

from motor.gui.motor_select_page import MotorSelectPage
from motor.gui.motor_page import MotorPage


class MainWindow(Adw.ApplicationWindow):
    def __init__(self,*args,**kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.set_title(title='Motor Controller')
        self.set_default_size(width=600, height=500)
        self.set_size_request(width=450, height=150)

        self.stack = Gtk.Stack()
        self.set_content(content=self.stack)

        self.motor_select_page = MotorSelectPage(
            on_motor_selected=self.open_motor
        )
        self.stack.add_named(
            child=self.motor_select_page,
            name='motor-selection'
        )

        self.connect(
            'close-request',
            self._on_close_request,
        )
    
    def open_motor(self, motor) -> None:
        self.motor_page = MotorPage(
            motor=motor,
            on_back_button=self.show_device_selection
        )
        
        self.stack.add_named(
            child=self.motor_page,
            name='motor-controller'
        )

        self.stack.set_visible_child_name(name='motor-controller')

    def show_device_selection(self) -> None:
        self.stack.set_visible_child_name('motor-selection')
        self.stack.remove(child=self.motor_page)

    def _on_close_request(
        self,
        _window: Adw.ApplicationWindow,
    ) -> bool:
        if self.motor_page:
            if self.motor_page._update_timer_id is not None:
                GLib.source_remove(
                    tag=self.motor_page._update_timer_id
                )
                self.motor_page._update_timer_id = None

            # disconnect() may wait for the tracking thread, so do it
            # outside GTK's main thread.
            threading.Thread(
                target=self.motor_page._disconnect_motor,
                name='motor-disconnect',
                daemon=True,
            ).start()

        return False


class App(Adw.Application):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._window: typing.Optional[MainWindow] = None

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

    def do_activate(self) -> None:
        if self._window is None:
            self._window = MainWindow(
                application=self
            )
        
        self._window.present()

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


def main() -> int:
    app = App(application_id='com.github.FarisRedza.Motor')
    return app.run(None)


if __name__ == '__main__':
    raise SystemExit(
        main()
    )