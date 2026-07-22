import gi
gi.require_version('Gtk', '4.0')
gi.require_version('Adw', '1')
from gi.repository import Gtk

from motor.gui import gui

settings = Gtk.Settings.get_default()

if settings is not None:
    settings.set_property(
        'gtk-icon-theme-name',
        'Yaru',
    )
    settings.set_property(
        'gtk-theme-name',
        'Yaru',
    )
    settings.set_property(
        'gtk-font-name',
        'Ubuntu Sans 11'
    )
gui.main()