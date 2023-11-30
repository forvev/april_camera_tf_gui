import os
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from TfPublisherGui import TfPublisherGui
from TfPublisher import TfPublisher

class MyPlugin(Plugin):
    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource', 'artur2.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)

        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        context.add_widget(self._widget)

        self.context = context
        self.tfp = TfPublisher()
        
        # self.element_map = {}
        # names_gui = ["x", "y", "z", "roll", "pitch", "yaw"]
        # for loop_name in names_gui:
        #     self.element_map[loop_name] = {
        #         "slidervalue": 0,
        #         "display": getattr(self._widget, f'{loop_name}_field'),
        #         "element": self.elements[loop_name],
        #     }
            
        #     # plus buttons
        #     button = getattr(self._widget, f'{loop_name}_plus')
        #     button.clicked.connect(self.on_plus_btn_1_clicked)

        #     button = getattr(self._widget, f'{loop_name}_minus')
        #     button.clicked.connect(self.on_minus_btn_1_clicked)

        # self._widget.comboBox_parent.currentTextChanged.connect(self.on_comboBox_parent_activation)
        # self._widget.zero_button.clicked.connect(self.on_zero_click)
        # self._widget.load_default_button.clicked.connect(self.on_default_click)
        # self._widget.x_plus.clicked.connect(self.on_default_click)
        TfPublisherGui(self)#tfp, self._widget)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass
    
    def on_default_click(self):
        print("fsdfdsfds")

