
import rospy
from math import pi
from python_qt_binding.QtWidgets import QWidget, QComboBox
from qt_gui.plugin import Plugin
#from my_module import MyPlugin

RANGE = 10000

class TfPublisherGui(Plugin):
    def __init__(self, my_module):
        super(TfPublisherGui, self).__init__(my_module.context)
        rospy.loginfo("start!")
        self.tfp = my_module.tfp
        self.element_map = {}
        self._widget = my_module._widget

        links = self.tfp.parent_links
        self._widget.comboBox_parent.addItems(list(links.keys()))
        self._widget.comboBox_child.addItems([])
    
        if self.tfp.parent_frame in links.keys():
            # idk how it works
            #self.parent_link.SetSelection(self.parent_link.FindString(tfp.parent_frame))
            clinks = links[self.tfp.parent_frame]
            #self.child_link.AppendItems(clinks)
            self._widget.comboBox_child.addItems(clinks)
            # if tfp.child_frame in clinks:
            #     self.child_link.SetSelection(
            #         self.child_link.FindString(tfp.child_frame)
            #     )
            # else:
            #     self.child_link.SetSelection(0)

        else:
            #self.child_link.AppendItems(links[list(links.keys())[0]])
            self._widget.comboBox_child.addItems(links[list(links.keys())[0]])
            # self.child_link.SetSelection(0)
            # self.parent_link.SetSelection(0)
        ### Sliders ###
        for name in self.tfp.element_list:
            element = self.tfp.elements[name]
            if element["min"] == element["max"]:
                continue
            
            self.element_map[name] = {
                "slidervalue": 0,
                "display": getattr(self._widget, f'{name}_field'),
                "element": element,
            }
            
            button = getattr(self._widget, f'{name}_plus')
            button.clicked.connect(self.on_plus_btn_1_clicked)

            button = getattr(self._widget, f'{name}_minus')
            button.clicked.connect(self.on_minus_btn_1_clicked)

        self._widget.comboBox_parent.currentTextChanged.connect(self.on_comboBox_parent_activation)
        self._widget.zero_button.clicked.connect(self.on_zero_click)
        self._widget.load_default_button.clicked.connect(self.on_default_click)

        self.load()
        self.update_values()

    def on_plus_btn_1_clicked(self):
        rospy.loginfo("clicked plus!")
        sending_button = self.sender().objectName()
        self.inc_dec_value(sending_button, 1.0)
    
    def on_minus_btn_1_clicked(self):
        rospy.loginfo("clicked minus!")
        sending_button = self.sender().objectName()
        self.inc_dec_value(sending_button, -1.0)

    def on_comboBox_parent_activation(self):
        print("comboBox parent activated!")
        parentIndex = self._widget.comboBox_parent.currentIndex()
        default_parent = list(self.tfp.parent_links.keys())[parentIndex]
        self._widget.comboBox_child.clear()
        self._widget.comboBox_child.addItems(self.tfp.parent_links[default_parent])

    def on_zero_click(self) -> None:
        rospy.loginfo("Center Event")
        self.center()

    def on_default_click(self) -> None:
        rospy.loginfo("Load Event")
        self.load()

    def update_values(self):
        for name, element_info in self.element_map.items():
            purevalue = element_info["slidervalue"]
            element = element_info["element"]
            value = self.sliderToValue(purevalue, element)
            element["value"] = value
            # element_info['slider'].SetValue(purevalue)
            element_info["display"].setText("%.2f" % value)
            if name in ["roll", "pitch", "yaw"]:
                deg = value / pi * 180
                element_info["display"].setText("%3d(%.2f)" % (deg, value))

    def inc_dec_value(self, name, sign):
        name = name.split("_")[0]
        offset = 0.01  # xyz 10mm
        if name in ["roll", "pitch", "yaw"]:
            offset = pi * 0.0025  # rpy 0.45 deg
        element_info = self.element_map[name]
        element = element_info["element"]

        newValue = element["value"] + offset * sign
        # round over angle from range to -pi ~ pi
        if name in ["roll", "pitch", "yaw"]:
            if newValue > pi:
                newValue -= 2 * pi
            if newValue < -pi:
                newValue += 2 * pi
        element_info["slidervalue"] = self.valueToSlider(newValue, element)
        self.update_values()

    def load_link(self, parent, child):
        self.tfp.load_link(parent, child)
        self.load()
        self.tfp.parent_frame = parent
        self.tfp.child_frame = child

    def choice_event(self, event):
        parent = event.GetEventObject().GetStringSelection()
        links = self.tfp.parent_links
        self.child_link.Clear()
        self.child_link.AppendItems(links[parent])
        self.child_link.SetSelection(0)
        child = self.child_link.GetString(0)
        self.load_link(parent, child)

    
    def choice2_event(self, event):
        child = event.GetEventObject().GetStringSelection()
        index = self.parent_link.GetSelection()
        parent = self.parent_link.GetString(index)
        self.load_link(parent, child)

    #added - buttons
    def plus_event(self, event):
        name = event.GetEventObject().GetName()
        rospy.loginfo(f"name of event plus: {name}")
        self.inc_dec_value(name, 1.0)

    #added - button
    def minus_event(self, event):
        name = event.GetEventObject().GetName()
        rospy.loginfo(f"name of event minus: {name}")
        self.inc_dec_value(name, -1.0)

    #added - button
    def center_event(self, event):
        rospy.loginfo("Center Event")
        self.center()

    #added - button
    def load_event(self, event):
        rospy.loginfo("Load Event")
        self.load()
    
    #added 
    def load(self):
        rospy.loginfo("Load TF from URDF")
        default = self.tfp.default_value

        for name, element_info in self.element_map.items():
            element = element_info["element"]
            element_info["slidervalue"] = self.valueToSlider(default[name], element)
        self.update_values()

    #added
    def center(self):
        rospy.loginfo("Centering")
        for name, element_info in self.element_map.items():
            element = element_info["element"]
            element_info["slidervalue"] = self.valueToSlider(element["zero"], element)
        self.update_values()

    #not added - not necessary 
    def slidsliderUpdateerUpdate(self, event):
        for name, element_info in self.element_map.items():
            element_info["slidervalue"] = element_info["slider"].GetValue()
        self.update_values()

    #added
    def valueToSlider(self, value, element):
        return (
            (value - element["min"]) * float(RANGE) / (element["max"] - element["min"])
        )

    #added
    def sliderToValue(self, slider, element): 
        pctvalue = slider / float(RANGE)
        return element["min"] + (element["max"] - element["min"]) * pctvalue
    
    