# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'april_camera_tf_gui.ui'
##
## Created by: Qt User Interface Compiler version 5.15.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *


class Ui_Form(object):
    def setupUi(self, Form):
        if not Form.objectName():
            Form.setObjectName(u"Form")
        Form.resize(570, 461)
        self.gridLayout = QGridLayout(Form)
        self.gridLayout.setObjectName(u"gridLayout")
        self.verticalLayout_8 = QVBoxLayout()
        self.verticalLayout_8.setObjectName(u"verticalLayout_8")
        self.verticalLayout_6 = QVBoxLayout()
        self.verticalLayout_6.setObjectName(u"verticalLayout_6")
        self.horizontalLayout_3 = QHBoxLayout()
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.label = QLabel(Form)
        self.label.setObjectName(u"label")

        self.horizontalLayout_3.addWidget(self.label)

        self.comboBox_parent = QComboBox(Form)
        self.comboBox_parent.setObjectName(u"comboBox_parent")

        self.horizontalLayout_3.addWidget(self.comboBox_parent)


        self.verticalLayout_6.addLayout(self.horizontalLayout_3)

        self.horizontalLayout_2 = QHBoxLayout()
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.label_2 = QLabel(Form)
        self.label_2.setObjectName(u"label_2")

        self.horizontalLayout_2.addWidget(self.label_2)

        self.comboBox_child = QComboBox(Form)
        self.comboBox_child.setObjectName(u"comboBox_child")

        self.horizontalLayout_2.addWidget(self.comboBox_child)


        self.verticalLayout_6.addLayout(self.horizontalLayout_2)


        self.verticalLayout_8.addLayout(self.verticalLayout_6)

        self.horizontalLayout_4 = QHBoxLayout()
        self.horizontalLayout_4.setObjectName(u"horizontalLayout_4")
        self.steps_slider = QSlider(Form)
        self.steps_slider.setObjectName(u"steps_slider")
        self.steps_slider.setOrientation(Qt.Horizontal)

        self.horizontalLayout_4.addWidget(self.steps_slider)

        self.horizontalSpacer_3 = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout_4.addItem(self.horizontalSpacer_3)

        self.label_9 = QLabel(Form)
        self.label_9.setObjectName(u"label_9")

        self.horizontalLayout_4.addWidget(self.label_9)

        self.steps_field = QLineEdit(Form)
        self.steps_field.setObjectName(u"steps_field")

        self.horizontalLayout_4.addWidget(self.steps_field)


        self.verticalLayout_8.addLayout(self.horizontalLayout_4)

        self.verticalLayout_7 = QVBoxLayout()
        self.verticalLayout_7.setObjectName(u"verticalLayout_7")
        self.horizontalLayout = QHBoxLayout()
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.verticalLayout_9 = QVBoxLayout()
        self.verticalLayout_9.setObjectName(u"verticalLayout_9")
        self.label_3 = QLabel(Form)
        self.label_3.setObjectName(u"label_3")

        self.verticalLayout_9.addWidget(self.label_3)

        self.label_6 = QLabel(Form)
        self.label_6.setObjectName(u"label_6")

        self.verticalLayout_9.addWidget(self.label_6)

        self.label_8 = QLabel(Form)
        self.label_8.setObjectName(u"label_8")

        self.verticalLayout_9.addWidget(self.label_8)

        self.label_7 = QLabel(Form)
        self.label_7.setObjectName(u"label_7")

        self.verticalLayout_9.addWidget(self.label_7)

        self.label_5 = QLabel(Form)
        self.label_5.setObjectName(u"label_5")

        self.verticalLayout_9.addWidget(self.label_5)

        self.label_4 = QLabel(Form)
        self.label_4.setObjectName(u"label_4")

        self.verticalLayout_9.addWidget(self.label_4)


        self.horizontalLayout.addLayout(self.verticalLayout_9)

        self.verticalLayout_5 = QVBoxLayout()
        self.verticalLayout_5.setObjectName(u"verticalLayout_5")
        self.x_minus = QPushButton(Form)
        self.x_minus.setObjectName(u"x_minus")

        self.verticalLayout_5.addWidget(self.x_minus)

        self.y_minus = QPushButton(Form)
        self.y_minus.setObjectName(u"y_minus")

        self.verticalLayout_5.addWidget(self.y_minus)

        self.z_minus = QPushButton(Form)
        self.z_minus.setObjectName(u"z_minus")

        self.verticalLayout_5.addWidget(self.z_minus)

        self.roll_minus = QPushButton(Form)
        self.roll_minus.setObjectName(u"roll_minus")

        self.verticalLayout_5.addWidget(self.roll_minus)

        self.pitch_minus = QPushButton(Form)
        self.pitch_minus.setObjectName(u"pitch_minus")

        self.verticalLayout_5.addWidget(self.pitch_minus)

        self.yaw_minus = QPushButton(Form)
        self.yaw_minus.setObjectName(u"yaw_minus")

        self.verticalLayout_5.addWidget(self.yaw_minus)


        self.horizontalLayout.addLayout(self.verticalLayout_5)

        self.horizontalSpacer = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout.addItem(self.horizontalSpacer)

        self.verticalLayout_3 = QVBoxLayout()
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.x_plus = QPushButton(Form)
        self.x_plus.setObjectName(u"x_plus")

        self.verticalLayout_3.addWidget(self.x_plus)

        self.y_plus = QPushButton(Form)
        self.y_plus.setObjectName(u"y_plus")

        self.verticalLayout_3.addWidget(self.y_plus)

        self.z_plus = QPushButton(Form)
        self.z_plus.setObjectName(u"z_plus")

        self.verticalLayout_3.addWidget(self.z_plus)

        self.roll_plus = QPushButton(Form)
        self.roll_plus.setObjectName(u"roll_plus")

        self.verticalLayout_3.addWidget(self.roll_plus)

        self.pitch_plus = QPushButton(Form)
        self.pitch_plus.setObjectName(u"pitch_plus")

        self.verticalLayout_3.addWidget(self.pitch_plus)

        self.yaw_plus = QPushButton(Form)
        self.yaw_plus.setObjectName(u"yaw_plus")

        self.verticalLayout_3.addWidget(self.yaw_plus)


        self.horizontalLayout.addLayout(self.verticalLayout_3)

        self.horizontalSpacer_2 = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout.addItem(self.horizontalSpacer_2)

        self.verticalLayout_4 = QVBoxLayout()
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.x_field = QLineEdit(Form)
        self.x_field.setObjectName(u"x_field")

        self.verticalLayout_4.addWidget(self.x_field)

        self.y_field = QLineEdit(Form)
        self.y_field.setObjectName(u"y_field")

        self.verticalLayout_4.addWidget(self.y_field)

        self.z_field = QLineEdit(Form)
        self.z_field.setObjectName(u"z_field")

        self.verticalLayout_4.addWidget(self.z_field)

        self.roll_field = QLineEdit(Form)
        self.roll_field.setObjectName(u"roll_field")

        self.verticalLayout_4.addWidget(self.roll_field)

        self.pitch_field = QLineEdit(Form)
        self.pitch_field.setObjectName(u"pitch_field")

        self.verticalLayout_4.addWidget(self.pitch_field)

        self.yaw_field = QLineEdit(Form)
        self.yaw_field.setObjectName(u"yaw_field")

        self.verticalLayout_4.addWidget(self.yaw_field)


        self.horizontalLayout.addLayout(self.verticalLayout_4)


        self.verticalLayout_7.addLayout(self.horizontalLayout)

        self.zero_button = QPushButton(Form)
        self.zero_button.setObjectName(u"zero_button")

        self.verticalLayout_7.addWidget(self.zero_button)

        self.load_default_button = QPushButton(Form)
        self.load_default_button.setObjectName(u"load_default_button")

        self.verticalLayout_7.addWidget(self.load_default_button)


        self.verticalLayout_8.addLayout(self.verticalLayout_7)


        self.gridLayout.addLayout(self.verticalLayout_8, 1, 1, 1, 1)

        self.horizontalSpacer_5 = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.gridLayout.addItem(self.horizontalSpacer_5, 1, 2, 1, 1)

        self.verticalSpacer = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.gridLayout.addItem(self.verticalSpacer, 0, 1, 1, 1)

        self.horizontalSpacer_4 = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.gridLayout.addItem(self.horizontalSpacer_4, 1, 0, 1, 1)

        self.verticalSpacer_2 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.gridLayout.addItem(self.verticalSpacer_2, 2, 1, 1, 1)


        self.retranslateUi(Form)

        QMetaObject.connectSlotsByName(Form)
    # setupUi

    def retranslateUi(self, Form):
        Form.setWindowTitle(QCoreApplication.translate("Form", u"Form", None))
        self.label.setText(QCoreApplication.translate("Form", u"parent:", None))
        self.label_2.setText(QCoreApplication.translate("Form", u"child:", None))
        self.label_9.setText(QCoreApplication.translate("Form", u"steps:", None))
        self.label_3.setText(QCoreApplication.translate("Form", u"x", None))
        self.label_6.setText(QCoreApplication.translate("Form", u"y", None))
        self.label_8.setText(QCoreApplication.translate("Form", u"z", None))
        self.label_7.setText(QCoreApplication.translate("Form", u"roll", None))
        self.label_5.setText(QCoreApplication.translate("Form", u"pitch", None))
        self.label_4.setText(QCoreApplication.translate("Form", u"yaw", None))
        self.x_minus.setText(QCoreApplication.translate("Form", u"-", None))
        self.y_minus.setText(QCoreApplication.translate("Form", u"-", None))
        self.z_minus.setText(QCoreApplication.translate("Form", u"-", None))
        self.roll_minus.setText(QCoreApplication.translate("Form", u"-", None))
        self.pitch_minus.setText(QCoreApplication.translate("Form", u"-", None))
        self.yaw_minus.setText(QCoreApplication.translate("Form", u"-", None))
        self.x_plus.setText(QCoreApplication.translate("Form", u"+", None))
        self.y_plus.setText(QCoreApplication.translate("Form", u"+", None))
        self.z_plus.setText(QCoreApplication.translate("Form", u"+", None))
        self.roll_plus.setText(QCoreApplication.translate("Form", u"+", None))
        self.pitch_plus.setText(QCoreApplication.translate("Form", u"+", None))
        self.yaw_plus.setText(QCoreApplication.translate("Form", u"+", None))
        self.zero_button.setText(QCoreApplication.translate("Form", u"Zero", None))
        self.load_default_button.setText(QCoreApplication.translate("Form", u"Load default", None))
    # retranslateUi

