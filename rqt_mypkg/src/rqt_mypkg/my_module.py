import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
import tf

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
        
        self._widget.plus_btn_1.clicked.connect(self.on_plus_btn_1_clicked)

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
    
    def on_plus_btn_1_clicked(self):
        print("button clicked!")

    def tfPublisher(self):
        self.parent_links = {}
        self.loadAllFrameList()

        if len(self.parent_links) == 0:
            print("No TF Data...")
            sys.exit(-1)

        default_parent = list(self.parent_links.keys())[0]
        default_child = self.parent_links[default_parent][0]
        self.parent_frame = rospy.get_param("~parent_frame", default_parent)
        self.child_frame = rospy.get_param("~child_frame", default_child)

        self.elements = {}
        self.element_list = []  # for maintaining the original order of the elements
        self.default_value = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        }

        for name in ["x", "y", "z"]:
            element = {"min": -2.0, "max": 2.0, "zero": 0.0, "value": 0.0}
            self.elements[name] = element
            self.element_list.append(name)

        for name in ["roll", "pitch", "yaw"]:
            element = {"min": -pi, "max": pi, "zero": 0.0, "value": 0.0}
            self.elements[name] = element
            self.element_list.append(name)

        # load all frame list

        rospy.loginfo("Default link %s => %s" % (self.parent_frame, self.child_frame))

        # save original tf as default value
        self.load_default()


    def loadAllFrameList(self):
        listener = tf.TransformListener(rospy.Duration(10))
        counter = 0
        while counter < 5:
            parent_links = self.parseFrameString(listener.allFramesAsDot())
            if len(parent_links.keys()) > 0:
                self.parent_links = parent_links
                return

            rospy.loginfo("Waiting for TF..")
            counter += 1
            time.sleep(1)
        print("No TF Data Available")

    def parseFrameString(self, str):
        alljoints = []
        for line in str.splitlines():
            joint = line.partition("[")[0]
            p, c = self.getParentChild(joint)
            alljoints.append(joint)
        alljoints = alljoints[1:-1]  # remove first and last item
        parent_links = {}
        for j in alljoints:
            p, c = self.getParentChild(j)
            if p in parent_links.keys():
                parent_links[p].append(c)
            else:
                parent_links[p] = [c]
        return parent_links

    def load_default(self):
        self.load_link(self.parent_frame, self.child_frame)

    def load_link(self, parent, child):
        listener = tf.TransformListener()
        trans = None
        rot = None

        rospy.loginfo("Loading TF between %s => %s" % (parent, child))
        # if not listener.frameExists(parent[1:]): # no '/'
        #     rospy.loginfo("Parent Frame not exist: %s" %(parent))
        #     return
        # if not listener.frameExists(child[1:]):# no '/'
        #     rospy.loginfo("Child not exist %s" %(child))
        #     return

        listener.waitForTransform(parent, child, rospy.Time(0), rospy.Duration(5))
        try:
            (trans, rot) = listener.lookupTransform(parent, child, rospy.Time(0))
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            print("Get TF ERROR")
            return
        euler = tf.transformations.euler_from_quaternion(rot)
        print("Trans", trans, "rot", euler)
        for i, name in enumerate(["x", "y", "z"]):
            self.default_value[name] = trans[i]
        for i, name in enumerate(["roll", "pitch", "yaw"]):
            self.default_value[name] = euler[i]

    def getParentChild(self, joint):
        parent, sep, child = joint.partition(" -> ")
        return ("/" + parent.replace('"', ""), "/" + child.replace('"', ""))

    def loadJoint(self, joint):
        p, c = self.getParentChild(joint)
        self.parent_frame = p
        self.child_frame = c
        self.load()  # reload default value


    # def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
