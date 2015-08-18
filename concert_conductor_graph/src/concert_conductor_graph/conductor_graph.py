#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

from __future__ import division
import os

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QAbstractListModel, Signal
from python_qt_binding.QtGui import QGraphicsScene, QIcon, QWidget, QLabel, QComboBox
from python_qt_binding.QtGui import QSizePolicy, QTextEdit, QCompleter, QColor, QPushButton
from python_qt_binding.QtGui import QVBoxLayout, QHBoxLayout, QPlainTextEdit
from python_qt_binding.QtGui import QGridLayout, QTextCursor, QDialog

import rospkg
import concert_msgs.msg as concert_msgs

# Delete this once we upgrade (hopefully anything after precise)
# Refer to https://github.com/robotics-in-concert/rocon_multimaster/issues/248
import threading
threading._DummyThread._Thread__stop = lambda x: 42

###########################

from .interactive_graphics_view import InteractiveGraphicsView
from qt_dotgraph.dot_to_qt import DotToQtGenerator
from qt_gui.plugin import Plugin

# pydot requires some hacks
from qt_dotgraph.pydotfactory import PydotFactory
# TODO: use pygraphviz instead, but non-deterministic layout will first be resolved in graphviz 2.30
# from qtgui_plugin.pygraphvizfactory import PygraphvizFactory

from concert_utilities.conductor_graph import GraphDotcodeGenerator
from concert_utilities.conductor_graph import ConductorGraphInfo

##############################################################################
# Utility Classes
##############################################################################


class GraphEventHandler():

    def __init__(self, tabWidget, item, callback_func):
        self._tabWidget = tabWidget
        self._callback_func = callback_func
        self._item = item

    def NodeEvent(self, event):
        self._callback_func(event)
        for k in range(self._tabWidget.count()):
            if self._tabWidget.tabText(k) == self._item._label.text():
                self._tabWidget.setCurrentIndex(k)

    def EdgeEvent(self, event):
        self._callback_func(event)
        self._item.set_node_color(QColor(0, 0, 255))


##############################################################################
# Dynamic Argument Layer Classes
##############################################################################


class DynamicArgumentLayer():
    def __init__(self, dialog_layout, name='', add=False, params=[]):
        self.dlg_layout = dialog_layout
        self.name = name
        self.add = add
        self.params = params
        self.params_list = []

        params_item = []
        for k in self.params:
            param_name = k[0]
            param_type = k[1]
            param_widget = None
            params_item.append([param_name, param_widget, param_type])
        self.params_list.append(params_item)

        print "DAL: %s" % (self.params_list)

        self.arg_ver_sub_widget = QWidget()
        self.arg_ver_layout = QVBoxLayout(self.arg_ver_sub_widget)
        self.arg_ver_layout.setContentsMargins(0, 0, 0, 0)
        self._create_layout()

    def _create_layout(self):
        name_hor_sub_widget = QWidget()
        name_hor_layout = QHBoxLayout(name_hor_sub_widget)

        name_widget = QLabel(self.name + ": ")
        name_hor_layout.addWidget(name_widget)
        if self.add:
            btn_add = QPushButton("+", name_hor_sub_widget)

            btn_add.clicked.connect(self._push_param)
            btn_add.clicked.connect(self._update_item)
            name_hor_layout.addWidget(btn_add)

            btn_subtract = QPushButton("-", name_hor_sub_widget)
            btn_subtract.clicked.connect(self._pop_param)
            btn_subtract.clicked.connect(self._update_item)
            name_hor_layout.addWidget(btn_subtract)
            pass

        self.arg_ver_layout.addWidget(name_hor_sub_widget)
        self.dlg_layout.addWidget(self.arg_ver_sub_widget)
        self._update_item()

    def _update_item(self):
        widget_layout = self.arg_ver_layout
        item_list = self.params_list

        widget_list = widget_layout.parentWidget().children()
        while len(widget_list) > 2:
            added_arg_widget = widget_list.pop()
            widget_layout.removeWidget(added_arg_widget)
            added_arg_widget.setParent(None)
            added_arg_widget.deleteLater()

        # resize
        dialog_widget = widget_layout.parentWidget().parentWidget()
        dialog_widget.resize(dialog_widget.minimumSize())
        for l in item_list:
            params_hor_sub_widget = QWidget()
            params_hor_layout = QHBoxLayout(params_hor_sub_widget)
            for k in l:
                param_name = k[0]
                param_type = k[2]
                name_widget = QLabel(param_name + ": ")
                if param_type == 'string' or param_type == 'int':
                    k[1] = QTextEdit()
                    k[1].setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.Ignored)
                    k[1].setMinimumSize(0, 30)
                    k[1].append("")
                elif param_type == 'bool':
                    k[1] = QTextEdit()
                    k[1] = QComboBox()
                    k[1].setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.Ignored)
                    k[1].setMinimumSize(0, 30)

                    k[1].addItem("True", True)
                    k[1].addItem("False", False)

                params_hor_layout.addWidget(name_widget)
                params_hor_layout.addWidget(k[1])
            widget_layout.addWidget(params_hor_sub_widget)

    def _push_param(self):
        params_item = []
        for k in self.params:
            param_name = k[0]
            param_type = k[1]
            param_widget = None
            params_item.append([param_name, param_widget, param_type])
        self.params_list.append(params_item)

    def _pop_param(self):
        if len(self.params_list) > 1:
            self.params_list.pop()
        else:
            pass

    def _get_param_list(self):
        return self.params_list
        pass

##############################################################################
# ConductorGraph Classes
##############################################################################


class ConductorGraph(Plugin):

    # pyqt signals are always defined as class attributes
    signal_deferred_fit_in_view = Signal()
    signal_update_conductor_graph = Signal()

    # constants
    # colour definitions from http://www.w3.org/TR/SVG/types.html#ColorKeywords
    # see also http://qt-project.org/doc/qt-4.8/qcolor.html#setNamedColor
    link_strength_colours = {'very_strong': QColor("lime"), 'strong': QColor("chartreuse"), 'normal': QColor("yellow"), 'weak': QColor("orange"), 'very_weak': QColor("red"), 'missing': QColor("powderblue")}

    def __init__(self, context):
        self._context = context
        super(ConductorGraph, self).__init__(context)
        self.initialised = False
        self.setObjectName('Conductor Graph')
        self._node_items = None
        self._edge_items = None
        self._node_item_events = {}
        self._edge_item_events = {}
        self._client_info_list = {}
        self._widget = QWidget()
        self.cur_selected_client_name = ""
        self.pre_selected_client_name = ""
        # factory builds generic dotcode items
        self.dotcode_factory = PydotFactory()
        # self.dotcode_factory=PygraphvizFactory()
        self.dotcode_generator = GraphDotcodeGenerator()
        self.dot_to_qt = DotToQtGenerator()

        self._graph = ConductorGraphInfo(self._update_conductor_graph_relay, self._set_network_statisics)

        rospack = rospkg.RosPack()
        ui_file = os.path.join(rospack.get_path('concert_conductor_graph'), 'ui', 'conductor_graph.ui')
        loadUi(ui_file, self._widget, {'InteractiveGraphicsView': InteractiveGraphicsView})
        self._widget.setObjectName('ConductorGraphUi')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        self._scene = QGraphicsScene()
        self._scene.setBackgroundBrush(Qt.white)
        self._widget.graphics_view.setScene(self._scene)

        self._widget.highlight_connections_check_box.toggled.connect(self._redraw_graph_view)
        self._widget.auto_fit_graph_check_box.toggled.connect(self._redraw_graph_view)
        self._widget.clusters_check_box.toggled.connect(self._redraw_graph_view)

        self.signal_deferred_fit_in_view.connect(self._fit_in_view, Qt.QueuedConnection)
        self.signal_deferred_fit_in_view.emit()

        self._widget.tabWidget.currentChanged.connect(self._change_client_tab)
        self.signal_update_conductor_graph.connect(self._update_conductor_graph)

        context.add_widget(self._widget)

    def restore_settings(self, plugin_settings, instance_settings):
        self.initialised = True
        self._update_conductor_graph()

    def shutdown_plugin(self):
        self._graph.shutdown()

    def _update_conductor_graph(self):
        if self.initialised:
            self._redraw_graph_view()
            self._update_client_tab()

    def _update_conductor_graph_relay(self):
        """
        This seems a bit obtuse, but we can't just dump the _update_conductor_graph callback on the underlying
        conductor graph info and trigger it from there since that trigger will operate from a ros thread and pyqt
        will crash trying to co-ordinate gui changes from an external thread. We need to relay via a signal.
        """
        self.signal_update_conductor_graph.emit()

    def _update_client_tab(self):
        # print('[conductor graph]: _update_client_tab')
        self.pre_selected_client_name = self.cur_selected_client_name
        self._widget.tabWidget.clear()

        for k in self._graph.concert_clients.values():
            # Only pull in information from connected or connectable clients
            if k.state not in [concert_msgs.ConcertClientState.AVAILABLE, concert_msgs.ConcertClientState.MISSING, concert_msgs.ConcertClientState.PENDING]:
                continue

            main_widget = QWidget()

            ver_layout = QVBoxLayout(main_widget)

            ver_layout.setContentsMargins(9, 9, 9, 9)
            ver_layout.setSizeConstraint(ver_layout.SetDefaultConstraint)

            # button layout
            sub_widget = QWidget()
            sub_widget.setAccessibleName('sub_widget')

            ver_layout.addWidget(sub_widget)

            # client information layout
            context_label = QLabel()
            context_label.setText("Client information")
            ver_layout.addWidget(context_label)

            app_context_widget = QPlainTextEdit()
            app_context_widget.setObjectName(k.concert_alias + '_' + 'app_context_widget')
            app_context_widget.setAccessibleName('app_context_widget')
            app_context_widget.appendHtml(k.get_rapp_context())
            app_context_widget.setReadOnly(True)

            cursor = app_context_widget.textCursor()
            cursor.movePosition(QTextCursor.Start, QTextCursor.MoveAnchor, 0)
            app_context_widget.setTextCursor(cursor)
            ver_layout.addWidget(app_context_widget)

            # new icon
            path = ""
            if k.is_new:
                # This only changes when the concert client changes topic publishes anew
                path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../resources/images/new.gif")

            # add tab
            self._widget.tabWidget.addTab(main_widget, QIcon(path), k.concert_alias)

        # set previous selected tab
        for k in range(self._widget.tabWidget.count()):
            tab_text = self._widget.tabWidget.tabText(k)
            if tab_text == self.pre_selected_client_name:
                self._widget.tabWidget.setCurrentIndex(k)

    def _change_client_tab(self, index):
        self.cur_selected_client_name = self._widget.tabWidget.tabText(self._widget.tabWidget.currentIndex())

    def _set_network_statisics(self):
        # we currently redraw every statistics update (expensive!) so passing for now, but we should
        # reenable this and drop the change callback to be more efficient
        # if self._edge_items == None:
        #    return
        # else:
        #    for edge_items in self._edge_items.itervalues():
        #        for edge_item in edge_items:
        #            edge_dst_name = edge_item.to_node._label.text()
        #            edge_item.setToolTip(str(self._graph.concert_clients[edge_dst_name].msg.conn_stats))
        pass

    def _redraw_graph_view(self):
        # print("[conductor graph]: _redraw_graph_view")
        # regenerate the dotcode
        current_dotcode = self.dotcode_generator.generate_dotcode(
            conductor_graph_instance=self._graph,
            dotcode_factory=self.dotcode_factory,
            clusters=self._widget.clusters_check_box.isChecked()
        )
        # print("Dotgraph: \n%s" % current_dotcode)
        self._scene.clear()
        self._node_item_events = {}
        self._edge_item_events = {}
        self._node_items = None
        self._edge_items = None

        highlight_level = 3 if self._widget.highlight_connections_check_box.isChecked() else 1

        # layout graph and create qt items
        (nodes, edges) = self.dot_to_qt.dotcode_to_qt_items(current_dotcode,
                                                            highlight_level=highlight_level,
                                                            same_label_siblings=True)
        self._node_items = nodes
        self._edge_items = edges

        # nodes - if we wish to make special nodes, do that here (maybe subclass GraphItem, just like NodeItem does
        for node_item in nodes.itervalues():
            # redefine mouse event
            # self._node_item_events[node_item._label.text()] = GraphEventHandler(self._widget.tabWidget, node_item, node_item.mouseDoubleClickEvent)
            # node_item.mouseDoubleClickEvent = self._node_item_events[node_item._label.text()].NodeEvent
            self._node_item_events[node_item._label.text()] = GraphEventHandler(self._widget.tabWidget, node_item, node_item.hoverEnterEvent)
            node_item.hoverEnterEvent = self._node_item_events[node_item._label.text()].NodeEvent

            self._scene.addItem(node_item)

        # edges
        for edge_items in edges.itervalues():
            for edge_item in edge_items:
                # redefine the edge hover event

                self._edge_item_events[edge_item._label.text()] = GraphEventHandler(self._widget.tabWidget, edge_item, edge_item._label.hoverEnterEvent)
                edge_item._label.hoverEnterEvent = self._edge_item_events[edge_item._label.text()].EdgeEvent

                # self._edge_item_events[edge_item._label.text()]=GraphEventHandler(self._widget.tabWidget,edge_item,edge_item.mouseDoubleClickEvent);
                # edge_item.mouseDoubleClickEvent=self._edge_item_events[edge_item._label.text()].EdgeEvent;

                edge_item.add_to_scene(self._scene)

                # set the color of node as connection strength one of red, yellow, green
                edge_dst_name = edge_item.to_node._label.text()
                try:
                    link_strength_colour = ConductorGraph.link_strength_colours[self._graph.concert_clients[edge_dst_name].get_connection_strength()]
                    edge_item._default_color = link_strength_colour
                    edge_item.set_node_color(link_strength_colour)
                    # set the tooltip about network information
                    edge_item.setToolTip(str(self._graph.concert_clients[edge_dst_name].msg.conn_stats))
                except KeyError:
                    # edge disappeared due to recent updates, (we could use better placement of mutexes around this data here and in callbacks)
                    # for now, skip rendering it
                    continue

        self._scene.setSceneRect(self._scene.itemsBoundingRect())

        if self._widget.auto_fit_graph_check_box.isChecked():
            self._fit_in_view()

    def _fit_in_view(self):
        self._widget.graphics_view.fitInView(self._scene.itemsBoundingRect(), Qt.KeepAspectRatio)
