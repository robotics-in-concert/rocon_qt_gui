^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rocon_qt_library
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.11 (2015-07-09)
-------------------
* resolve conflict between threading lock and qmessage box closes `#196 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/196>`_
* Contributors: Jihoon Lee

0.7.10 (2015-04-27)
-------------------
* add missing dependency closes `#189 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/189>`_
* cleanup resource item list to closes `#187 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/187>`_
* Contributors: Jihoon Lee

0.7.9 (2015-03-30)
------------------

0.7.8 (2015-03-23)
------------------

0.7.7 (2015-03-02)
------------------

0.7.6 (2015-02-28)
------------------

0.7.5 (2015-02-09)
------------------
* fix name typo
* ann use pose covariance
* rocon_qt_library convert table to waypoint
* Contributors: Jihoon Lee

0.7.4 (2014-12-30)
------------------
* update setup.py to install submodules fixes `#173 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/173>`_
* Contributors: Jihoon Lee

0.7.3 (2014-12-29)
------------------
* relocate ar_marker height as waiterbot
* force to use lower case and underscore instead of capitals and space in world and map name `#167 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/167>`_
* Contributors: Jihoon Lee

0.7.2 (2014-11-21)
------------------

0.7.1 (2014-11-21)
------------------
* shutdown_plugin hook to releasing resources resolves `#165 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/165>`_
* it now reveals gone resource and updates lists Closes `#163 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/163>`_
* having lock around resource_item_list to avoid racing condition. close`#161 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/161>`_
* pep8 style method names in world_canvas library Fix `#159 <https://github.com/robotics-in-concert/rocon_qt_gui/issues/159>`_
* bug fixed
* blocks improper save button use
* now listworld is available
* ignore combobox reset event
* remove log
* change draw viz_marker
* load map implemented
* upgrade map drawing
* updates mirror transition
* adjust x-axis symmetry to annotations
* change display method about annotation info at textbox
* minor update
* updates
* add destroyed hook in resource chooser
* display unit in ui
* create concert qt map annotater
* save working
* updates
* updates
* skipping processing scan and robot pose if it didn't receive map
* removing unnecessary files
* delete usused code
* fixed map save btn layout
* make slam widget instead of slam view
* migrate code into util directory
* create rocon qt util and update slam viewer code clean
* delete unused code
* rendering robot pose and laser scan info
* scan and robot pose topic set
* add concert_qt_make_a_map
* resource chooser and teleop interface are now separated. rocon_teleop and concert_teleop works fine
* Contributors: DongWook Lee, Jihoon Lee, dwlee

0.7.0 (2014-08-25)
------------------
* remove debugging print.
* update publisher queue_size to avoid warning in indigo.
* rocon_qt_teleop added.
* Contributors: Daniel Stonier

0.5.4 (2013-09-11)
------------------

0.5.3 (2013-08-30)
------------------

0.5.2 (2013-07-17)
------------------

0.5.1 (2013-06-10 16:50:50 +0900)
---------------------------------

0.5.0 (2013-05-27)
------------------

0.3.1 (2013-04-09)
------------------

0.3.0 (2013-02-05)
------------------

0.2.0 (2013-01-31)
------------------
