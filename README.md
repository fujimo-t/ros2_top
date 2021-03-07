# WIP: Top-like monitoring tool for ROS2

## Implemented features

* List of node/topic/service/action
* Node info

## Dependencies

* ROS2 Foxy
* [asciimatics](https://github.com/peterbrittain/asciimatics)

## Install

Clone to colcon workspace and build

## Related tools

* [rqt_top](https://github.com/ros-visualization/rqt_top)
* [rosmon](https://github.com/xqms/rosmon)
* [ros2cli](https://github.com/ros2/ros2cli)

## TODO

* Actions for each list (e.g. change node state, echo topic)
* change asciimatics theme
* history
* Shortcut keys

### Scenes

* action
  * info
  * send_goal
* topic
  * info
  * echo/hz/bandwidth/delay
  * publish
* node
  * state change
* services
  * call

### Lists

* Filtering
* horizontal scroll
* Dynamic columns width
* Sort
