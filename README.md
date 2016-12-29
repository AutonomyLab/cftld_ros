## Intro

ROS wrapper for CFTld, a long-term visual tracker, Based on [CFTld tracker](https://github.com/klahaag/CFtld),

## Installation

Pre-requirements:

- Ubuntu Xenial (16.04)
- ROS Kinetic
- OpenCV 3.x ([provided by ROS Kinetic](http://wiki.ros.org/opencv3?distro=kinetic))

1. Clone this repository into your `<catkin_ws>/src` folder
2. Install the required dependencies 

    $ cd /path/to/catkin_ws
    $ rosdep update
    $ rosdep install --from-paths src -i

3. Build the package using `catkin_make` (or `catkin build` if you are using [the new catkin](https://catkin-tools.readthedocs.io/en/latest/))

## Usage

TBA

## Misc

Debug the nodelet with GDB:

```bash
$ rosrun --prefix "gdb --args" nodelet nodelet standalone cftld_ros/CFtldRosNodelet```

### Some resources

- https://github.com/ros-perception/image_pipeline/tree/indigo/image_proc/src
- https://groups.google.com/forum/?utm_medium=email&utm_source=footer#!msg/ros-sig-perception/K5__71SX7eU/mxWwn3AeAwAJ
- http://wiki.ros.org/opencv3
- http://answers.ros.org/question/214043/use-ros-indigo-opencv3-alongside-248/
- https://github.com/ros-perception/vision_opencv/issues/91
- http://tayyabnaseer.blogspot.ca/2013/04/porting-nodes-to-nodelets-in-ros.html
