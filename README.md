# SWC_RaspPi-bot

This is [Yu Lin](https://github.com/yulint/) and [Lucas](https://github.com/lssimoes) implementation of the robot's algorithm for _SWC Bootcamp 2018_.

## The problem

The task is to use [RPi 3 B+](https://www.raspberrypi.org/products/raspberry-pi-3-model-b-plus/) to build a robot that sees blue balloons and pops them, while keeping red balloons intact. 

## The implementation

The way to run the code is to get inside the folder `robot/`, enter the Python shell (run `python3`) and then:

```python
Python 3.x.y
>>> from robotfunctions import *
```

This should get you in a python environment with all the functions imported! (work in progress, not very python-like).

The testing still needs to get implemented, but there are some ideas in the `tests/` folder .

Our implementation is written in [Python 3](https://www.python.org/) and uses several libraries, such as:

- [OpenCV](https://docs.opencv.org/3.4.3/)
- [RPi.GPIO](https://sourceforge.net/projects/raspberry-gpio-python/)

## The reasoning

An outline of the (initial) idea for the robot is as follows:

```python
while True:
    img = capture_image
    blue_object, x, y = find_objects(img) # centroid ellipse
    
    if blue_object:
        decide_direction_move(x, y)
        obstacles = check_for_obstacles_near_directions(x, y)
        if not obstacles:
            move_towards_blue_objects
            check_success: (no blue_object OR bigger(blue_object)) AND image_changed
            if not_success:
                take_stuck_measures
    
        if obstacles_other_blue:
            take_some_turns
            if not_success:
                take_stuck_measures

    if not blue_object:
        while not_moved:
            check_other_directions
            if not_near_object  # reasonably far
                change_direction

```
