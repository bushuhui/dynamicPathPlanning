# Dynamic Path Planning

This program demostrates how to dynamicly plan a path from a begin point to an end point, where the environment is incrementally scanned. The simplest A* algorithm is used to plan a path from current position to destination using currenly scanned map. Different with traditionaly path planning, it can plan route for completely unknown environment.


## Requirements:
* Qt4 (sudo apt-get install libqt4-core libqt4-dev)


## Compile:
`qmake pathplan_gui.pro`

`make`


## Usage:
```
./pathplan_gui
```

Keyboard:
* 'L' - load a map
* 'B' - Begin dynamic path planning (step)
* 'N' - Next step
* 'H' - Automatically run
* 'P' - A* path planning
* 'C' - Clear results

Mouse:
* 'Right click' - open menu
* 'Scroll - up' - Zoom out
* 'Scroll - down' - Zoom in


## Plateform:
Only test on Linux Mint 16 64-bit. 


## Screenshot:
-![alt text](http://blog.adv-ci.com/wp-content/uploads/2013/05/Screenshot-2D-SLAM-1.png "Screenshot 1")
-![alt text](http://blog.adv-ci.com/wp-content/uploads/2013/05/Screenshot-2D-SLAM.png "Screenshot 2")


## Project homepage:
http://www.adv-ci.com/blog/source/dynamicpathplanning/
