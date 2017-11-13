### roboy_rqt_plugins
rqt gui plugins 

### installation
* add the repo as a submodule to your catkin workspace:
```
git submodule add https://github.com/Roboy/roboy_rqt_plugins
```
* build your workspace:
```
catkin_make
```
* source your devel/setup.bash

### run it
* fire up rqt, then add the plugins you need
* ![rqt](https://github.com/Roboy/roboy_rqt_plugins/blob/master/images/rqt.png?raw=true "rqt")

### FAQ
* the plugins cannot be added/ they dont show up. This seems to be caused by rqt not updateing. Use the following command to reset the remove init file. Then restart rqt:
```
rm ~/.config/ros.org/rqt_gui.ini
rqt
```
