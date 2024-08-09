# Heimdall GUI

## Description
The repository defines the GUI that was used to interact with Heimdall for the
2023 URC challenge. It contains code for defining the structure and functionality
of the GUI, as well as 'roslink' files that connect the  

## Prequisites
Requires sshpass (sudo apt install sshpass) for ssh-ing for the daemon restart

Notes about daemon reset:
* The GUI will freeze for a few seconds while resetting the daemon node
* sshpass is required to provide a password when ssh-ing into the rover


Requires the following pip packages:
* pyqt5
* qtpy
* pyqtlet2 (version 0.8.0)
* pyqtgraph
* pymap3d
* pyproj

Requires the following apt packages:
* python3-qtpy

Run `colcon build` to build all required packages

Source the setup file with `source install/setup.bash`

Run `ros2 launch heimdall_gui gui_bringup.launch.py` to launch the GUI

## To Launch from a shortcut
Run colcon build in the project directory

Run install_desktop.sh

Right click the icon on the desktop and select "Allow launching"

Double click the icon

## To use mission selector
*Now works with the workspace-heimdall*
Run the install_desktop.sh file to install the desktop icon

Right click the icon on the desktop for 'Launch Selector' and select "Allow launching"

Double click the icon

## Known Limitations
The structure of this repository is a bit messy and is hard to understand.