# Heimdall GUI Functionality
#### *This document contains information about the features in each tab work and how to use them*
#### *Updated 5/09/2024*

## Mission Selector
**Works with the workspace-heimdall directory now**
#### How to Use
The mission selector can be used after running the "install_desktop.sh" script, navigating to the desktop, right clicking the launch selector icon and allow launching, and then double clicking the icon. The mission selector will launch a window that displays all of the available launch scripts. Clicking the checkbox next to a script and then pressing launch will launch the selected script.

## Camera Tabs
All of the features associated with the camera tabs are applicable to each camera slot, including the cameras in the science tab
#### Features:
*   Camera select: The dropdown menu in the top left corner will show available cameras. Clicking on one will display the feed from that camera
*   Enable / Disable: The checkbox next to the select dropdown allows you to enable or disable the current camera feed. Unchecking it will simply replace the camera images with a white square
*   Screenshot button: The screenshot button in the top center will save the image that is currently displayed in the camera feed square. It saves images to the ~/.ros/screenshots directory
*   Contrast slider: The slider on the right side is currently configured to change the contrast of the camera that is currently selected
*   Refresh button: **may not work exactly as it was intended** This button above the contrast slider effectively "refreshes" the camera feed by changing the resolution of the camera and then changing it back


## Map Tab
#### Features:
* Marker list: The marker list table displays markers that are currently being kept track of
* Inputs for adding markers: Under the table are several input fields for adding new markers to the list. Currently, the radius and second aruco ID are not being used and are inputs kept from last year
* Marker presets: **WIP, currently trying to fix** Under the clear markers button is a section for loading marker presets. Before alterations to marker types, this section would allow saving and loading markers to/from a file. It also appeared to have an autosave functionality so that manually saving after every new waypoint wasn't necessary. Due to changes made this year, this feature currently isn't functional but should be soon. The preset files are saved to ~/.ros/autonomy_saves
* Loading previously recorded paths: The load path subtab displays a table that contains a list of files, if any, that were created with previously recorded rover paths. Selecting the checkbox next to a listed file will display that path on the map at the coordinates where the path was recorded.
* Map server select: **Currently just works with ArcGIS server** The server select on the bottom left allows for selecting which map to display on the right. Currently it is only configured to use either the online ArcGIS World Imagery map server or a cached version, which provides limited available tiles. An internet connection is required in order to view the online map or to cache the tiles for offline use. As mentioned in the code document, other map servers are able to be added if they follow the allows tile format. The cached tiles are saved to the path ~/.ros/tile_cache
* Scrollable tile map: The tile map on the right side displays a World Imagery Map at varying resolutions. It is possible to scroll in and out of the map, or to drag in order to move. This map also displays markers, the curent position of the rover, and can display the path the rover has traveled.
* Coordinate autofill: Double clicking the map at any point will autofill the longitude and latitude input boxes with the coordinates that closely approximate where the mouse was clicked.

## Science Tab
### Actuation Control Subtab
#### Features:
*   Actuator slider: The five sliders allow for adjusting the length of the actuator that is titled above the slider. Dragging the slider down will increase the extension of the actuator.
*   Actuator increment: The subsurface actuator can be incrementally lowered/raised using the increment button. The input box with the number allows for selecting the amount that the actuator is incremented, and pressing the button will lower/raise the actuator that amount.
*   Subsurface Motor Control: The motor control radio buttons are used to turn the subsurface motor on and off
*   Soil sensor read/display: **WIP** The soil sensor buttons will read the data from the connected soil sensors and display that data directly under the read button.
*   Scoop buttons: The scoop dropdown menu is used to select the scoop, the scoop button will activate it, and the reset scoop button will reset the scoop position.
*   Pump buttons: The dropdown menus in the pump section allow for selecting a pump and selecting a direction to turn it. The duration input box is used to specifiy how long the pump will turn for. The pump button will start turning it for the specified amount of time.
*   Error console: The console at the bottom of the tab is used to display specific science related messages, such as failure or success to perform science operations.

### Analysis Subtab
#### Features
*   Spectrometer graph display:
*   Spectrometer overlays:
*   Spin centrifuge/move cuvette:
*   Spectrometer data capture and history:
*   Light switch control: *Turns off after a certain amount of time*

## Controls Tab
*Not implemented yet*

## Tools Tab
#### Features:
*   Custom timer:
*   Ros log console: 
*   Mission checklist display: (uses yaml files in ~/.ros/mission_checklists directory)
  
## Settings Tab
#### Features:
*   Camera info:
*   Resolution select: 
*   Camera presets: 
*   Daemon reset button: 