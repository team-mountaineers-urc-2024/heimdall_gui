# Heimdall GUI Code Description
#### *This document contains information about how the code in each section of the GUI works*


## Current file structure as of 4/17/2024
GUI code in src/heimdall_gui/heimdall_gui

*   /rover_gui_common contains code that hasn't been fully implemented, specifically for driving controls
*   /ui_python contains large python files generated from QT Designer files. Specifically for the science and controls tab, and the right border display
*   /ui_xml contains .xml files, currently just for the updated science tab. Will update later
*   /urc_gui_common contains code for most of the tabs, split into subdirectories
    *   /helpers contains code for helper functions, such as checking for internet access
    *   /tabs contains python files for creating components in each tab that don't have a ui file. Specically cameras, the map tab, the settings tab, and the tools tab
    *   /ui_python consists of python files generated from ui files for smaller components
    *   /widgets contains code that initializes and modifies the files in the /ui_python directory mentioned directly above
    *   The rest of the /urc_gui_common files are components that either implement functionality or are separate from a tab
        *   app.py initializes the window and adds controls such as closing the window
        *   camera_link.py contains code for handling camera subscribers and related services
        *   ros_link.py contains code that creates most of the subscribers related to the autonomy mission
        *   status_bar.py creates a the display on the bottom border of the window
        *   tile_scraper.py contains code to cache map tiles so that the map can be viewed without access to internet
*   Other files in the src/heimdall_gui/heimdall_gui directory include:
    *   current_display_widget.py which initializes the widget on the right border
    *   heimdall_controls.py (unimplemented) which initializes the controls tab
    *   heimdall_ros_link which contains code for creating ROS nodes mostly related to the science mission
    *   main.py which contains a class that creates the main window and adds all of the tabs/components to the window. **This file is what is run when launching the GUI**
    *   science.py contains all of the logic and input handlers for the science mission
    *   spectrometer_readings.py is a list of spectrometer readings used in science.py


## How it works
#### *The rest of the document will explain the code for the major components in the GUI*

### Cameras
*Note: the current code for cameras is slightly convoluted. It could end up being refactored*

Every camera in the GUI consists of the same components with the SuperCameraWidget (may be renamed if refactored) being the main component. The code that runs the cameras is split across multiple files: settings.py, super_camera_widget.py, camera_link.py, and ros_link.py.
* **ros_link.py** is used to connect the settings tab to the cameras tab. This file contains a variable called "camera_funnel", which is an instance of the CameraFunnel class from camera_link.py (more on that below). The RosLink class is passed to both the settings tab and each camera tab so they all have access to the camera_funnel.
* **settings.py** is what detects the camera topics and adds them to a list in the CameraFunnel class. It contains a function "search_for_cameras" that gets a list of all available topics and checks them against a regular expression to find camera topics. If a valid topic is found, it adds the name of the camera in format 'logitech_xx' to a list of CameraData, and then calls a function that updates a list in the CameraFunnel. It also uses the CameraData to keep track of the cameras for use with the resolution buttons so that it can call the service associated with the selected camera topic.
*  **super_camera_widget.py** creates the components for displaying the camera image and the controls for adjusting it. Each camera display has it's own instance of the SuperCameraWidget so that the controls are separate for each display. It contains functions that take input from components such as the screenshot button, the contrast/brightness slider, and the list of cameras to select from. It also contains a function that updates the image displayed from the camera feed. 
*  **camera_link** contains two classes. One is the CameraSubscriber which is created for each camera topic whenever a camera is detected. The other class is the CameraFunnel which is used for sharing information across tabs in the GUI. The CameraFunnel class keeps a list of subscribers, which is updated every time the the function in settings.py detects a camera topic. These subsribers are instances of the CameraSubscriber class. The CameraSubscriber class keeps track of the camera topic that the camera is associated with and contains functions that handle ROS connections, such as creating a subscriber to the camera topic and calling the service associated with the topic when changing the resolution. The CameraFunnel connects the functions in the CameraSubscriber class to buttons and sliders in the super_camera_widget and the settings tab.

### Map
The map tab code is spread across two files: urc_gui_common/tabs/map.py and urc_gui_common/widgets/map_viewer.py. **map.py** mostly consists of code that creates and handles user input while **map_viewer.py** creates the map and manages points/waypoints/lines drawn on the map. The map itself utilizes a python library called 'pyqtlet2' which is a wrapper around a javascript library called 'Leaflet'. This wrapper allows the map to be displayed in Qt using python, but some javascript code is still required to make everything work. 
*   **map_viewer.py** is what creates the map using pyqtlet2 and manages points through the use of layers. Everything displayed on the map is on a layer. The bottom-most layer is composed of tiles, which is how the map is displayed. Above that are the layers that hold points or markers. Points in this file are kept in a list and then passed to the 'set_points' function, along with the layer they should be added to. This function then calls another function to draw a line through the points. This 'draw_line_through_points' function takes a Polyline (created during initialization of the map) and calls a javascript function that takes the list of points (specifically the lat and lon from the points), and then adds those points to the Polyline. This is what draws the line that represents the rover's path. The 'add_saved_path' function, which allows for loading previous paths from a file, uses this functionality by reading lat, lon coordinates from a file and creating a new layer with those coordinates as points. These layers can then be added or removed as needed.
*   **map.py** handles the Qt code for displaying a table of markers and also taking input from buttons and input fields. Most of the input is taken from the fields and then passed to the map_viewer, if valid, to then display things like added markers. One thing to note is that this file also utilizes the tile_scraper.py file to determine what map servers are available and to also cache map tiles if internet is available. The tiles are cached to the directory '~/.ros/tile_cache'
  
#### Important notes about tiles:
The map that is displayed is obtained from a map server provider called Arcgis, from 'https://server.arcgisonline.com/arcgis/rest/services', specifically the World Imagery server. Other map servers can be added as long as they follow the same tile format. The file format has to be either a url or directory ending in '/{z}/{y}/{x}', where z represents the layer number, y represents the tile number from top to bottom, and x represents the tile number from left to right. The layer number represents the current resolution, with 0 usually being the lowest having only one image, and each layer increasing the resolution (number of tile images). The tiles are images with the name of the tile being the x value. When displayed, the images are arranged in a grid with the x and y following the format above.

### Science
The science code, while long, is actually very simple. The UI (ui_python/science_tab.py) is instantated in science.py, and then every button and slider is connected to a function in science.py. Most of these functions just retrieve data from the input, validate it, and then pass the data to heimdall_ros_link to call the relevant services/publishers. 

### Settings / Camera Controls
The settings code is also fairly simple. settings.py contains the code that searches for camera topics and adds them to a list, changes the resolution of cameras, and also saves and loads camera presets. The most important function (search_for_cameras), as mentioned in the camera section, uses regular expressions to check every topic to find one that matches. It keeps a list of the camera names so that it can access the matching camera subscriber in the CameraFunnel. The resolution buttons in the settings tab are linked to a function in the SuperCameraWidget, which just calls the relevant service for changing resolution parameters. The camera preset code just saves and loads from a .yaml file, by saving relevant information for each camera name and then also reading in and loading that information into the list it keeps, and then updates the camera with information kept in the list.