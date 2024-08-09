#!/bin/bash

cp urc_logo.png ~/.icons/urc_logo.png

cp gui_launcher.desktop ~/Desktop/gui_launcher.desktop

cp launch_selector/launch_selector.desktop ~/Desktop/launch_selector.desktop

# Add launch selector folder to home directory
cp -r launch_selector ~

cd ~/Desktop

# gio set gui_launcher.desktop metadata::trusted true

sudo chmod a+x gui_launcher.desktop

sudo chmod a+x launch_selector.desktop

sudo desktop-file-install gui_launcher.desktop 

sudo desktop-file-install launch_selector.desktop 
