# Code Readme (Cat Tracker)

## Folder: Sensor2nodeServer2csv2html2plot_Wifi
#### File: Q3_Server_Wifi.js
This contains the NodeJS server that runs on the laptop

#### File: Q3_Web_Observation_Portal.html
This contains the html webpage observation portal that show all of the sensors and video

#### File: Sensor_Output.csv
This is the csv file which is an intermediary step between the ESP32, server, and html page

## Folder: Q3_udp_client_LED
### Sub-folder: main
#### File: udp_client_LED.c
This contains the ESP32 code for the LED which is triggered by the html button via the server

## Folder: Q3_udp_client_thermistor
### Sub-folder: main
#### File: udp_client_thermsitor.c
This contains the ESP32 code for the thermsitor data which is outputted to the server