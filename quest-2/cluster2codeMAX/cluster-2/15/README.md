#  Infrared Rangefinder

Author: Maxwell Bakalos

Date: 2023-02-24


### Summary
This code uses the IR Rangefinder powered by the USB pin (5v) on the ESP32. I have the GP2Y0A21YK model which can sense roughly from 10 to 80 centimeters. I copied down values from a graph on the datasheet and interpolate between them to get the most accurate distance reading possible with the information I have. Voltage is read in the ADC and then converted to a distance in cm.

### Sketches/Diagrams

![IR rangefinder (small)](https://user-images.githubusercontent.com/114166327/221328049-62a3ab28-3cb0-4dd9-b46c-a2a8ff02ba30.JPG)
![Q2-15 terminal (attempting to move paper closer at constant rate)](https://user-images.githubusercontent.com/114166327/221327992-6f4752d9-585f-4368-9ff2-c22371e60e5a.JPG)
<br>
I attempted to move a piece of paper closer to the sensor at a constant speed

### Modules, Tools, Source Used Including Attribution
 - The voltage & distance tables are from the graph in the GP2Y0A21YK IR Rangefinder datasheet
 - I used some of the code from the 13 skill (Thermistor) to help. Especially with the interpolation between datasheet table values

### Supporting Artifacts

